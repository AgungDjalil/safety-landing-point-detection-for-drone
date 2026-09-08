"""Autonomous PX4 offboard mission: arm, take off, fly to a waypoint, find a
safe landing spot there, and land on it.

The waypoint is given as ROS parameters in the ROS `map` frame (ENU: x east,
y north, z up), so the numbers can be read straight off RViz. The node
converts to PX4 local NED internally.

Once the waypoint is reached the mission hovers and watches the dynamic TF
`map -> safety_point`, which `segmentation_node/landing_circle` broadcasts for
the landing candidate it currently selects. When that point stops moving, the
mission locks onto it, flies above it, descends, and hands the touchdown to
PX4 (`VEHICLE_CMD_NAV_LAND`). If the point is lost or the perception pipeline
retargets mid-approach, the mission climbs back and scans again.

Run:
    ros2 run offboard_mission waypoint_node --ros-args \
        -p target_x:=5.0 -p target_y:=0.0 -p target_z:=3.0 -p use_sim_time:=true

The perception chain is started BY THIS NODE on arrival, not from launch:
`dbl_gng_cpu_node` alone was measured at ~701% CPU, and nothing consumes its
output until the vehicle is hovering over the area to scan. Set
`manage_perception:=false` to go back to running those nodes by hand.
"""
import json
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from offboard_mission.perception_supervisor import PerceptionSupervisor

from px4_offboard_lib.px4_helpers import (
    ArrivalDetector,
    build_vehicle_command,
    enu_to_ned,
    horizontal_distance,
    nan_filled_trajectory,
    ned_to_enu,
    now_us,
    position_trajectory,
)


class State(Enum):
    """Mission stages.

    WAIT_POSITION .. DESCEND run in order on a nominal flight. Two edges break
    the straight line: an aborted approach or descent returns to SCAN (the
    vehicle climbs back because SCAN streams the waypoint pose again), and any
    stage that gives up ends in HOLD.

    HOLD is the failure / idle state, not the normal ending. LANDED is.
    """

    WAIT_POSITION = 'WAIT_POSITION'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    GOTO = 'GOTO'
    SCAN = 'SCAN'
    APPROACH = 'APPROACH'
    DESCEND = 'DESCEND'
    LANDING = 'LANDING'
    LANDED = 'LANDED'
    HOLD = 'HOLD'


class WaypointMission(Node):
    # VehicleCommand IDs (px4_msgs/msg/VehicleCommand.msg)
    CMD_COMPONENT_ARM_DISARM = 400
    CMD_NAV_LAND = 21
    CMD_SET_NAV_STATE = 100001

    # VehicleStatus nav_state values
    NAV_OFFBOARD = 14
    NAV_AUTO_LAND = 18

    # How many times NAV_LAND may be re-sent before the mission reports that
    # PX4 never took it.
    MAX_LAND_COMMANDS = 3

    # PX4 refuses to enter offboard until setpoints are already flowing.
    MIN_SETPOINTS_BEFORE_OFFBOARD = 10

    # How often to re-send mode / arm commands while waiting for them to take.
    COMMAND_RESEND_S = 1.0

    def __init__(self):
        super().__init__('waypoint_mission')

        # ── Target, in ROS `map` frame (ENU) ─────────────────────────────────
        self._target_x = self.declare_parameter('target_x', 5.0).value
        self._target_y = self.declare_parameter('target_y', 0.0).value
        self._target_z = self.declare_parameter('target_z', 3.0).value

        self._takeoff_alt = float(self.declare_parameter('takeoff_alt', 2.5).value)
        self._target_yaw_deg = float(
            self.declare_parameter('target_yaw_deg', float('nan')).value)

        # ── Timing & tolerances ──────────────────────────────────────────────
        self._rate_hz = float(self.declare_parameter('offboard_rate', 20.0).value)
        arrive_tol = float(self.declare_parameter('arrive_tol_m', 0.5).value)
        takeoff_tol = float(self.declare_parameter('takeoff_tol_m', 0.3).value)
        settle_s = float(self.declare_parameter('settle_s', 1.0).value)

        self._auto_arm = bool(self.declare_parameter('auto_arm', True).value)
        self._position_timeout_s = float(
            self.declare_parameter('position_timeout_s', 15.0).value)

        # ── Landing-point search ─────────────────────────────────────────────
        # The mission reads the dynamic TF that landing_circle broadcasts for
        # its currently selected candidate. Defaults match that node's own
        # `odom_frame` / `safety_frame` parameters.
        self._map_frame = self.declare_parameter('map_frame', 'map').value
        self._safety_frame = self.declare_parameter(
            'safety_frame', 'safety_point').value

        # The frame this node broadcasts at the waypoint, and which
        # landing_circle is told to select against (its `base_frame`).
        self._scan_frame = self.declare_parameter(
            'scan_frame', 'scan_center').value
        # How far from the waypoint a landing point may be, measured
        # horizontally.
        #
        # Tied to what the camera can actually see, not picked by feel. The
        # OakD-Lite depth sensor has a 1.274 rad horizontal FOV and looks
        # straight down, so from height h above the surface it covers
        # h * tan(0.637) = 0.74 * h metres either side. At an 11 m waypoint
        # over ground ~2 m below the origin that is a 9.6 m half-width, and a
        # radius beyond it could never match anything anyway.
        #
        # Measured in the rubicon world: the ground directly under the
        # waypoint is not landable, and the nearest safe candidate sat 5.86 m
        # away. A 5 m radius rejected every candidate and the mission held
        # without landing — correct behaviour, but a limit tighter than the
        # sensor's own reach throws away usable ground for no benefit.
        self._scan_radius_m = float(
            self.declare_parameter('scan_radius_m', 8.0).value)

        # The frame broadcast at a point the mission has just abandoned, and
        # how close counts as the same point. Kept equal to landing_circle's
        # own reject_radius_m default so both agree on what "that one" means.
        self._reject_frame = self.declare_parameter(
            'reject_frame', 'reject_point').value
        self._reject_radius_m = float(
            self.declare_parameter('reject_radius_m', 1.5).value)

        # How long a rejection stands. It means "I just failed there", not
        # "that place is condemned": once an obstacle has moved on, the
        # registry's own blocked/recover logic is a better judge than a stale
        # memory held by the mission.
        self._reject_hold_s = float(
            self.declare_parameter('reject_hold_s', 30.0).value)

        # How long landing_circle collects candidates before publishing any
        # decision at all. The mission does NOT settle the TF a second time:
        # upstream withholds it until the choice is final, so a second wait
        # would add 5-8 s of hover without adding any confidence.
        self._scan_collect_s = float(
            self.declare_parameter('scan_collect_s', 10.0).value)
        # The budget now covers starting two processes as well as scanning:
        # ros2 run + node startup + the TF buffer filling all happen inside it.
        self._scan_timeout_s = float(
            self.declare_parameter('scan_timeout_s', 45.0).value)
        self._max_scan_attempts = int(
            self.declare_parameter('max_scan_attempts', 3).value)

        approach_tol = float(self.declare_parameter('approach_tol_m', 0.4).value)
        self._descend_speed_ms = float(
            self.declare_parameter('descend_speed_ms', 0.4).value)
        self._land_handoff_alt_m = float(
            self.declare_parameter('land_handoff_alt_m', 1.0).value)
        # Below this height above the surface the mission commits: it stops
        # acting on a lost or moved target and rides the descent out.
        #
        # Raised from 1.5 m after measurement. The depth camera looks straight
        # down and covers 1.475 * h metres across, so at 3.6 m it sees only
        # 5.3 m — about nine 0.6 m grid cells — and the fill-ratio and obstacle
        # tests stop being able to evaluate a 1.3 m disk at all. In flight the
        # detector duly stopped confirming the very zone the vehicle was
        # descending onto, at 3.64 m, while position tracking was still within
        # 0.46 m. A floor below the height where perception goes blind does not
        # protect anything; it just aborts good landings.
        #
        # State plainly what this costs: below this height nothing will react
        # to an obstacle that newly enters the zone. That protection is not
        # being given up — it is not available down there, and pretending
        # otherwise is the more dangerous choice.
        self._abort_floor_m = float(
            self.declare_parameter('abort_floor_m', 4.0).value)

        # How closely the vehicle must be following its own setpoint before
        # the commit floor is allowed to switch the abort checks off. See
        # `_should_abort` for why this guard exists.
        self._commit_track_tol_m = float(
            self.declare_parameter('commit_track_tol_m', 1.5).value)

        # Touching down further than this from the chosen point is reported as
        # a failure. Sized against safe_diameter (1.3 m, so a 0.65 m radius):
        # beyond about a metre the vehicle is no longer on the disk that was
        # examined and declared safe, whatever it is standing on instead.
        self._landing_miss_warn_m = float(
            self.declare_parameter('landing_miss_warn_m', 1.0).value)
        # How old the safety_point TF may be before the target counts as lost.
        #
        # Sized from measurement, not taste. landing_circle only broadcasts on
        # each point cloud it processes, and /plane_cpu was measured at 0.79 Hz
        # with gaps of 1.07-1.68 s over 119 frames. A 2.0 s limit left just
        # 0.32 s of headroom, so a single late frame would have read as a lost
        # target. 3.5 s tolerates two missed frames at that rate and still
        # notices a pipeline that has genuinely stopped within a few seconds.
        self._target_timeout_s = float(
            self.declare_parameter('target_timeout_s', 3.5).value)
        self._retarget_abort_m = float(
            self.declare_parameter('retarget_abort_m', 1.0).value)
        self._land_confirm_s = float(
            self.declare_parameter('land_confirm_s', 5.0).value)

        # Escape hatch for the first flights: prove the scan and the approach
        # before betting a vehicle on the automatic descent.
        self._enable_landing = bool(
            self.declare_parameter('enable_landing', True).value)

        # ── Perception processes ─────────────────────────────────────────────
        # Each entry is `ros_node_name: command`. The name is only used to skip
        # a node the operator already has running in another terminal.
        self._manage_perception = bool(
            self.declare_parameter('manage_perception', True).value)
        perception_commands = self.declare_parameter(
            'perception_commands',
            ['dbl_gng_cpu: ros2 run gng_node dbl_gng_cpu_node',
             'landing_circle: ros2 run segmentation_node landing_circle '
             '--ros-args -p input_topic:=/plane_cpu']).value
        self._perception_stop_grace_s = float(
            self.declare_parameter('perception_stop_grace_s', 3.0).value)

        # Penanda penerbangan, dicap ke tiap peristiwa. Tanpa ini data dari
        # penerbangan GNG dan penerbangan RANSAC bercampur di berkas yang sama
        # dan seluruh perbandingannya jadi sia-sia. Kosong = tidak dipakai.
        self._run_id = str(self.declare_parameter('run_id', '').value)

        # Settings that must not be duplicated into the command strings.
        #
        # base_frame is the important one. landing_circle picks the candidate
        # nearest its `base_frame`; pointing that at the drone means the
        # selection changes every time the drone moves, and a consumer reading
        # the safety_point TF cannot tell that apart from its target becoming
        # unsafe. Measured in flight: three aborted landings in a row while
        # every candidate was still safe. Pointing it at the scan centre, and
        # making the selection sticky, gives a TF that moves for one reason
        # only.
        extra_args = [
            '-p', 'base_frame:=%s' % self._scan_frame,
            '-p', 'sticky_target:=true',
            '-p', 'select_radius_m:=%.3f' % self._scan_radius_m,
            '-p', 'reject_frame:=%s' % self._reject_frame,
            '-p', 'reject_radius_m:=%.3f' % self._reject_radius_m,
            '-p', 'commit_after_s:=%.3f' % self._scan_collect_s,
        ]
        if bool(self.get_parameter('use_sim_time').value):
            extra_args += ['-p', 'use_sim_time:=true']

        self._perception = PerceptionSupervisor(
            perception_commands if self._manage_perception else [],
            extra_args=extra_args,
            list_running_nodes=self.get_node_names,
            logger=self.get_logger())

        # PX4 >= 1.16 with versioned topics publishes suffixed names
        # (/fmu/out/vehicle_local_position_v1). Keep these overridable so a
        # mismatch can be fixed without rebuilding.
        local_pos_topic = self.declare_parameter(
            'local_position_topic', '/fmu/out/vehicle_local_position').value
        status_topic = self.declare_parameter(
            'status_topic', '/fmu/out/vehicle_status').value

        # Z is ENU "up". A negative target means below the origin, which is
        # almost always a NED/ENU mix-up rather than an intent to fly a hole.
        if self._target_z <= 0.0:
            self.get_logger().warn(
                'target_z=%.2f is at or below the map origin. Remember target_z '
                'is ENU altitude (positive = up), not NED down.' % self._target_z)

        # ── Target in PX4 NED ────────────────────────────────────────────────
        self._target_ned = enu_to_ned(self._target_x, self._target_y, self._target_z)
        self._target_yaw = (math.radians(self._target_yaw_deg)
                            if not math.isnan(self._target_yaw_deg) else None)

        # ── State ────────────────────────────────────────────────────────────
        self._state = State.WAIT_POSITION
        self._setpoint_count = 0
        self._home_ned = None            # (n, e, d) captured once position is valid
        self._takeoff_detector = ArrivalDetector(takeoff_tol, settle_s)
        self._goto_detector = ArrivalDetector(arrive_tol, settle_s)
        self._approach_detector = ArrivalDetector(approach_tol, settle_s)
        self._descend_detector = ArrivalDetector(approach_tol, settle_s)
        self._scan_attempts = 0

        # ── Akuntansi metrik ──────────────────────────────────────────────────
        # Angka-angka ini sudah dihitung node ini sejak lama, tapi berhenti di
        # log konsol dan hilang begitu terminal ditutup. Untuk menilai algoritma
        # pencari titik pendaratan, galat pendaratanlah angka utamanya — ia
        # layak keluar sebagai data, bukan sebagai kalimat.
        self._abort_count = 0
        self._state_since_s = 0.0
        self._state_durations = {}
        self._commit_at_s = None      # detik saat safety_point pertama terlihat
        self._arrived_at_s = None     # detik saat SCAN dimulai
        self._scan_elapsed_s = 0.0
        self._scan_saw_target = False
        self._land_point = None          # locked (x, y, z) ENU landing target
        self._descend_z = 0.0            # ramped ENU altitude setpoint
        self._last_setpoint_ned = None   # last position setpoint commanded
        self._rejected_point = None      # ENU point the mission gave up on
        self._rejected_at_s = 0.0
        self._land_cmd_count = 0
        self._land_cmd_s = 0.0
        self._land_failure_reported = False

        # HOLD streams whatever pose it was entered with, not the waypoint:
        # holding after an aborted descent must keep the vehicle where it is,
        # not fly it back across the map.
        self._hold_ned = None

        self._pos = None                 # latest VehicleLocalPosition
        self._arming_state = None
        self._nav_state = None
        self._last_tick_ns = None
        self._last_command_s = -1e9

        # Baseline waktu SENGAJA belum diambil di sini. Dengan use_sim_time,
        # get_clock() masih mengembalikan 0 saat __init__ karena pesan /clock
        # pertama belum tiba; tick berikutnya sudah membaca waktu simulasi
        # penuh (mis. 329 s), sehingga elapsed langsung melampaui timeout apa
        # pun dan node menyerah pada tick pertama. Diambil di tick pertama saja,
        # ketika jamnya sudah pasti sahih.
        self._start_time = None
        self._timeout_reported = False

        # ── QoS ──────────────────────────────────────────────────────────────
        # PX4's uXRCE-DDS publishers are BEST_EFFORT. A RELIABLE subscriber
        # never matches them, so this must not be left at the rclpy default.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # PX4 >= 1.16 dengan topik berversi menerbitkan nama bersufiks _v1.
        # Berlangganan kedua-duanya sekaligus jauh lebih murah daripada membuat
        # pengguna menebak nama yang benar: berlangganan topik yang tidak ada
        # tidak berbiaya apa pun di ROS 2, dan hanya satu dari keduanya yang
        # akan benar-benar mengirim pesan.
        for topic in self._topic_variants(local_pos_topic):
            self.create_subscription(
                VehicleLocalPosition, topic, self._on_local_position, px4_qos)
        for topic in self._topic_variants(status_topic):
            self.create_subscription(
                VehicleStatus, topic, self._on_status, px4_qos)

        # TF is the mission's only view of the perception pipeline. It carries
        # exactly what is needed — landing_circle broadcasts map -> safety_point
        # only while it holds a selection, so the transform going stale IS the
        # "target lost" signal, with no extra topic to subscribe to.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Dynamic, not static: the scan centre is a position this node chooses
        # and could later move (a second waypoint, a search pattern). A frame
        # that can move at runtime does not belong on /tf_static.
        self._tf_broadcaster = TransformBroadcaster(self)

        self._pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self._pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Peristiwa misi sebagai JSON.
        #
        # TRANSIENT_LOCAL supaya perekam yang menyala terlambat tetap menerima
        # peristiwa yang sudah lewat. Sebuah penerbangan tidak bisa diulang
        # dengan murah, jadi kehilangan barisnya karena urutan penyalaan
        # terminal adalah kerugian yang tidak sepadan.
        self._pub_events = self.create_publisher(
            String, '/mission_events',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST,
                       depth=100))

        self._hold_ned = self._target_ned

        self.create_timer(1.0 / self._rate_hz, self._on_tick)

        self.get_logger().info(
            'WaypointMission: target map/ENU (%.2f, %.2f, %.2f) -> PX4 NED '
            '(%.2f, %.2f, %.2f) | takeoff_alt=%.2fm | tol=%.2fm settle=%.2fs | '
            'auto_arm=%s | sub %s, %s'
            % (self._target_x, self._target_y, self._target_z,
               self._target_ned[0], self._target_ned[1], self._target_ned[2],
               self._takeoff_alt, arrive_tol, settle_s,
               self._auto_arm, local_pos_topic, status_topic))

    # ── Subscriptions ────────────────────────────────────────────────────────

    @staticmethod
    def _topic_variants(topic):
        """Nama topik itu sendiri, plus varian berversi _v1 bila belum ada."""
        if topic.endswith('_v1'):
            return [topic]
        return [topic, topic + '_v1']

    def _on_local_position(self, msg):
        if self._pos is None:
            self.get_logger().info('VehicleLocalPosition mulai diterima.')
        self._pos = msg

    def _on_status(self, msg):
        self._arming_state = msg.arming_state
        self._nav_state = msg.nav_state

    # ── Helpers ──────────────────────────────────────────────────────────────

    # Two clocks are in play here, deliberately:
    #
    #   now_us()          WALL clock, for the `timestamp` field of outgoing PX4
    #                     messages. With UXRCE_DDS_SYNCT=1 the bridge maps the
    #                     companion's wall clock onto PX4's internal clock; a
    #                     simulation timestamp (which starts near zero under
    #                     use_sim_time) reads as decades stale and PX4 discards
    #                     the setpoint.
    #
    #   get_clock()       ROS clock, for DURATIONS: dt between ticks, elapsed
    #                     time for timeouts. Under use_sim_time these follow
    #                     simulation time, which is what we want — the arrival
    #                     dwell and the position timeout should track the
    #                     vehicle's motion, not the wall.
    #
    # Do not unify these. They answer different questions.

    def _elapsed_s(self):
        if self._start_time is None:
            return 0.0
        return (self.get_clock().now() - self._start_time).nanoseconds / 1e9

    def _send_command(self, command, p1=0.0, p2=0.0):
        cmd = build_vehicle_command(command, p1=p1, p2=p2)
        cmd.timestamp = now_us()
        self._pub_cmd.publish(cmd)

    def _publish_offboard_mode(self, position):
        msg = OffboardControlMode()
        msg.timestamp = now_us()
        msg.position = position
        msg.velocity = not position
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self._pub_offboard.publish(msg)

    def _publish_position_setpoint(self, ned, yaw=None):
        self._publish_offboard_mode(position=True)
        msg = position_trajectory(ned[0], ned[1], ned[2], yaw=yaw)
        msg.timestamp = now_us()
        self._pub_traj.publish(msg)
        self._last_setpoint_ned = (float(ned[0]), float(ned[1]), float(ned[2]))

    def _publish_hold_velocity(self):
        """Zero-velocity setpoint.

        Used before the vehicle's position is known. Streaming a POSITION
        setpoint at this stage would be dangerous: with no valid estimate the
        only coordinates available are zeros, which commands a flight to the
        local origin. Zero velocity means "stay put" regardless.
        """
        self._publish_offboard_mode(position=False)
        msg = nan_filled_trajectory(0.0, 0.0, 0.0, 0.0)
        msg.timestamp = now_us()
        self._pub_traj.publish(msg)

    def _emit_event(self, kind, **fields):
        """Terbitkan satu peristiwa misi sebagai JSON ke /mission_events.

        Dipisah dari log konsol dengan sengaja: log itu untuk dibaca manusia
        saat penerbangan berlangsung, peristiwa ini untuk dianalisis sesudahnya.
        Menggabungkan keduanya berarti salah satu harus mengalah, dan yang
        mengalah selalu yang tidak sedang ditonton.
        """
        payload = {
            'run_id': self._run_id,
            'kind': kind,
            't_s': round(self._elapsed_s(), 3),
            'state': self._state.value,
        }
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload)
        self._pub_events.publish(msg)

    def _transition(self, new_state):
        self.get_logger().info('%s -> %s' % (self._state.value, new_state.value))

        # Durasi tiap state diakumulasi, bukan sekadar dicatat sekali: SCAN bisa
        # dimasuki berkali-kali setelah abort, dan yang ingin diketahui adalah
        # total waktu yang dihabiskan di sana.
        now_s = self._elapsed_s()
        prev = self._state.value
        held_s = now_s - self._state_since_s
        self._state_durations[prev] = (
            self._state_durations.get(prev, 0.0) + held_s)
        self._state_since_s = now_s

        self._emit_event('transition',
                         to_state=new_state.value,
                         prev_state=prev,
                         prev_duration_s=round(held_s, 3))
        self._state = new_state

    def _position_valid(self):
        return (self._pos is not None
                and self._pos.xy_valid and self._pos.z_valid)

    # ── Main loop ────────────────────────────────────────────────────────────

    def _on_tick(self):
        if self._start_time is None:
            self._start_time = self.get_clock().now()

        self._publish_scan_center_tf()
        self._publish_reject_tf()

        now_ns = self.get_clock().now().nanoseconds
        if self._last_tick_ns is None:
            dt_s = 1.0 / self._rate_hz
        else:
            dt_s = (now_ns - self._last_tick_ns) / 1e9
            if dt_s <= 0.0 or dt_s > 1.0:      # clock reset or long stall
                dt_s = 1.0 / self._rate_hz
        self._last_tick_ns = now_ns

        handler = {
            State.WAIT_POSITION: self._tick_wait_position,
            State.ARMING: self._tick_arming,
            State.TAKEOFF: self._tick_takeoff,
            State.GOTO: self._tick_goto,
            State.SCAN: self._tick_scan,
            State.APPROACH: self._tick_approach,
            State.DESCEND: self._tick_descend,
            State.LANDING: self._tick_landing,
            State.LANDED: self._tick_landed,
            State.HOLD: self._tick_hold,
        }[self._state]
        handler(dt_s)

        self._setpoint_count += 1

    def _tick_wait_position(self, dt_s):
        self._publish_hold_velocity()

        if not self._position_valid():
            if (self._elapsed_s() > self._position_timeout_s
                    and not self._timeout_reported):
                self._timeout_reported = True
                self.get_logger().error(
                    'No valid VehicleLocalPosition after %.1fs. NOT arming. '
                    'Check the topic name (PX4 >= 1.16 publishes versioned '
                    'names like /fmu/out/vehicle_local_position_v1) and that '
                    'the uXRCE-DDS agent is running.'
                    % self._position_timeout_s)
            return

        if self._setpoint_count < self.MIN_SETPOINTS_BEFORE_OFFBOARD:
            return

        self._home_ned = (self._pos.x, self._pos.y, self._pos.z)
        self.get_logger().info(
            'Home captured at NED (%.2f, %.2f, %.2f)' % self._home_ned)
        self._transition(State.ARMING)

    def _tick_arming(self, dt_s):
        self._publish_hold_velocity()

        armed = (self._arming_state == VehicleStatus.ARMING_STATE_ARMED)
        offboard = (self._nav_state == self.NAV_OFFBOARD)

        if armed and offboard:
            self._takeoff_detector.reset()
            self._transition(State.TAKEOFF)
            return

        # Re-send at 1 Hz: a single command can be missed, and PX4 rejects the
        # mode switch outright while it considers the setpoint stream too young.
        if self._elapsed_s() - self._last_command_s < self.COMMAND_RESEND_S:
            return
        self._last_command_s = self._elapsed_s()

        if not offboard:
            self._send_command(self.CMD_SET_NAV_STATE, p1=float(self.NAV_OFFBOARD))
        if not armed:
            if self._auto_arm:
                self._send_command(self.CMD_COMPONENT_ARM_DISARM, p1=1.0)
            else:
                self.get_logger().info(
                    'auto_arm is false — waiting for the operator to arm.',
                    throttle_duration_sec=5.0)

    def _tick_takeoff(self, dt_s):
        target = (self._home_ned[0], self._home_ned[1], -self._takeoff_alt)
        self._publish_position_setpoint(target)

        # Only the vertical axis matters here; the horizontal target is home.
        dz = abs(self._pos.z - target[2]) if self._pos is not None else float('inf')
        if self._takeoff_detector.update(dz, dt_s):
            self._goto_detector.reset()
            self._transition(State.GOTO)

    def _tick_goto(self, dt_s):
        self._publish_position_setpoint(self._target_ned, yaw=self._target_yaw)

        if self._pos is None:
            return
        dist = math.sqrt(
            (self._pos.x - self._target_ned[0]) ** 2
            + (self._pos.y - self._target_ned[1]) ** 2
            + (self._pos.z - self._target_ned[2]) ** 2)

        self.get_logger().info(
            'GOTO: %.2f m to target' % dist, throttle_duration_sec=2.0)

        if self._goto_detector.update(dist, dt_s):
            self.get_logger().info(
                'Arrived at map/ENU (%.2f, %.2f, %.2f). Starting the '
                'landing-point search.'
                % (self._target_x, self._target_y, self._target_z))
            self._begin_scan()

    # ── Landing-point search ─────────────────────────────────────────────────

    def _publish_scan_center_tf(self):
        """Broadcast `map -> scan_frame` at the waypoint, every tick.

        Published from the very first tick rather than on arrival: it costs a
        few bytes, and it guarantees the frame is already in every TF buffer
        before landing_circle starts and looks it up. landing_circle resolves
        the transform at the point cloud's own stamp, so a frame that only
        appears at the same instant it is needed would be missed for the first
        few frames.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._scan_frame
        t.transform.translation.x = float(self._target_x)
        t.transform.translation.y = float(self._target_y)
        t.transform.translation.z = float(self._target_z)
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

    def _publish_reject_tf(self):
        """Broadcast `map -> reject_frame` at the point the mission gave up on.

        landing_circle holds its selection sticky, which is what stops the TF
        jittering — but it also means nothing would ever make it let go of a
        point the mission has already tried and abandoned. The mission would
        climb, rescan, and lock the very same point, which from outside looks
        like a landing target frozen in place.

        Only the mission knows a landing attempt failed, so only the mission
        can say so. It says it here, through the same channel it already uses
        for the scan centre, and stops broadcasting once the rejection expires.
        """
        if self._rejected_point is None:
            return
        if self._elapsed_s() - self._rejected_at_s > self._reject_hold_s:
            self.get_logger().info(
                'Rejection of map/ENU (%.2f, %.2f, %.2f) has expired after '
                '%.0fs — that point may be chosen again.'
                % (self._rejected_point + (self._reject_hold_s,)))
            self._rejected_point = None
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._reject_frame
        t.transform.translation.x = float(self._rejected_point[0])
        t.transform.translation.y = float(self._rejected_point[1])
        t.transform.translation.z = float(self._rejected_point[2])
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

    def _is_rejected(self, point):
        """True if `point` is the one the mission just walked away from.

        Duplicated on purpose: landing_circle is told to exclude it too, but
        this node must not depend on another process having been started with
        the right parameters in order to keep its own promise.
        """
        if self._rejected_point is None or point is None:
            return False
        return math.dist(point, self._rejected_point) <= self._reject_radius_m

    def _safety_point(self):
        """Latest `map -> safety_point` translation as an ENU tuple, or None.

        None means "no landing target the mission may act on". Two different
        causes collapse into it deliberately, because the mission reacts to
        both the same way:

          * the transform was never broadcast, or landing_circle stopped
            broadcasting it — it only publishes while it holds a selection, so
            an empty registry or an all-blocked one silently ends the stream;
          * the last broadcast is older than `target_timeout_s`, i.e. the
            pipeline is stalled and whatever it last said is no longer a claim
            about the present.

        Freshness is measured on the ROS clock, not `now_us()`. Under
        use_sim_time the transform stamps are simulation time; comparing them
        against a wall-clock reading would make every target look decades old.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._safety_frame, Time())
        except TransformException:
            return None

        age_s = (self.get_clock().now()
                 - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        if age_s > self._target_timeout_s:
            # Distinguish this from "no TF at all" in the log. A transform that
            # exists but is permanently stale usually means the two nodes
            # disagree about use_sim_time, and "the TF never appeared" would
            # send the operator looking in exactly the wrong place.
            self.get_logger().warn(
                '%s TF is %.1fs old (limit %.1fs) — treating the landing target '
                'as lost. If this never clears, check that landing_circle and '
                'this node agree on use_sim_time.'
                % (self._safety_frame, age_s, self._target_timeout_s),
                throttle_duration_sec=5.0)
            return None

        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def _height_above_surface(self):
        """Vehicle altitude above the locked landing point, in metres.

        Infinity when either is unknown — that keeps the abort checks ENABLED
        while the situation is unclear, which is the safe direction to fail.
        """
        if self._pos is None or self._land_point is None:
            return float('inf')
        z_up = ned_to_enu(self._pos.x, self._pos.y, self._pos.z)[2]
        return z_up - self._land_point[2]

    def _tracking_error_m(self):
        """How far the vehicle is from the position it was last told to hold,
        or infinity when that cannot be answered yet."""
        if self._pos is None or self._last_setpoint_ned is None:
            return float('inf')
        return math.dist(
            (self._pos.x, self._pos.y, self._pos.z), self._last_setpoint_ned)

    def _tracking_ok(self):
        """True only when the vehicle is demonstrably following its setpoint.

        Unknown counts as NOT ok. Everything that reads this uses it to decide
        whether it is safe to stop checking, and 'we have no evidence' must
        never be the reason protection switches itself off.
        """
        if self._pos is None or self._last_setpoint_ned is None:
            return False
        return self._tracking_error_m() <= self._commit_track_tol_m

    def _should_abort(self):
        """Reason to give up on the locked landing point, or None.

        Both signals come off the same TF:

          * it is gone or stale (see `_safety_point`) — the pipeline no longer
            confirms any landing target at all;
          * it moved more than `retarget_abort_m` — landing_circle switched to
            a different candidate, which is exactly what it does when the one
            we locked becomes blocked by a moving object or drops out of the
            registry. The old point is no longer being vouched for.

        Neither is acted on below `abort_floor_m`. The depth camera looks
        straight down, so the lower the vehicle gets the less surface it sees;
        near the ground the plane leaves the sensor's useful range and the
        target reliably goes stale. Without a commit floor every descent would
        abort at its final metre and the mission would loop for ever instead
        of touching down.
        """
        if self._land_point is None:
            return None

        # The commit floor only applies while the vehicle is actually flying
        # the profile it was given. Measured in flight: it dropped ~9 m in two
        # seconds, which put it under the floor at once, and the mission then
        # committed to a landing it could no longer see and came to rest 12 m
        # away. Committing is right when the vehicle is tracking well and has
        # merely run out of camera view; a vehicle metres from its own
        # setpoint is not in that situation, and giving up the checks there is
        # exactly backwards.
        if (self._height_above_surface() <= self._abort_floor_m
                and self._tracking_ok()):
            return None

        point = self._safety_point()
        if point is None:
            return 'no fresh %s TF (landing_circle has no selection)' % (
                self._safety_frame,)

        moved = math.dist(point, self._land_point)
        if moved > self._retarget_abort_m:
            return ('target moved %.2f m (limit %.2f m) — landing_circle '
                    'retargeted' % (moved, self._retarget_abort_m))
        return None

    def _begin_scan(self):
        """Enter (or re-enter) SCAN with a clean slate."""
        if self._arrived_at_s is None:
            self._arrived_at_s = self._elapsed_s()
        self._scan_attempts += 1
        self._scan_elapsed_s = 0.0
        self._scan_saw_target = False
        self._land_point = None
        self._transition(State.SCAN)

        # Only on the first attempt. A re-scan after an aborted approach finds
        # the processes still up, and the candidate registry they have built by
        # then is worth keeping — restarting would throw it away.
        if self._scan_attempts == 1 and self._manage_perception:
            started, skipped = self._perception.start()
            self.get_logger().info(
                'Perception started on arrival: %d process(es) launched, '
                '%d already running (%s).'
                % (len(started), len(skipped), ', '.join(skipped) or 'none'))

        self.get_logger().info(
            'SCAN attempt %d/%d: hovering at the waypoint. landing_circle '
            'collects for %.1fs before it publishes %s at all; the mission '
            'locks on as soon as that frame appears.'
            % (self._scan_attempts, self._max_scan_attempts,
               self._scan_collect_s, self._safety_frame))

    def _abort_to_scan(self, reason):
        self.get_logger().warn(
            'Aborting %s: %s' % (self._state.value, reason))

        self._abort_count += 1
        self._emit_event('abort', reason=reason,
                         abort_count=self._abort_count,
                         point=list(self._land_point) if self._land_point else None)

        # Remember it before _begin_scan clears _land_point, or the mission
        # would have nothing left to refuse.
        if self._land_point is not None:
            self._rejected_point = self._land_point
            self._rejected_at_s = self._elapsed_s()
            self.get_logger().info(
                'Not returning to map/ENU (%.2f, %.2f, %.2f) for the next '
                '%.0fs. Looking for a different landing point.'
                % (self._rejected_point + (self._reject_hold_s,)))

        if self._scan_attempts >= self._max_scan_attempts:
            self.get_logger().error(
                'Landing abandoned after %d attempts (max_scan_attempts=%d). '
                'Holding position; PX4 is still in offboard and the vehicle is '
                'still airborne.'
                % (self._scan_attempts, self._max_scan_attempts))
            self._enter_hold(self._current_ned_or_target())
            return

        self._begin_scan()

    def _current_ned_or_target(self):
        """Where to hold: the vehicle's own position if it is known, else the
        waypoint. Holding at a stale waypoint after an aborted descent would
        command a flight back across the map."""
        if self._pos is None:
            return self._target_ned
        return (self._pos.x, self._pos.y, self._pos.z)

    def _enter_hold(self, ned):
        self._hold_ned = ned
        self._transition(State.HOLD)
        # Misi yang menyerah tetap menghasilkan data. `outcome` dan
        # `abort_count` pada penerbangan yang tidak jadi mendarat adalah
        # hasilnya, bukan ketiadaan hasil.
        self._emit_summary('gave_up')

    def _tick_scan(self, dt_s):
        # Streaming the waypoint pose is what makes a re-entry from DESCEND
        # climb back on its own — no separate recovery state is needed.
        self._publish_position_setpoint(self._target_ned, yaw=self._target_yaw)
        self._scan_elapsed_s += dt_s

        failure = self._perception.poll()
        if failure is not None:
            self.get_logger().error(
                'Perception died during the scan: %s. Holding at the waypoint; '
                'not landing.' % failure)
            self._enter_hold(self._target_ned)
            return

        # The TF existing IS the decision. landing_circle publishes nothing
        # until it has collected for scan_collect_s, so there is no settling
        # left for this node to do — only a check that the mission is not being
        # handed back a point it already walked away from.
        point = self._safety_point()

        if point is not None and not self._scan_saw_target:
            self._scan_saw_target = True
            self.get_logger().info(
                'SCAN: %s published at map/ENU (%.2f, %.2f, %.2f) — decision '
                'is final upstream.' % ((self._safety_frame,) + point))

            # Sekali saja, pada kemunculan PERTAMA: yang diukur adalah berapa
            # lama pipeline butuh untuk berani memutuskan, bukan berapa lama
            # ia butuh setelah sebuah abort mengulang prosesnya.
            if self._commit_at_s is None:
                self._commit_at_s = self._elapsed_s()
                self._emit_event('commit', point=list(point),
                                 time_to_commit_s=round(
                                     self._commit_at_s
                                     - (self._arrived_at_s or 0.0), 3))

        if point is not None and self._is_rejected(point):
            # landing_circle is told to exclude it too, so this should only be
            # seen in the moment before it processes the new reject frame.
            self.get_logger().warn(
                'Ignoring map/ENU (%.2f, %.2f, %.2f): this mission already '
                'abandoned it.' % point, throttle_duration_sec=5.0)
            point = None

        if point is not None:
            self._land_point = point
            self.get_logger().info(
                'Landing point locked at map/ENU (%.2f, %.2f, %.2f) after '
                '%.1fs in SCAN.' % (point + (self._scan_elapsed_s,)))
            self._approach_detector.reset()
            self._transition(State.APPROACH)
            return

        if self._scan_elapsed_s > self._scan_timeout_s:
            # Retrying the same hover at the same pose would repeat the same
            # observation, so this does not consume an attempt — it ends the
            # mission. max_scan_attempts governs aborted approaches, where the
            # situation genuinely changed.
            self.get_logger().error(
                'No landing point after %.1fs. %s Holding at the waypoint; '
                'not landing.'
                % (self._scan_timeout_s,
                   ('The %s TF never appeared — is landing_circle running, is '
                    'it finding candidates, and is scan_collect_s (%.1fs) well '
                    'inside this timeout?'
                    % (self._safety_frame, self._scan_collect_s))
                   if not self._scan_saw_target
                   else 'The only points offered were ones this mission had '
                        'already abandoned.'))
            self._enter_hold(self._target_ned)

    def _tick_approach(self, dt_s):
        # Fly over the landing point at the waypoint's altitude: cross first,
        # descend second, so the vehicle never cuts a diagonal through
        # whatever the point was chosen to avoid.
        target_ned = enu_to_ned(
            self._land_point[0], self._land_point[1], self._target_z)
        self._publish_position_setpoint(target_ned, yaw=self._target_yaw)

        reason = self._should_abort()
        if reason is not None:
            self._abort_to_scan(reason)
            return

        if self._pos is None:
            return
        dist = math.dist(
            (self._pos.x, self._pos.y, self._pos.z), target_ned)
        self.get_logger().info(
            'APPROACH: %.2f m to the landing point' % dist,
            throttle_duration_sec=2.0)

        if not self._approach_detector.update(dist, dt_s):
            return

        if not self._enable_landing:
            self.get_logger().info(
                'Above the landing point at map/ENU (%.2f, %.2f, %.2f). '
                'enable_landing is false — holding here instead of descending.'
                % (self._land_point[0], self._land_point[1], self._target_z))
            self._enter_hold(target_ned)
            return

        self._descend_z = ned_to_enu(self._pos.x, self._pos.y, self._pos.z)[2]
        self._descend_detector.reset()
        self._transition(State.DESCEND)

    def _tick_descend(self, dt_s):
        # Ramp the altitude setpoint instead of commanding the floor outright.
        # A step command hands PX4 a large error and it descends as fast as its
        # own limits allow; a ramp keeps the rate at descend_speed_ms and keeps
        # every intermediate tick abortable.
        floor_z = self._land_point[2] + self._land_handoff_alt_m
        self._descend_z = max(
            floor_z, self._descend_z - self._descend_speed_ms * dt_s)

        target_ned = enu_to_ned(
            self._land_point[0], self._land_point[1], self._descend_z)
        self._publish_position_setpoint(target_ned, yaw=self._target_yaw)

        reason = self._should_abort()
        if reason is not None:
            self._abort_to_scan(reason)
            return

        if self._pos is None:
            return
        dist = math.dist(
            (self._pos.x, self._pos.y, self._pos.z), target_ned)
        self.get_logger().info(
            'DESCEND: %.2f m above the surface, %.2f m to the setpoint'
            % (self._height_above_surface(), dist),
            throttle_duration_sec=2.0)

        at_floor = self._descend_z <= floor_z + 1e-3
        if at_floor and self._descend_detector.update(dist, dt_s):
            self.get_logger().info(
                'At the handoff altitude (%.2f m above the surface). Sending '
                'NAV_LAND; PX4 owns the touchdown from here.'
                % self._land_handoff_alt_m)
            self._send_land()
            self._transition(State.LANDING)

    def _send_land(self):
        self._send_command(self.CMD_NAV_LAND)
        self._land_cmd_count += 1
        self._land_cmd_s = self._elapsed_s()

    def _tick_landing(self, dt_s):
        """Deliberately publishes NOTHING.

        NAV_LAND takes PX4 out of offboard mode. Continuing to stream
        setpoints would keep asking it to come back, fighting the autopilot's
        own landing controller for the vehicle mid-descent.
        """
        if self._arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self._report_touchdown()
            self._perception.stop(self._perception_stop_grace_s)
            self._transition(State.LANDED)
            return

        if self._nav_state == self.NAV_AUTO_LAND:
            self.get_logger().info(
                'PX4 in AUTO_LAND — waiting for touchdown and disarm.',
                throttle_duration_sec=5.0)
            return

        if self._elapsed_s() - self._land_cmd_s < self._land_confirm_s:
            return

        if self._land_cmd_count >= self.MAX_LAND_COMMANDS:
            if not self._land_failure_reported:
                self._land_failure_reported = True
                self.get_logger().error(
                    'PX4 did not enter AUTO_LAND after %d NAV_LAND commands '
                    '(nav_state=%s). No setpoints are being sent, so the '
                    'offboard-loss failsafe will take over. Land manually.'
                    % (self.MAX_LAND_COMMANDS, self._nav_state))
            return

        self.get_logger().warn(
            'NAV_LAND not acknowledged after %.1fs (nav_state=%s). Re-sending.'
            % (self._land_confirm_s, self._nav_state))
        self._send_land()

    def _emit_summary(self, outcome, touchdown=None, landing_error_m=None):
        """Satu peristiwa penutup berisi seluruh angka penilaian misi.

        Diterbitkan baik saat mendarat maupun saat menyerah ke HOLD. Misi yang
        gagal justru yang paling perlu terekam: `outcome` dan `abort_count`
        pada penerbangan yang tidak jadi mendarat adalah datanya, bukan
        ketiadaan data.
        """
        # Tutup buku state yang sedang berjalan supaya durasinya tidak hilang.
        now_s = self._elapsed_s()
        durations = dict(self._state_durations)
        cur = self._state.value
        durations[cur] = (durations.get(cur, 0.0)
                          + (now_s - self._state_since_s))

        self._emit_event(
            'summary',
            outcome=outcome,
            target=[self._target_x, self._target_y, self._target_z],
            chosen_point=list(self._land_point) if self._land_point else None,
            touchdown=list(touchdown) if touchdown else None,
            landing_error_m=(round(landing_error_m, 4)
                             if landing_error_m is not None else None),
            time_to_commit_s=(round(self._commit_at_s - self._arrived_at_s, 3)
                              if (self._commit_at_s is not None
                                  and self._arrived_at_s is not None) else None),
            time_to_arrive_s=(round(self._arrived_at_s, 3)
                              if self._arrived_at_s is not None else None),
            total_time_s=round(now_s, 3),
            scan_attempts=self._scan_attempts,
            abort_count=self._abort_count,
            state_durations_s={k: round(v, 3) for k, v in durations.items()},
        )

    def _report_touchdown(self):
        """Say where the vehicle actually came down, not merely that it did.

        A previous flight logged 'Mission complete' after coming to rest 12.0 m
        from the point it had chosen and examined. Landing somewhere is not the
        goal; landing on the surface that was checked is.
        """
        if self._pos is None:
            self.get_logger().warn(
                'Landed and disarmed, but no position estimate is available, '
                'so the touchdown point could not be checked against the '
                'landing point at map/ENU (%.2f, %.2f, %.2f).'
                % self._land_point)
            self._emit_summary('landed_position_unknown')
            return

        here = ned_to_enu(self._pos.x, self._pos.y, self._pos.z)
        miss = horizontal_distance(here, self._land_point)

        if miss > self._landing_miss_warn_m:
            self.get_logger().error(
                'Landed and disarmed at map/ENU (%.2f, %.2f, %.2f), but that '
                'is %.2f m from the landing point (%.2f, %.2f, %.2f) this '
                'mission chose — more than landing_miss_warn_m=%.2f. The '
                'vehicle is NOT on the surface that was examined and declared '
                'safe.'
                % (here + (miss,) + self._land_point
                   + (self._landing_miss_warn_m,)))
            self._emit_summary('landed_off_target', touchdown=here,
                               landing_error_m=miss)
            return

        self.get_logger().info(
            'Landed and disarmed at map/ENU (%.2f, %.2f, %.2f), %.2f m from '
            'the chosen landing point. Mission complete.'
            % (here + (miss,)))
        self._emit_summary('landed', touchdown=here, landing_error_m=miss)

    def _tick_landed(self, dt_s):
        """Terminal. The vehicle is down and disarmed; there is nothing left to
        command, and streaming anything here could re-arm the offboard path."""

    def _tick_hold(self, dt_s):
        """Station-keep at whatever pose HOLD was entered with.

        Perception is deliberately left running here. HOLD means the mission
        gave up on landing, which is exactly when the operator wants to look at
        /landing_candidates in RViz and see what the pipeline was seeing.
        shutdown() still stops the processes on Ctrl+C.
        """
        self._publish_position_setpoint(self._hold_ned, yaw=self._target_yaw)

    # ── Shutdown ─────────────────────────────────────────────────────────────

    def shutdown(self):
        """Stop streaming and let PX4 take over.

        Deliberately does NOT disarm: disarming an airborne vehicle drops it
        out of the sky. Cutting the setpoint stream makes PX4 trip its own
        offboard-loss failsafe (COM_OF_LOSS_T), which is the behaviour a
        vehicle in flight should get.
        """
        # First, because it is the part that outlives this process if missed:
        # the children are in their own process groups and would keep running
        # (GNG at ~700% CPU) with no terminal that owns them.
        self._perception.stop(self._perception_stop_grace_s)

        airborne = (State.TAKEOFF, State.GOTO, State.SCAN, State.APPROACH,
                    State.DESCEND, State.LANDING, State.HOLD)
        if self._state in airborne:
            self.get_logger().warn(
                'Shutting down while airborne — setpoint stream stops here and '
                'PX4 offboard-loss failsafe takes over. Not disarming.')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
