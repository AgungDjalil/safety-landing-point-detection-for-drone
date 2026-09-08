"""Keyboard-driven PX4 offboard velocity control node.

Lets the operator fly the SITL drone with the keyboard by streaming PX4
offboard velocity setpoints over MicroXRCE-DDS.

Topic names are constants (choice A) matching PX4's actual MicroXRCE-DDS
topic names (verified live):
    Subscribe: /fmu/out/vehicle_odometry, /fmu/out/vehicle_status_v1,
               /fmu/out/vehicle_command_ack
    Publish:   /fmu/in/trajectory_setpoint,
               /fmu/in/vehicle_command,
               /fmu/in/offboard_control_mode
"""
import atexit
import math
import os
import sys
import termios
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleOdometry,
    VehicleStatus,
)

# Pure PX4 helpers live in the px4_offboard_lib package so this node and the
# autonomous waypoint node share one implementation. They are unit-tested
# there, in src/px4_offboard_lib/test/test_helpers.py.
from px4_offboard_lib.px4_helpers import (
    body_to_ned,
    build_vehicle_command,
    nan_filled_trajectory,
    now_us,
    yaw_from_quaternion,
)


# ---------------------------------------------------------------------------
# Keymap
# ---------------------------------------------------------------------------

# Keys whose continued hold produces a velocity component.
HELD_KEYS = {
    'w': ('fwd',  +1.0),
    's': ('fwd', -1.0),
    'a': ('strafe', -1.0),
    'd': ('strafe', +1.0),
    'r': ('alt',   +1.0),   # up: NED z is negative up, sign applied in node
    'f': ('alt',   -1.0),
    'q': ('yaw',  +1.0),    # yaw CCW
    'e': ('yaw',  -1.0),    # yaw CW
    ' ': ('hover', None),   # space → zero all velocities
}

# Single-press keys that trigger a discrete VehicleCommand / mode switch.
ACTION_KEYS = {
    'i': 'arm',
    'k': 'disarm',
    't': 'takeoff',
    'l': 'land',
    'o': 'offboard',
    'h': 'hold',
}


# ---------------------------------------------------------------------------
# KeyboardReader — raw non-blocking termios reader on a daemon thread
# ---------------------------------------------------------------------------

class KeyboardReader:
    """Reads single keystrokes from stdin in raw, non-blocking mode.

    Terminal raw mode does NOT emit key-release events, so we treat a key
    as "held" only if it was seen within the last `hold_timeout` seconds.
    The terminal's auto-repeat (~30 Hz) refreshes the timestamp while the
    key is physically held; when released, the timestamp stops refreshing
    and the key expires. This gives a natural "hold to move, release to
    stop" feel without needing OS key-release events.

    Discrete action keys (arm/disarm/takeoff/land/mode) are queued once
    per press via a simple de-duplication gate (ignore repeats within
    `action_debounce` seconds).

    The terminal is restored to its original state on exit via atexit and
    on explicit stop().
    """

    HOLD_TIMEOUT = 0.20        # seconds a key stays "held" after last press
    ACTION_DEBOUNCE = 0.30     # min seconds between identical action presses

    def __init__(self):
        self._lock = threading.Lock()
        # key -> last_seen monotonic time
        self._held: dict[str, float] = {}
        # action name -> last fire time (for debounce)
        self._action_last: dict[str, float] = {}
        # FIFO of discrete actions to dispatch
        self._actions: list[str] = []
        self._stop = threading.Event()
        self._fd = sys.stdin.fileno()
        try:
            self._old_attrs = termios.tcgetattr(self._fd)
        except termios.error:
            self._old_attrs = None
        self._thread: threading.Thread | None = None

    def start(self):
        if self._old_attrs is None:
            return  # no TTY; nothing to read
        atexit.register(self.restore_terminal)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.2)
        self.restore_terminal()

    def restore_terminal(self):
        if self._old_attrs is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attrs)
            except termios.error:
                pass

    def _run(self):
        # Switch stdin to raw mode (no echo, no line buffering, no signals).
        import tty
        tty.setraw(self._fd)
        while not self._stop.is_set():
            ch = self._read_char()
            if ch is None:
                self._stop.wait(0.01)
                continue
            self._handle_char(ch)

    def _read_char(self):
        """Read one character from stdin in non-blocking mode.

        Sets VMIN=0 VTIME=0 for a strictly non-blocking read, then restores
        VMIN=1 so the raw-mode loop doesn't spin. Returns the decoded key
        or None if nothing was pressed.
        """
        try:
            attrs = termios.tcgetattr(self._fd)
        except termios.error:
            return None
        # c_cc indices: VMIN=4, VTIME=5 on Linux.
        attrs[6][4] = 0  # VMIN
        attrs[6][5] = 0  # VTIME
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        try:
            data = os.read(self._fd, 1)
        except (BlockingIOError, OSError):
            return None
        if not data:
            return None
        return data.decode('latin-1')

    def _handle_char(self, ch):
        # Recognize Ctrl-C (0x03) as a graceful stop request.
        if ch == '\x03':
            self._stop.set()
            return
        low = ch.lower()
        now = time.monotonic()
        with self._lock:
            if low in HELD_KEYS:
                self._held[low] = now
            elif low in ACTION_KEYS:
                action = ACTION_KEYS[low]
                last = self._action_last.get(action, 0.0)
                if now - last >= self.ACTION_DEBOUNCE:
                    self._actions.append(action)
                    self._action_last[action] = now

    def get_held(self) -> dict:
        """Return a snapshot of the held movement keys as a dict of
        axis -> signed magnitude, with opposing keys cancelling out.

        A key counts as held only if it was seen within HOLD_TIMEOUT.
        """
        now = time.monotonic()
        with self._lock:
            live = {k: t for k, t in self._held.items()
                    if now - t <= self.HOLD_TIMEOUT}
            # Expire stale keys so the dict doesn't grow.
            self._held = live
        return {
            'fwd': net_axis(live, 'fwd'),
            'strafe': net_axis(live, 'strafe'),
            'alt': net_axis(live, 'alt'),
            'yaw': net_axis(live, 'yaw'),
            'hover': ' ' in live,
        }

    def pop_action(self):
        """Return the next discrete action ('arm', 'disarm', 'takeoff',
        'land', 'offboard', 'hold') or None if the queue is empty."""
        with self._lock:
            if self._actions:
                return self._actions.pop(0)
            return None

    def clear_held(self):
        with self._lock:
            self._held.clear()


def held_to_axis(held_set):
    """Helper: map each held key to its (axis, magnitude) tuple."""
    return {k: HELD_KEYS[k] for k in held_set if k in HELD_KEYS}


def net_axis(held_set, axis):
    """Sum signed magnitudes of all held keys belonging to `axis`."""
    total = 0.0
    for k, (ax, m) in held_to_axis(held_set).items():
        if ax == axis:
            total += m
    return total


# ---------------------------------------------------------------------------
# OffboardController — the ROS node
# ---------------------------------------------------------------------------

class OffboardController(Node):
    """Streams PX4 offboard velocity setpoints built from keyboard input."""

    # PX4 nav_state constants (from VehicleStatus.msg)
    NAV_OFFBOARD = 14
    NAV_AUTO_LOITER = 4

    # VehicleCommand constants (from VehicleCommand.msg)
    CMD_COMPONENT_ARM_DISARM = 400
    CMD_NAV_LAND = 21
    CMD_SET_NAV_STATE = 100001

    def __init__(self):
        super().__init__('keyboard_offboard_node')

        # Tunable parameters (topics are constants — choice A).
        self.declare_parameter('linear_speed', 0.8)
        self.declare_parameter('yaw_speed', 0.5)
        self.declare_parameter('takeoff_alt', 2.5)
        self.declare_parameter('offboard_rate', 20)

        self._linear_speed = self.get_parameter('linear_speed').value
        self._yaw_speed = self.get_parameter('yaw_speed').value
        self._takeoff_alt = self.get_parameter('takeoff_alt').value
        rate_hz = self.get_parameter('offboard_rate').value

        # Cached state from PX4 subscriptions.
        self._yaw = 0.0
        self._altitude_ned = 0.0   # current z in NED (m), negative = up
        self._arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self._nav_state = 0

        # Offboard engagement bookkeeping.
        self._setpoint_count = 0
        self._streaming = True
        # Takeoff sub-state: when active, we send a position z-target instead
        # of velocity and monitor altitude until within tolerance.
        self._takeoff_active = False
        self._takeoff_target_z = 0.0   # NED z target (= -takeoff_alt)

        # Publishers.
        self._pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self._pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscribers. PX4 publishes with BEST_EFFORT reliability + KEEP_LAST,
        # so we must match or relax our QoS to actually receive messages.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self._on_odom, px4_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._on_status, px4_qos)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self._on_cmd_ack, px4_qos)

        # The offboard timer — heart of the node.
        self._timer = self.create_timer(1.0 / float(rate_hz), self._on_tick)

        self._keyboard = KeyboardReader()
        self._keyboard.start()

        self.get_logger().info(
            'keyboard_offboard_node ready.\n'
            '  HOLD to move: W/S fwd/back  A/D strafe  R/F up/down  Q/E yaw  Space=hover\n'
            '  PRESS once:   I=arm  K=disarm  T=takeoff  L=land  O=offboard  H=hold\n'
            '  Ctrl-C to quit (auto-disarm + terminal restore)')

    # --- subscription callbacks -------------------------------------------

    def _on_odom(self, msg: VehicleOdometry):
        # Quaternion is [w, x, y, z]. Guard against NaN from EKF warmup.
        q = list(msg.q)
        if q and not (math.isnan(q[0]) or math.isnan(q[1])
                      or math.isnan(q[2]) or math.isnan(q[3])):
            self._yaw = yaw_from_quaternion(q)
        if not math.isnan(msg.position[2]):
            self._altitude_ned = float(msg.position[2])

    def _on_status(self, msg: VehicleStatus):
        self._arming_state = msg.arming_state
        self._nav_state = msg.nav_state

    def _on_cmd_ack(self, msg: VehicleCommandAck):
        result_names = {
            0: 'ACCEPTED',
            1: 'TEMPORARILY_REJECTED',
            2: 'DENIED',
            3: 'UNSUPPORTED',
            4: 'FAILED',
            5: 'IN_PROGRESS',
            6: 'CANCELLED',
        }
        name = result_names.get(msg.result, str(msg.result))
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().info(
                'Command ACK: command=%d result=%s' % (msg.command, name))
        else:
            self.get_logger().warn(
                'Command ACK: command=%d result=%s' % (msg.command, name))

    # --- timer callback ---------------------------------------------------

    def _on_tick(self):
        if not self._streaming:
            return

        # Drain any discrete actions queued by the keyboard reader.
        action = self._keyboard.pop_action()
        while action is not None:
            self._handle_action(action)
            action = self._keyboard.pop_action()

        if self._takeoff_active:
            self._publish_takeoff_setpoint()
        else:
            self._publish_velocity_setpoint()

        self._setpoint_count += 1

    # --- setpoint publishing ----------------------------------------------

    def _publish_velocity_setpoint(self):
        held = self._keyboard.get_held()
        speed = self._linear_speed
        yaw_rate = self._yaw_speed

        if held['hover']:
            vx_ned = vy_ned = vz = yawspeed = 0.0
        else:
            vx_body = held['fwd'] * speed
            vy_body = held['strafe'] * speed
            # NED z: up is negative. 'alt' +1 means up.
            vz = -held['alt'] * speed
            # CCW (Q, +1) → negative yawspeed in NED (z points down).
            yawspeed = -held['yaw'] * yaw_rate
            vx_ned, vy_ned = body_to_ned(vx_body, vy_body, self._yaw)

        offboard = OffboardControlMode()
        offboard.timestamp = now_us()
        offboard.position = False
        offboard.velocity = True
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        offboard.thrust_and_torque = False
        offboard.direct_actuator = False
        self._pub_offboard.publish(offboard)

        traj = nan_filled_trajectory(vx_ned, vy_ned, vz, yawspeed)
        traj.timestamp = now_us()
        self._pub_traj.publish(traj)

    def _publish_takeoff_setpoint(self):
        # Position-control setpoint to climb to takeoff_alt above start.
        offboard = OffboardControlMode()
        offboard.timestamp = now_us()
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        offboard.thrust_and_torque = False
        offboard.direct_actuator = False
        self._pub_offboard.publish(offboard)

        traj = TrajectorySetpoint()
        nan = float('nan')
        # Hold horizontal position (NaN velocity → PX4 keeps current pos);
        # set only z target. x/y position NaN → no horizontal change.
        traj.position = [nan, nan, float(self._takeoff_target_z)]
        traj.velocity = [nan, nan, nan]
        traj.acceleration = [nan, nan, nan]
        traj.jerk = [nan, nan, nan]
        traj.yaw = nan
        traj.yawspeed = 0.0
        traj.timestamp = now_us()
        self._pub_traj.publish(traj)

        # Check completion: within 0.2 m of the target altitude.
        if abs(self._altitude_ned - self._takeoff_target_z) < 0.2:
            self.get_logger().info(
                'Takeoff complete (alt %.2f m). Switching to velocity mode.'
                % -self._altitude_ned)
            self._takeoff_active = False
            self._keyboard.clear_held()

    # --- discrete actions -------------------------------------------------

    def _handle_action(self, action: str):
        if action == 'arm':
            self._publish_command(self.CMD_COMPONENT_ARM_DISARM, p1=1.0)
            self.get_logger().info('Command: ARM')
        elif action == 'disarm':
            self._publish_command(self.CMD_COMPONENT_ARM_DISARM, p1=0.0)
            self.get_logger().info('Command: DISARM')
        elif action == 'takeoff':
            # Target altitude in NED z = -takeoff_alt (up is negative).
            self._takeoff_target_z = -float(self._takeoff_alt)
            self._takeoff_active = True
            self.get_logger().info(
                'Command: TAKEOFF to %.2f m' % self._takeoff_alt)
        elif action == 'land':
            self._publish_command(self.CMD_NAV_LAND)
            self.get_logger().info('Command: LAND (PX4 auto-land takes over)')
            # Stop streaming — PX4 auto-land owns the descent.
            self._streaming = False
        elif action == 'offboard':
            # PX4 requires streaming setpoints before accepting offboard.
            if self._setpoint_count < 10:
                self.get_logger().warn(
                    'Offboard requested but only %d setpoints streamed; '
                    'wait a moment and retry.' % self._setpoint_count)
                return
            self._publish_command(self.CMD_SET_NAV_STATE,
                                  p1=float(self.NAV_OFFBOARD))
            self.get_logger().info('Command: enter OFFBOARD mode')
        elif action == 'hold':
            self._publish_command(self.CMD_SET_NAV_STATE,
                                  p1=float(self.NAV_AUTO_LOITER))
            self.get_logger().info('Command: HOLD (AUTO_LOITER)')
        else:
            self.get_logger().warn('Unknown action: %s' % action)

    def _publish_command(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                         p5=0.0, p6=0.0, p7=0.0):
        cmd = build_vehicle_command(command, p1, p2, p3, p4, p5, p6, p7)
        cmd.timestamp = now_us()
        self._pub_cmd.publish(cmd)

    # --- shutdown ---------------------------------------------------------

    def shutdown(self):
        """Best-effort cleanup: restore terminal and disarm."""
        try:
            self._keyboard.stop()
        except Exception:  # noqa: BLE001
            pass
        if self._arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info('Shutdown: publishing DISARM command')
            self._publish_command(self.CMD_COMPONENT_ARM_DISARM, p1=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
