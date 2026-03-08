#!/usr/bin/env python3
"""
keyboard_control.py
====================
Kontrol drone PX4 menggunakan keyboard via ROS2 offboard mode.

Struktur mengikuti px4-offboard (Jaeyoung-Lim) dengan tambahan
input keyboard menggunakan pynput.

Keybind:
  TAKEOFF / LAND
    T          : Arm + Takeoff ke ketinggian default
    L          : Land (turun perlahan ke tanah)
    SPACE      : Disarm darurat (kill motors)

  GERAK (dalam frame NED / body-relative)
    W / S      : Maju / Mundur   (+X / -X)
    A / D      : Kiri / Kanan    (-Y / +Y)
    Q / E      : Naik / Turun    (-Z / +Z)  [NED: Z negatif = naik]
    ← / →      : Yaw kiri / kanan

  KECEPATAN
    + / =      : Tambah kecepatan translasi
    - / _      : Kurangi kecepatan translasi

  LAIN
    R          : Reset posisi ke home
    ESC        : Quit node

Requirement:
    pip install pynput
    ros2 pkg: px4_msgs, rclpy
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

try:
    from pynput import keyboard as kb
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False
    print("[WARN] pynput tidak tersedia. Install dengan: pip install pynput")


# ======================================================================= #
#  Konstanta
# ======================================================================= #
DEFAULT_TAKEOFF_HEIGHT = -2.0   # meter (NED: negatif = naik)
DEFAULT_SPEED          = 0.5    # m/s translasi
DEFAULT_YAW_SPEED      = 0.5    # rad/s
SPEED_STEP             = 0.1    # increment/decrement kecepatan
MAX_SPEED              = 3.0
MIN_SPEED              = 0.1


class KeyboardDroneController(Node):
    def __init__(self):
        super().__init__("keyboard_drone_controller")

        # ── QoS untuk PX4 ────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_pos_callback,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._vehicle_status_callback,
            qos,
        )

        # ── State ────────────────────────────────────────────────────────
        self.local_pos        = VehicleLocalPosition()
        self.vehicle_status   = VehicleStatus()
        self.is_armed         = False
        self.offboard_active  = False
        self.flight_phase     = "grounded"   # grounded | taking_off | flying | landing

        # Posisi setpoint saat ini (NED frame)
        self.sp_x   = 0.0
        self.sp_y   = 0.0
        self.sp_z   = 0.0
        self.sp_yaw = 0.0    # radian

        self.speed     = DEFAULT_SPEED
        self.yaw_speed = DEFAULT_YAW_SPEED

        # Keys yang sedang ditekan
        self._pressed: set = set()
        self._lock = threading.Lock()

        self.offboard_counter = 0   # harus kirim >10 frame sebelum switch ke offboard

        # ── Timer utama 50 Hz ────────────────────────────────────────────
        self.create_timer(0.02, self._control_loop)

        # ── Keyboard listener ────────────────────────────────────────────
        if PYNPUT_AVAILABLE:
            self._start_keyboard_listener()
        else:
            self.get_logger().error("pynput tidak tersedia! Install: pip install pynput")

        self._print_help()

    # ================================================================== #
    #  Keyboard
    # ================================================================== #
    def _start_keyboard_listener(self):
        def on_press(key):
            with self._lock:
                try:
                    self._pressed.add(key.char.lower())
                except AttributeError:
                    self._pressed.add(key)
            self._handle_one_shot(key)

        def on_release(key):
            with self._lock:
                try:
                    self._pressed.discard(key.char.lower())
                except AttributeError:
                    self._pressed.discard(key)
            if key == kb.Key.esc:
                self.get_logger().info("ESC ditekan — menutup node.")
                rclpy.shutdown()
                return False

        listener = kb.Listener(on_press=on_press, on_release=on_release)
        listener.daemon = True
        listener.start()
        self.get_logger().info("Keyboard listener aktif.")

    def _handle_one_shot(self, key):
        """Aksi yang dipicu sekali saat tombol ditekan (bukan hold)."""
        try:
            ch = key.char.lower()
        except AttributeError:
            ch = None

        if ch == "t":
            self._cmd_takeoff()
        elif ch == "l":
            self._cmd_land()
        elif ch == " ":
            self._cmd_disarm()
        elif ch == "r":
            self._cmd_reset_home()
        elif ch in ("+", "="):
            self.speed = min(self.speed + SPEED_STEP, MAX_SPEED)
            self.get_logger().info(f"Kecepatan: {self.speed:.1f} m/s")
        elif ch in ("-", "_"):
            self.speed = max(self.speed - SPEED_STEP, MIN_SPEED)
            self.get_logger().info(f"Kecepatan: {self.speed:.1f} m/s")

    def _is_pressed(self, *chars) -> bool:
        with self._lock:
            return any(c in self._pressed for c in chars)

    # ================================================================== #
    #  Callbacks
    # ================================================================== #
    def _local_pos_callback(self, msg: VehicleLocalPosition):
        self.local_pos = msg

    def _vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg
        self.is_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    # ================================================================== #
    #  Kontrol loop utama (50 Hz)
    # ================================================================== #
    def _control_loop(self):
        # Selalu publish offboard control mode agar PX4 tidak timeout
        self._publish_offboard_mode()

        if self.flight_phase == "grounded":
            # Sebelum takeoff: kirim setpoint posisi saat ini agar
            # offboard counter bertambah
            self._publish_setpoint(
                self.local_pos.x,
                self.local_pos.y,
                self.local_pos.z,
                self.sp_yaw,
            )
            self.offboard_counter += 1
            return

        # ── Update setpoint berdasarkan keyboard ─────────────────────────
        self._apply_keyboard_input()

        # ── Publish setpoint ─────────────────────────────────────────────
        self._publish_setpoint(self.sp_x, self.sp_y, self.sp_z, self.sp_yaw)

    def _apply_keyboard_input(self):
        """Update sp_x, sp_y, sp_z, sp_yaw berdasarkan key yang ditekan."""
        if self.flight_phase not in ("flying", "taking_off"):
            return

        dt     = 0.02   # 50 Hz
        spd    = self.speed
        yaw_s  = self.yaw_speed
        yaw    = self.sp_yaw

        # Translasi dalam frame body → konversi ke NED
        # Maju (+X body) = cos(yaw)*dx - sin(yaw)*dy
        dx_body = 0.0
        dy_body = 0.0
        dz      = 0.0
        dyaw    = 0.0

        if self._is_pressed("w"):
            dx_body += spd * dt
        if self._is_pressed("s"):
            dx_body -= spd * dt
        if self._is_pressed("a"):
            dy_body -= spd * dt
        if self._is_pressed("d"):
            dy_body += spd * dt
        if self._is_pressed("q"):
            dz -= spd * dt          # NED: Z negatif = naik
        if self._is_pressed("e"):
            dz += spd * dt          # NED: Z positif = turun

        # Arrow keys untuk yaw
        if self._is_pressed(kb.Key.left):
            dyaw -= yaw_s * dt
        if self._is_pressed(kb.Key.right):
            dyaw += yaw_s * dt

        # Rotasi body → NED
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        self.sp_x   += cos_y * dx_body - sin_y * dy_body
        self.sp_y   += sin_y * dx_body + cos_y * dy_body
        self.sp_z   += dz
        self.sp_yaw += dyaw

        # Clamp yaw ke [-π, π]
        self.sp_yaw = math.atan2(math.sin(self.sp_yaw), math.cos(self.sp_yaw))

    # ================================================================== #
    #  Perintah satu kali (takeoff, land, arm, dsb)
    # ================================================================== #
    def _cmd_takeoff(self):
        if self.flight_phase != "grounded":
            self.get_logger().warn("Sudah terbang / dalam proses.")
            return

        if self.offboard_counter < 10:
            self.get_logger().warn(
                f"Tunggu offboard counter ({self.offboard_counter}/10)..."
            )
            return

        self.get_logger().info("Takeoff!")

        # Set setpoint ke atas
        self.sp_x   = self.local_pos.x
        self.sp_y   = self.local_pos.y
        self.sp_z   = DEFAULT_TAKEOFF_HEIGHT
        self.sp_yaw = self.local_pos.heading if hasattr(self.local_pos, "heading") else 0.0

        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self._arm()
        self.flight_phase = "taking_off"

        # Setelah 3 detik anggap sudah terbang
        threading.Timer(3.0, self._set_flying).start()

    def _set_flying(self):
        if self.flight_phase == "taking_off":
            self.flight_phase = "flying"
            self.get_logger().info("Status: FLYING — gunakan WASD/QE/Arrow untuk bergerak.")

    def _cmd_land(self):
        if self.flight_phase == "grounded":
            self.get_logger().warn("Drone sudah di tanah.")
            return
        self.get_logger().info("Landing...")
        self.flight_phase = "landing"
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        threading.Timer(5.0, self._set_grounded).start()

    def _set_grounded(self):
        self.flight_phase = "grounded"
        self.offboard_counter = 0
        self.get_logger().info("Status: GROUNDED.")

    def _cmd_disarm(self):
        self.get_logger().warn("DISARM (kill motors)!")
        self._disarm()
        self.flight_phase = "grounded"
        self.offboard_counter = 0

    def _cmd_reset_home(self):
        self.get_logger().info("Reset ke posisi home.")
        self.sp_x   = 0.0
        self.sp_y   = 0.0
        self.sp_z   = DEFAULT_TAKEOFF_HEIGHT
        self.sp_yaw = 0.0

    # ================================================================== #
    #  Publish helpers
    # ================================================================== #
    def _publish_offboard_mode(self):
        msg                  = OffboardControlMode()
        msg.timestamp        = self._timestamp()
        msg.position         = True
        msg.velocity         = False
        msg.acceleration     = False
        msg.attitude         = False
        msg.body_rate        = False
        self.offboard_mode_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: float):
        msg           = TrajectorySetpoint()
        msg.timestamp = self._timestamp()
        msg.position  = [float(x), float(y), float(z)]
        msg.yaw       = float(yaw)
        self.trajectory_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, param1=0.0, param2=0.0):
        msg                     = VehicleCommand()
        msg.timestamp           = self._timestamp()
        msg.param1              = float(param1)
        msg.param2              = float(param2)
        msg.command             = command
        msg.target_system       = 1
        msg.target_component    = 1
        msg.source_system       = 1
        msg.source_component    = 1
        msg.from_external       = True
        self.vehicle_command_pub.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0
        )
        self.get_logger().info("ARM command dikirim.")

    def _disarm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0
        )
        self.get_logger().info("DISARM command dikirim.")

    def _timestamp(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    # ================================================================== #
    #  Help
    # ================================================================== #
    def _print_help(self):
        self.get_logger().info("""
╔══════════════════════════════════════╗
║     PX4 Keyboard Drone Control       ║
╠══════════════════════════════════════╣
║  T          : Arm + Takeoff           ║
║  L          : Land                    ║
║  SPACE      : Disarm (emergency)      ║
║  R          : Reset ke home           ║
╠══════════════════════════════════════╣
║  W / S      : Maju / Mundur           ║
║  A / D      : Kiri / Kanan            ║
║  Q / E      : Naik / Turun            ║
║  ← / →      : Yaw kiri / kanan        ║
╠══════════════════════════════════════╣
║  + / -      : Tambah/kurangi speed    ║
║  ESC        : Quit                    ║
╚══════════════════════════════════════╝
""")


# ======================================================================= #
#  Main
# ======================================================================= #
def main():
    rclpy.init()
    node = KeyboardDroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()