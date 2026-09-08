"""Pure helpers for PX4 offboard control.

Everything in this module is free of ROS node state: no publishers, no
subscriptions, no clock. That is deliberate — it lets the tricky parts
(frame conversion, message field discipline, arrival hysteresis) be unit
tested without a running ROS graph or a live autopilot.

Shared by `offboard_mission.waypoint_node` and
`keyboard_offboard_control.keyboard_offboard_node`.
"""
import math
import time

from px4_msgs.msg import TrajectorySetpoint, VehicleCommand


def now_us():
    """Current wall-clock time in microseconds (for PX4 message timestamps).

    PX4 uses time-since-system-start in microseconds, but with
    UXRCE_DDS_SYNCT=1 the MicroXRCE-DDS bridge translates companion
    wall-clock timestamps into PX4's internal clock. Sending 0 makes PX4
    treat the setpoint as stale (56 years old) and reject it. We send the
    real wall-clock microseconds instead.
    """
    return int(time.time() * 1_000_000)


# ---------------------------------------------------------------------------
# Orientation
# ---------------------------------------------------------------------------

def yaw_from_quaternion(q):
    """Extract yaw (rotation about NED z, in radians, wrapped to (-pi, pi])
    from a [w, x, y, z] quaternion.

    Uses the standard formula for the third Euler angle (ZYX convention).
    """
    w, x, y, z = q
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def body_to_ned(vx_body, vy_body, yaw):
    """Rotate a body-frame 2D velocity (vx forward, vy right) into the NED
    frame using the vehicle's current yaw about NED-z.

    NED: x = north, y = east.  Body: x = forward, y = right.
        vx_ned =  vx_body * cos(yaw) - vy_body * sin(yaw)
        vy_ned =  vx_body * sin(yaw) + vy_body * cos(yaw)
    """
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return (vx_body * cos_y - vy_body * sin_y,
            vx_body * sin_y + vy_body * cos_y)


# ---------------------------------------------------------------------------
# Frame conversion  ENU (ROS / `map`) <-> NED (PX4)
# ---------------------------------------------------------------------------
#
# ROS `map` is ENU:  x = east,  y = north, z = UP
# PX4 local is NED:  x = north, y = east,  z = DOWN
#
# So the two horizontal axes swap and the vertical axis flips sign. The pair
# below are exact inverses of each other, and `ned_to_enu` matches
# `ned_to_enu_pos()` in src/gz_bridge_ros2/src/drone_kinematic.cpp — the node
# that publishes the odom -> base_link TF. If these ever disagree, TF and the
# setpoints would silently point at different places.

def enu_to_ned(x_east, y_north, z_up):
    """Convert an ENU position (ROS `map` frame) to PX4 local NED.

    Returns (north, east, down). Note that an altitude ABOVE the origin comes
    back as a NEGATIVE `down` value.
    """
    return (y_north, x_east, -z_up)


def ned_to_enu(north, east, down):
    """Convert a PX4 local NED position to ENU (ROS `map` frame).

    Returns (x_east, y_north, z_up). Exact inverse of `enu_to_ned`.
    """
    return (east, north, -down)


def horizontal_distance(a, b):
    """Distance between two (x, y, z) points, ignoring z.

    Where a vehicle came down is a horizontal question. Comparing a resting
    place against a chosen landing point in full 3D would report a miss that
    is not one: the chosen point is the surface centroid, and the vehicle
    stands on its legs above it.
    """
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


# ---------------------------------------------------------------------------
# PX4 message builders
# ---------------------------------------------------------------------------

def build_vehicle_command(command, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                          p5=0.0, p6=0.0, p7=0.0):
    """Build a VehicleCommand with the standard envelope used for internal
    commands issued by a companion computer.

    from_external=False because we reach the autopilot over the internal
    MicroXRCE-DDS bridge; target_system=1 / target_component=1 is the
    autopilot itself.
    """
    cmd = VehicleCommand()
    cmd.command = command
    cmd.param1 = float(p1)
    cmd.param2 = float(p2)
    cmd.param3 = float(p3)
    cmd.param4 = float(p4)
    cmd.param5 = float(p5)
    cmd.param6 = float(p6)
    cmd.param7 = float(p7)
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.source_system = 255
    cmd.source_component = 0
    cmd.from_external = False
    cmd.confirmation = 0
    return cmd


def nan_filled_trajectory(vx, vy, vz, yawspeed):
    """Build a TrajectorySetpoint that controls only velocity + yawspeed.

    position / acceleration / jerk / yaw are NaN so PX4 ignores them and
    regulates the velocity setpoint alone.
    """
    msg = TrajectorySetpoint()
    nan = float('nan')
    msg.position = [nan, nan, nan]
    msg.velocity = [float(vx), float(vy), float(vz)]
    msg.acceleration = [nan, nan, nan]
    msg.jerk = [nan, nan, nan]
    msg.yaw = nan
    msg.yawspeed = float(yawspeed)
    return msg


def position_trajectory(north, east, down, yaw=None):
    """Build a TrajectorySetpoint that controls only position (NED metres).

    The mirror of `nan_filled_trajectory` for position control: velocity,
    acceleration and jerk are NaN so PX4 plans its own approach to the point.

    `yaw` is left NaN unless a finite value is given, which tells PX4 to keep
    whatever heading it currently holds. Passing NaN explicitly behaves the
    same as omitting it.
    """
    msg = TrajectorySetpoint()
    nan = float('nan')
    msg.position = [float(north), float(east), float(down)]
    msg.velocity = [nan, nan, nan]
    msg.acceleration = [nan, nan, nan]
    msg.jerk = [nan, nan, nan]
    msg.yaw = float(yaw) if (yaw is not None and not math.isnan(float(yaw))) else nan
    msg.yawspeed = 0.0
    return msg


# ---------------------------------------------------------------------------
# Arrival detection
# ---------------------------------------------------------------------------

class ArrivalDetector:
    """Report arrival only after the vehicle has DWELLED inside a tolerance.

    A bare distance test fires the instant the vehicle crosses the target,
    which for a drone still carrying horizontal speed means "arrived" is
    announced mid-fly-through and the mission advances while the vehicle sails
    past. Requiring the distance to stay within tolerance for `settle_s`
    continuous seconds removes that failure mode.

    Leaving the tolerance RESTARTS the dwell timer from zero rather than
    pausing it — a vehicle that keeps oscillating in and out of the radius has
    not settled, and accumulating its brief visits would claim otherwise.
    """

    def __init__(self, tol_m, settle_s):
        self._tol_m = float(tol_m)
        self._settle_s = float(settle_s)
        self._dwell_s = 0.0

    def update(self, distance_m, dt_s):
        """Feed one sample. Returns True once dwell >= settle_s."""
        if distance_m <= self._tol_m:
            self._dwell_s += max(0.0, float(dt_s))
        else:
            self._dwell_s = 0.0
        return bool(self._dwell_s >= self._settle_s)

    def reset(self):
        """Forget any accumulated dwell, e.g. when switching to a new target."""
        self._dwell_s = 0.0

    @property
    def dwell_s(self):
        return self._dwell_s
