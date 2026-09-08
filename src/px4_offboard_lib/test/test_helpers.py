"""Unit tests for the pure PX4 offboard helpers.

These tests intentionally avoid spinning up ROS. They import only the pure
functions and the px4_msgs message classes. The message classes themselves
are generated ROS messages, so this file is meant to be run after
`colcon build` has produced the px4_msgs Python module, OR inside a sourced
ROS workspace.
"""
import math

import pytest


def _helpers():
    """Import lazily so the file can still be collected when px4_msgs is not
    yet on the Python path."""
    from px4_offboard_lib import px4_helpers
    return px4_helpers


def _import_msgs():
    from px4_msgs.msg import TrajectorySetpoint, VehicleCommand
    return TrajectorySetpoint, VehicleCommand


# ---------------------------------------------------------------------------
# yaw_from_quaternion
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("q, expected_yaw", [
    ([1.0, 0.0, 0.0, 0.0], 0.0),                       # identity
    ([math.cos(0.25 * math.pi), 0.0, 0.0, math.sin(0.25 * math.pi)], 0.5 * math.pi),
    ([math.cos(0.5 * math.pi), 0.0, 0.0, math.sin(0.5 * math.pi)], math.pi),
    ([math.cos(-0.25 * math.pi), 0.0, 0.0, math.sin(-0.25 * math.pi)], -0.5 * math.pi),
])
def test_yaw_from_quaternion_returns_rotation_about_z(q, expected_yaw):
    assert _helpers().yaw_from_quaternion(q) == pytest.approx(expected_yaw, abs=1e-6)


def test_yaw_from_quaternion_wraps_to_principal_range():
    """A quaternion representing +270 deg about z must come back as -90 deg."""
    q = [math.cos(0.75 * math.pi), 0.0, 0.0, math.sin(0.75 * math.pi)]
    assert _helpers().yaw_from_quaternion(q) == pytest.approx(-0.5 * math.pi, abs=1e-6)


# ---------------------------------------------------------------------------
# body_to_ned
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("vx_body, vy_body, yaw, expected", [
    (1.0, 0.0, 0.0, (1.0, 0.0)),                       # facing north: fwd -> north
    (1.0, 0.0, 0.5 * math.pi, (0.0, 1.0)),             # facing east:  fwd -> east
    (0.0, 1.0, 0.0, (0.0, 1.0)),                       # facing north: right -> east
    (1.0, 1.0, math.pi, (-1.0, -1.0)),                 # facing south
    (1.0, 0.0, -0.5 * math.pi, (0.0, -1.0)),           # facing west
])
def test_body_to_ned_rotates_body_velocity_into_ned(vx_body, vy_body, yaw, expected):
    assert _helpers().body_to_ned(vx_body, vy_body, yaw) == pytest.approx(expected, abs=1e-6)


# ---------------------------------------------------------------------------
# enu_to_ned / ned_to_enu
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("enu, expected_ned", [
    ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),    # 1 m east  -> NED east  = 1
    ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),    # 1 m north -> NED north = 1
    ((0.0, 0.0, 1.0), (0.0, 0.0, -1.0)),   # 1 m up    -> NED down  = -1
    ((3.0, -2.0, 5.0), (-2.0, 3.0, -5.0)),
])
def test_enu_to_ned_maps_axes(enu, expected_ned):
    assert _helpers().enu_to_ned(*enu) == pytest.approx(expected_ned, abs=1e-9)


def test_enu_to_ned_altitude_up_becomes_negative_down():
    """The sign flip on the vertical axis is the mistake that silently flies a
    drone into the ground, so it gets its own test."""
    _, _, down = _helpers().enu_to_ned(0.0, 0.0, 3.0)   # 3 m ABOVE origin
    assert down == pytest.approx(-3.0)
    assert down < 0.0


@pytest.mark.parametrize("enu", [
    (0.0, 0.0, 0.0),
    (1.0, 2.0, 3.0),
    (-4.5, 6.25, -0.5),
])
def test_enu_to_ned_and_back_is_identity(enu):
    h = _helpers()
    assert h.ned_to_enu(*h.enu_to_ned(*enu)) == pytest.approx(enu, abs=1e-9)


def test_ned_to_enu_matches_drone_kinematic_convention():
    """drone_kinematic.cpp does: xe = ye, yn = xn, zu = -zd.
    Our Python inverse must agree, or TF and setpoints disagree about north."""
    north, east, down = 7.0, -3.0, -12.0
    x_east, y_north, z_up = _helpers().ned_to_enu(north, east, down)
    assert (x_east, y_north, z_up) == pytest.approx((east, north, -down))


# ---------------------------------------------------------------------------
# build_vehicle_command
# ---------------------------------------------------------------------------

def test_build_vehicle_command_sets_envelope_and_forwards_params():
    _, VehicleCommand = _import_msgs()
    cmd = _helpers().build_vehicle_command(
        command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0, p7=2.5)

    assert isinstance(cmd, VehicleCommand)
    assert cmd.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    assert cmd.target_system == 1
    assert cmd.target_component == 1
    assert cmd.source_system == 255
    assert cmd.source_component == 0
    assert cmd.from_external is False
    assert cmd.confirmation == 0
    assert cmd.param1 == 1.0
    assert cmd.param7 == 2.5
    for p in (cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6):
        assert p == 0.0


def test_build_vehicle_command_defaults_all_params_to_zero():
    _, VehicleCommand = _import_msgs()
    cmd = _helpers().build_vehicle_command(command=VehicleCommand.VEHICLE_CMD_NAV_LAND)
    for p in (cmd.param1, cmd.param2, cmd.param3,
              cmd.param4, cmd.param5, cmd.param6, cmd.param7):
        assert p == 0.0


# ---------------------------------------------------------------------------
# nan_filled_trajectory
# ---------------------------------------------------------------------------

def test_nan_filled_trajectory_populates_only_velocity_and_yawspeed():
    TrajectorySetpoint, _ = _import_msgs()
    msg = _helpers().nan_filled_trajectory(vx=0.5, vy=-0.3, vz=0.1, yawspeed=0.2)

    assert isinstance(msg, TrajectorySetpoint)
    assert list(msg.velocity) == [0.5, -0.3, 0.1]
    assert msg.yawspeed == 0.2
    for field in (msg.position, msg.acceleration, msg.jerk):
        for v in field:
            assert math.isnan(v)
    assert math.isnan(msg.yaw)


def test_nan_filled_trajectory_does_not_mutate_shared_template():
    a = _helpers().nan_filled_trajectory(vx=1.0, vy=0.0, vz=0.0, yawspeed=0.0)
    b = _helpers().nan_filled_trajectory(vx=0.0, vy=0.0, vz=0.0, yawspeed=0.0)
    assert list(a.velocity) == [1.0, 0.0, 0.0]
    assert list(b.velocity) == [0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# position_trajectory
# ---------------------------------------------------------------------------

def test_position_trajectory_populates_only_position():
    TrajectorySetpoint, _ = _import_msgs()
    msg = _helpers().position_trajectory(north=1.0, east=2.0, down=-3.0)

    assert isinstance(msg, TrajectorySetpoint)
    assert list(msg.position) == [1.0, 2.0, -3.0]
    for field in (msg.velocity, msg.acceleration, msg.jerk):
        for v in field:
            assert math.isnan(v)
    assert math.isnan(msg.yaw), "yaw must stay NaN so PX4 keeps the current heading"


def test_position_trajectory_sets_yaw_when_given():
    msg = _helpers().position_trajectory(north=0.0, east=0.0, down=-2.0, yaw=1.25)
    assert msg.yaw == pytest.approx(1.25)


def test_position_trajectory_ignores_nan_yaw():
    """Passing NaN explicitly must behave exactly like passing nothing."""
    msg = _helpers().position_trajectory(0.0, 0.0, -2.0, yaw=float('nan'))
    assert math.isnan(msg.yaw)


# ---------------------------------------------------------------------------
# ArrivalDetector
# ---------------------------------------------------------------------------

def test_arrival_detector_requires_settle_time():
    det = _helpers().ArrivalDetector(tol_m=0.5, settle_s=1.0)
    assert det.update(0.3, 0.25) is False
    assert det.update(0.3, 0.25) is False
    assert det.update(0.3, 0.25) is False
    assert det.update(0.3, 0.25) is True, "should arrive once settle_s has elapsed"


def test_arrival_detector_stays_false_while_outside_tolerance():
    det = _helpers().ArrivalDetector(tol_m=0.5, settle_s=1.0)
    for _ in range(20):
        assert det.update(2.0, 0.25) is False


def test_arrival_detector_restarts_count_after_leaving_tolerance():
    """This is what stops a fast fly-through from being reported as arrival:
    the dwell timer must restart from zero, not merely pause."""
    det = _helpers().ArrivalDetector(tol_m=0.5, settle_s=1.0)
    det.update(0.3, 0.25)
    det.update(0.3, 0.25)          # 0.5 s accumulated
    assert det.update(0.9, 0.25) is False   # left tolerance -> reset

    assert det.update(0.3, 0.25) is False   # 0.25 s
    assert det.update(0.3, 0.25) is False   # 0.50 s
    assert det.update(0.3, 0.25) is False   # 0.75 s
    assert det.update(0.3, 0.25) is True    # 1.00 s


def test_arrival_detector_reset_clears_progress():
    det = _helpers().ArrivalDetector(tol_m=0.5, settle_s=1.0)
    det.update(0.1, 0.9)
    det.reset()
    assert det.update(0.1, 0.5) is False


def test_arrival_detector_zero_settle_arrives_immediately():
    det = _helpers().ArrivalDetector(tol_m=0.5, settle_s=0.0)
    assert det.update(0.4, 0.05) is True


def test_arrival_detector_boundary_distance_counts_as_inside():
    det = _helpers().ArrivalDetector(tol_m=0.5, settle_s=0.0)
    assert det.update(0.5, 0.1) is True


# ---------------------------------------------------------------------------
# now_us
# ---------------------------------------------------------------------------

def test_now_us_is_nonzero():
    """A zero timestamp is exactly what makes PX4 discard a setpoint as
    decades stale, so this is the property that actually matters."""
    assert _helpers().now_us() > 0


def test_now_us_returns_integer_microseconds():
    import time
    value = _helpers().now_us()
    assert isinstance(value, int)
    # Within a couple of seconds of wall clock, expressed in microseconds.
    # A seconds-based or nanosecond-based return would miss by 10^6 either way.
    assert value == pytest.approx(time.time() * 1_000_000, abs=2_000_000)


def test_now_us_is_monotonic_enough_to_order_setpoints():
    h = _helpers()
    first = h.now_us()
    second = h.now_us()
    assert second >= first


# ---------------------------------------------------------------------------
# horizontal_distance
# ---------------------------------------------------------------------------
#
# Where a vehicle touched down is a horizontal question. The mission compares
# its resting place against the landing point it chose, and those two are at
# different altitudes by construction: the chosen point is the surface
# centroid, the vehicle sits on its legs above it. Including z would report a
# miss that is not one.


def test_horizontal_distance_ignores_altitude():
    h = _helpers()
    assert h.horizontal_distance((0.0, 0.0, 0.0), (3.0, 4.0, 99.0)) == pytest.approx(5.0)


def test_horizontal_distance_is_zero_for_the_same_column():
    h = _helpers()
    assert h.horizontal_distance((2.0, -7.0, 1.0), (2.0, -7.0, -30.0)) == pytest.approx(0.0)


def test_horizontal_distance_is_symmetric():
    h = _helpers()
    a, b = (1.0, 2.0, 3.0), (-4.0, 6.5, 0.0)
    assert h.horizontal_distance(a, b) == pytest.approx(h.horizontal_distance(b, a))


def test_horizontal_distance_handles_negative_coordinates():
    h = _helpers()
    assert h.horizontal_distance((-4.79, 16.21, -2.37),
                                 (5.11, 9.36, 0.41)) == pytest.approx(12.04, abs=0.01)
