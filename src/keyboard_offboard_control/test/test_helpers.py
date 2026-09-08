"""Unit tests for pure helpers in keyboard_offboard_node.

These tests intentionally avoid spinning up ROS. They import only the
pure functions and the px4_msgs message classes. The message classes
themselves are generated ROS messages, so this test file is meant to be
run after `colcon build` has produced the px4_msgs Python module, OR
inside a sourced ROS workspace.
"""
import math

import pytest


def _import_helpers():
    """Import the helpers lazily so the test file can still be collected
    even if px4_msgs is not yet on the Python path."""
    from keyboard_offboard_control.keyboard_offboard_node import (
        body_to_ned,
        build_vehicle_command,
        nan_filled_trajectory,
        yaw_from_quaternion,
    )
    return body_to_ned, build_vehicle_command, nan_filled_trajectory, yaw_from_quaternion


def _import_msgs():
    from px4_msgs.msg import TrajectorySetpoint, VehicleCommand
    return TrajectorySetpoint, VehicleCommand


# ---------------------------------------------------------------------------
# yaw_from_quaternion
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("q, expected_yaw", [
    ([1.0, 0.0, 0.0, 0.0], 0.0),                       # identity
    ([math.cos(0.25 * math.pi), 0.0, 0.0, math.sin(0.25 * math.pi)], 0.5 * math.pi),  # +90 deg about z
    ([math.cos(0.5 * math.pi), 0.0, 0.0, math.sin(0.5 * math.pi)], math.pi),          # +180 deg
    ([math.cos(-0.25 * math.pi), 0.0, 0.0, math.sin(-0.25 * math.pi)], -0.5 * math.pi),  # -90 deg
])
def test_yaw_from_quaternion_returns_rotation_about_z(q, expected_yaw):
    _, _, _, yaw_from_quaternion = _import_helpers()
    assert yaw_from_quaternion(q) == pytest.approx(expected_yaw, abs=1e-6)


def test_yaw_from_quaternion_wraps_to_principal_range():
    """A quaternion representing +270 deg about z must come back as -90 deg,
    i.e. the function should wrap into (-pi, pi]."""
    _, _, _, yaw_from_quaternion = _import_helpers()
    # [cos(0.75pi), 0, 0, sin(0.75pi)] encodes a 2*0.75pi = 270 deg rotation about z.
    q = [math.cos(0.75 * math.pi), 0.0, 0.0, math.sin(0.75 * math.pi)]  # +270 deg
    assert yaw_from_quaternion(q) == pytest.approx(-0.5 * math.pi, abs=1e-6)


# ---------------------------------------------------------------------------
# body_to_ned
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("vx_body, vy_body, yaw, expected", [
    (1.0, 0.0, 0.0, (1.0, 0.0)),                       # facing north: fwd -> north
    (1.0, 0.0, 0.5 * math.pi, (0.0, 1.0)),             # facing east:  fwd -> east
    (0.0, 1.0, 0.0, (0.0, 1.0)),                       # facing north: right -> east
    (1.0, 1.0, math.pi, (-1.0, -1.0)),                 # facing south: fwd+right -> south+west
    (1.0, 0.0, -0.5 * math.pi, (0.0, -1.0)),           # facing west:  fwd -> west
])
def test_body_to_ned_rotates_body_velocity_into_ned(vx_body, vy_body, yaw, expected):
    body_to_ned, _, _, _ = _import_helpers()
    assert body_to_ned(vx_body, vy_body, yaw) == pytest.approx(expected, abs=1e-6)


# ---------------------------------------------------------------------------
# build_vehicle_command
# ---------------------------------------------------------------------------

def test_build_vehicle_command_sets_envelope_and_forwards_params():
    _, build_vehicle_command, _, _ = _import_helpers()
    _, VehicleCommand = _import_msgs()

    cmd = build_vehicle_command(
        command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        p1=1.0,
        p7=2.5,
    )

    assert isinstance(cmd, VehicleCommand)
    assert cmd.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    assert cmd.target_system == 1
    assert cmd.target_component == 1
    assert cmd.source_system == 255
    assert cmd.source_component == 0
    assert cmd.from_external is False
    assert cmd.confirmation == 0
    assert cmd.param1 == 1.0
    assert cmd.param2 == 0.0
    assert cmd.param3 == 0.0
    assert cmd.param4 == 0.0
    assert cmd.param5 == 0.0
    assert cmd.param6 == 0.0
    assert cmd.param7 == 2.5


def test_build_vehicle_command_defaults_all_params_to_zero():
    _, build_vehicle_command, _, _ = _import_helpers()
    _, VehicleCommand = _import_msgs()

    cmd = build_vehicle_command(command=VehicleCommand.VEHICLE_CMD_NAV_LAND)

    assert cmd.param1 == 0.0
    assert cmd.param2 == 0.0
    assert cmd.param3 == 0.0
    assert cmd.param4 == 0.0
    assert cmd.param5 == 0.0
    assert cmd.param6 == 0.0
    assert cmd.param7 == 0.0


# ---------------------------------------------------------------------------
# nan_filled_trajectory
# ---------------------------------------------------------------------------

def test_nan_filled_trajectory_populates_only_velocity_and_yawspeed():
    _, _, nan_filled_trajectory, _ = _import_helpers()
    TrajectorySetpoint, _ = _import_msgs()

    msg = nan_filled_trajectory(vx=0.5, vy=-0.3, vz=0.1, yawspeed=0.2)

    assert isinstance(msg, TrajectorySetpoint)
    # Velocity + yawspeed are the controlled fields
    assert list(msg.velocity) == [0.5, -0.3, 0.1]
    assert msg.yawspeed == 0.2
    # Everything else must be NaN so PX4 ignores it
    for v in msg.position:
        assert math.isnan(v)
    for v in msg.acceleration:
        assert math.isnan(v)
    for v in msg.jerk:
        assert math.isnan(v)
    assert math.isnan(msg.yaw)


def test_nan_filled_trajectory_does_not_mutate_shared_template():
    """Sanity check: building two messages must not bleed state."""
    _, _, nan_filled_trajectory, _ = _import_helpers()
    TrajectorySetpoint, _ = _import_msgs()

    a = nan_filled_trajectory(vx=1.0, vy=0.0, vz=0.0, yawspeed=0.0)
    b = nan_filled_trajectory(vx=0.0, vy=0.0, vz=0.0, yawspeed=0.0)

    assert list(a.velocity) == [1.0, 0.0, 0.0]
    assert list(b.velocity) == [0.0, 0.0, 0.0]
