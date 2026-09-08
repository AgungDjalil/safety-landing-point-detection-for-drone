"""Import-contract test for offboard_mission.

The behaviour of the pure helpers is tested where they live, in
src/px4_offboard_lib/test/test_helpers.py. What this file guards is narrower
and specific to this package: that `waypoint_node` can actually resolve those
helpers through px4_offboard_lib. A broken or renamed dependency would
otherwise only surface at run time, on an armed vehicle.
"""


def test_waypoint_node_resolves_shared_helpers():
    from offboard_mission import waypoint_node

    for name in ('ArrivalDetector', 'build_vehicle_command', 'enu_to_ned',
                 'nan_filled_trajectory', 'now_us', 'position_trajectory'):
        assert hasattr(waypoint_node, name), (
            '%s did not resolve through px4_offboard_lib' % name)


def test_waypoint_node_timestamps_use_wall_clock_helper():
    """The node must take PX4 message timestamps from the shared wall-clock
    helper, not from its ROS clock: under use_sim_time the ROS clock returns
    simulation time starting near zero, which PX4 discards as stale."""
    import inspect

    from offboard_mission import waypoint_node

    source = inspect.getsource(waypoint_node)
    assert 'timestamp = now_us()' in source
    assert 'timestamp = self.get_clock()' not in source


def test_topic_variants_adds_versioned_suffix():
    """PX4 >= 1.16 publishes /fmu/out/..._v1. Subscribing to both variants is
    what stops a silent 'no position ever arrives' failure."""
    from offboard_mission.waypoint_node import WaypointMission

    assert WaypointMission._topic_variants('/fmu/out/vehicle_local_position') == [
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_local_position_v1',
    ]


def test_topic_variants_does_not_double_suffix():
    from offboard_mission.waypoint_node import WaypointMission

    assert WaypointMission._topic_variants('/fmu/out/vehicle_status_v1') == [
        '/fmu/out/vehicle_status_v1',
    ]


def test_start_time_is_not_captured_during_construction():
    """Regression: under use_sim_time the ROS clock still reads 0 inside
    __init__ because no /clock message has arrived yet. Capturing the baseline
    there made the first tick see the whole simulation uptime as elapsed
    (measured: 329 s), tripping position_timeout_s instantly and refusing to
    arm 2 ms after startup. The baseline must be taken on the first tick."""
    import inspect

    from offboard_mission import waypoint_node

    source = inspect.getsource(waypoint_node.WaypointMission.__init__)
    assert 'self._start_time = None' in source
    assert 'self._start_time = self.get_clock()' not in source


def test_scan_trusts_the_tf_instead_of_settling_it_again():
    """landing_circle now withholds the safety_point TF until it has collected
    for scan_collect_s seconds, so the TF appearing already means the decision
    is final. Waiting for it to stop moving a second time added 5-8 s of hover
    and no extra confidence."""
    import inspect

    from offboard_mission import waypoint_node

    assert not hasattr(waypoint_node, 'StablePointDetector')

    source = inspect.getsource(waypoint_node.WaypointMission)
    assert 'StablePointDetector' not in source
    assert '_scan_detector' not in source


def test_state_machine_covers_the_whole_landing_mission():
    from offboard_mission.waypoint_node import State

    for name in ('SCAN', 'APPROACH', 'DESCEND', 'LANDING', 'LANDED'):
        assert hasattr(State, name), '%s missing from the mission states' % name


def test_goto_hands_over_to_scan_not_hold():
    """HOLD is now the failure/idle state. Arriving at the waypoint must start
    the landing-point search, which is the whole point of this mission."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = inspect.getsource(WaypointMission._tick_goto)
    assert '_begin_scan()' in source
    assert 'State.HOLD' not in source


def _code_without_docstring(func):
    """Source of `func` with its docstring removed.

    These contract tests look for the absence of certain calls. Prose that
    merely NAMES the thing being avoided — which good comments do — would
    otherwise fail them, so only executable lines are inspected.
    """
    import ast
    import inspect
    import textwrap

    tree = ast.parse(textwrap.dedent(inspect.getsource(func)))
    body = tree.body[0].body
    if (body and isinstance(body[0], ast.Expr)
            and isinstance(body[0].value, ast.Constant)
            and isinstance(body[0].value.value, str)):
        body = body[1:]
    return '\n'.join(ast.unparse(node) for node in body)


def test_landing_state_stops_streaming_setpoints():
    """VEHICLE_CMD_NAV_LAND makes PX4 leave offboard mode. Continuing to
    stream setpoints after that fights the autopilot's own landing controller
    for the vehicle, so the tick for this state must publish nothing."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission._tick_landing)
    assert '_publish_position_setpoint' not in source
    assert '_publish_hold_velocity' not in source
    assert '_publish_offboard_mode' not in source


def test_shutdown_treats_every_airborne_state_as_airborne():
    """Ctrl+C during the scan or the descent leaves a flying vehicle just as
    much as during GOTO. Omitting the new states would silently drop the
    warning exactly when it matters most."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = inspect.getsource(WaypointMission.shutdown)
    for state in ('TAKEOFF', 'GOTO', 'SCAN', 'APPROACH', 'DESCEND'):
        assert 'State.%s' % state in source, '%s not covered by shutdown' % state


def test_tf_freshness_uses_the_ros_clock_not_the_wall_clock():
    """Same two-clock discipline as the rest of the node: durations follow the
    ROS clock so they track simulation time. Measuring TF age against
    now_us() would compare a simulated stamp with a wall-clock reading and
    declare every target stale by decades."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission._safety_point)
    assert 'self.get_clock()' in source
    assert 'now_us()' not in source


def test_abort_is_disabled_below_the_floor():
    """Close to the surface the downward camera loses the plane, so the target
    always goes stale at the end of a descent. Without a commit floor the
    mission would abort every single landing and loop forever."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = inspect.getsource(WaypointMission._should_abort)
    assert '_abort_floor_m' in source


def test_shutdown_stops_the_perception_processes():
    """The children live in their own process groups, so anything shutdown()
    misses outlives this process — GNG at ~700% CPU with no terminal that owns
    it. This is the single most costly thing to forget."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission.shutdown)
    assert '_perception.stop(' in source


def test_perception_starts_only_on_the_first_scan_attempt():
    """A re-scan after an aborted approach must not restart the pipeline: the
    processes are still up, and the candidate registry they have accumulated
    is worth more than a clean slate."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission._begin_scan)
    assert '_scan_attempts == 1' in source
    assert '_perception.start()' in source


def test_scan_reports_a_dead_perception_process_immediately():
    """Without this the mission hovers for the whole scan timeout and then
    blames a missing TF, pointing the operator at the wrong thing."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission._tick_scan)
    assert '_perception.poll()' in source


def test_mission_broadcasts_the_scan_centre_frame():
    """landing_circle selects relative to its `base_frame`. The mission points
    that at a frame it broadcasts itself, so selection becomes 'nearest to the
    waypoint' instead of 'nearest to the drone' — the latter kept changing
    simply because the drone was moving, which read as a lost target."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = inspect.getsource(WaypointMission)
    assert 'TransformBroadcaster' in source
    assert 'StaticTransformBroadcaster' not in source     # the frame can move


def test_scan_frame_name_has_a_single_source_of_truth():
    """The frame the mission broadcasts and the frame landing_circle selects
    against must be the same string. Deriving the command-line argument from
    the same parameter is what stops them drifting apart."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = inspect.getsource(WaypointMission.__init__)
    assert "'base_frame:=%s' % self._scan_frame" in source


def test_landing_circle_is_run_with_sticky_selection():
    """Without this the TF moves whenever the drone moves, and the mission's
    abort test cannot tell that apart from a genuinely blocked target. Measured
    in flight: three aborted landings in a row, every candidate still safe."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = inspect.getsource(WaypointMission.__init__)
    assert 'sticky_target:=true' in source


def test_commit_floor_only_applies_while_tracking_is_good():
    """Measured in flight: the vehicle dropped ~9 m in 2 s, which put it under
    abort_floor_m instantly, and the mission then committed to a landing it
    could no longer see. Committing makes sense when the vehicle is following
    its setpoint and has merely run out of camera view. A vehicle 7 m from its
    own setpoint is not in that situation, and committing is the opposite of
    the right response."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission._should_abort)
    assert '_abort_floor_m' in source
    assert '_tracking_ok()' in source


def test_tracking_is_unknown_until_proven():
    """No position or no setpoint yet means tracking is NOT confirmed, which
    must keep the abort checks enabled rather than disable them."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission._tracking_ok)
    assert 'return False' in source


def test_landing_reports_how_far_off_it_touched_down():
    """The mission logged 'Mission complete' after coming to rest 12.0 m from
    the point it chose. A landing that misses its target is a failure to
    report, not a success."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission._tick_landing)
    assert '_report_touchdown()' in source

    report = _code_without_docstring(WaypointMission._report_touchdown)
    assert 'horizontal_distance' in report
    assert '_landing_miss_warn_m' in report
    # A miss must be reported at error level, not buried in an info line.
    assert 'get_logger().error' in report


def test_waypoint_node_resolves_horizontal_distance():
    from offboard_mission import waypoint_node

    assert hasattr(waypoint_node, 'horizontal_distance')


def test_mission_broadcasts_the_point_it_just_abandoned():
    """Selection in landing_circle is sticky, so nothing makes it let go of a
    point the mission gave up on — it would re-lock the same one, which reads
    from outside as a landing point stuck at the old position. The mission
    publishes the abandoned point as a frame so the selection moves on."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    whole = inspect.getsource(WaypointMission)
    assert '_reject_frame' in whole
    assert '_publish_reject_tf' in whole

    abort = _code_without_docstring(WaypointMission._abort_to_scan)
    assert '_rejected_point' in abort


def test_scan_refuses_to_relock_the_abandoned_point():
    """Belt and braces: landing_circle is told to exclude it, but the mission
    must not depend on someone else running with the right parameters to keep
    its own promise."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission._tick_scan)
    assert '_is_rejected(' in source


def test_the_rejection_expires_rather_than_lasting_for_ever():
    """A rejection means 'I just failed there', not 'that place is condemned'.
    Once the obstacle has moved on, the registry's own blocked/recover logic is
    a better judge than a stale memory."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    whole = inspect.getsource(WaypointMission)
    assert '_reject_hold_s' in whole


def test_mission_passes_the_collection_window_to_landing_circle():
    """One source of truth: the window the mission waits for and the window
    landing_circle counts must be the same number."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = inspect.getsource(WaypointMission.__init__)
    assert "'commit_after_s:=%.3f' % self._scan_collect_s" in source


def test_scan_locks_the_first_tf_it_is_given():
    """With the decision made upstream, SCAN's job is to notice the TF and go —
    minus points this mission already abandoned."""
    import inspect

    from offboard_mission.waypoint_node import WaypointMission

    source = _code_without_docstring(WaypointMission._tick_scan)
    assert '_safety_point()' in source
    assert '_is_rejected(' in source
