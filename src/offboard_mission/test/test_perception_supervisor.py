"""Unit tests for PerceptionSupervisor.

Nothing here forks a process. `spawn`, `killpg`, `getpgid` and the running-node
lookup are all injected, so the supervision logic — argv construction, the
duplicate guard, signal escalation — is tested on its own. The parts that
cannot be faked (that SIGINT really reaches a `ros2 run` child) are covered by
the SITL run instead.
"""
import signal
import subprocess

import pytest


def _mod():
    from offboard_mission import perception_supervisor
    return perception_supervisor


class FakeProc:
    """Minimal stand-in for subprocess.Popen."""

    def __init__(self, pid, argv, hang=False):
        self.pid = pid
        self.argv = argv
        self.hang = hang
        self.waits = []
        self._rc = None

    def poll(self):
        return self._rc

    def wait(self, timeout=None):
        self.waits.append(timeout)
        if self.hang and self._rc is None:
            raise subprocess.TimeoutExpired(self.argv, timeout)
        if self._rc is None:
            self._rc = 0
        return self._rc

    def die(self, code=1):
        self._rc = code


class Harness:
    """Injected collaborators plus the record of what they were asked to do."""

    def __init__(self, running=(), hang_pids=()):
        self.spawned = []            # argv lists, in order
        self.procs = []
        self.signals = []            # (target, signum)
        self.running = list(running)
        self.hang_pids = set(hang_pids)
        self._next_pid = 100
        self.missing_groups = set()

    def spawn(self, argv):
        pid = self._next_pid
        self._next_pid += 1
        proc = FakeProc(pid, argv, hang=pid in self.hang_pids)
        self.spawned.append(argv)
        self.procs.append(proc)
        return proc

    def getpgid(self, pid):
        return 1000 + pid            # distinct from the pid on purpose

    def killpg(self, pgid, signum):
        if pgid in self.missing_groups:
            raise ProcessLookupError(pgid)
        self.signals.append((pgid, signum))
        for proc in self.procs:
            if 1000 + proc.pid == pgid and signum == signal.SIGKILL:
                proc.hang = False    # SIGKILL is not survivable
                proc.die(-9)

    def list_running_nodes(self):
        return list(self.running)

    def build(self, commands, **kwargs):
        return _mod().PerceptionSupervisor(
            commands,
            spawn=self.spawn,
            killpg=self.killpg,
            getpgid=self.getpgid,
            list_running_nodes=self.list_running_nodes,
            **kwargs)


GNG = 'dbl_gng_cpu: ros2 run gng_node dbl_gng_cpu_node'
CIRCLE = ('landing_circle: ros2 run segmentation_node landing_circle '
          '--ros-args -p input_topic:=/plane_cpu')


# ---------------------------------------------------------------------------
# start()
# ---------------------------------------------------------------------------

def test_start_spawns_each_command_split_into_argv():
    h = Harness()
    sup = h.build([GNG, CIRCLE])
    sup.start()

    assert h.spawned[0] == ['ros2', 'run', 'gng_node', 'dbl_gng_cpu_node']
    assert h.spawned[1] == [
        'ros2', 'run', 'segmentation_node', 'landing_circle',
        '--ros-args', '-p', 'input_topic:=/plane_cpu']


def test_node_name_prefix_is_stripped_from_the_command():
    """The `name: command` form carries the ROS node name for the duplicate
    guard. It must never leak into the argv that gets executed."""
    h = Harness()
    h.build([GNG]).start()

    assert 'dbl_gng_cpu:' not in h.spawned[0]
    assert h.spawned[0][0] == 'ros2'


def test_commands_without_a_name_prefix_still_run():
    h = Harness()
    h.build(['ros2 run some_pkg some_node']).start()

    assert h.spawned == [['ros2', 'run', 'some_pkg', 'some_node']]


def test_a_ros_arg_assignment_is_not_mistaken_for_a_name_prefix():
    """`input_topic:=/plane_cpu` contains a colon. Only a leading
    `identifier: ` counts as the node-name prefix."""
    h = Harness()
    h.build(['ros2 run p n --ros-args -p input_topic:=/plane_cpu']).start()

    assert h.spawned[0][0] == 'ros2'
    assert '-p' in h.spawned[0]


def test_extra_args_are_appended_in_their_own_block():
    h = Harness()
    h.build([GNG], extra_args=['-p', 'use_sim_time:=true']).start()

    assert h.spawned[0][-3:] == ['--ros-args', '-p', 'use_sim_time:=true']


def test_no_ros_args_block_is_added_when_there_are_no_extra_args():
    h = Harness()
    h.build([GNG], extra_args=[]).start()

    assert '--ros-args' not in h.spawned[0]


def test_extra_args_survive_an_existing_ros_args_block():
    """rcl accepts more than one --ros-args block on a command line (verified
    against a real node), so extras can simply be appended rather than merged
    into whatever block the operator wrote."""
    h = Harness()
    h.build([CIRCLE], extra_args=['-p', 'sticky_target:=true']).start()

    argv = h.spawned[0]
    assert argv.count('--ros-args') == 2
    assert argv[-3:] == ['--ros-args', '-p', 'sticky_target:=true']
    assert 'input_topic:=/plane_cpu' in argv       # the original block survives


def test_extra_args_go_to_every_command():
    """One source of truth for the scan-centre frame name beats a value copied
    into each command string, where the copies can silently drift apart."""
    h = Harness()
    h.build([GNG, CIRCLE], extra_args=['-p', 'base_frame:=scan_center']).start()

    for argv in h.spawned:
        assert argv[-1] == 'base_frame:=scan_center'


def test_start_skips_a_command_whose_node_is_already_running():
    """Two GNG processes would be 1400% CPU on a 12-core machine. An already
    running node is left alone, and the rest still start."""
    h = Harness(running=['dbl_gng_cpu'])
    sup = h.build([GNG, CIRCLE])
    started, skipped = sup.start()

    assert len(h.spawned) == 1
    assert h.spawned[0][3] == 'landing_circle'
    assert skipped == ['dbl_gng_cpu']
    assert len(started) == 1


def test_start_reports_when_everything_was_already_running():
    h = Harness(running=['dbl_gng_cpu', 'landing_circle'])
    started, skipped = h.build([GNG, CIRCLE]).start()

    assert h.spawned == []
    assert started == []
    assert skipped == ['dbl_gng_cpu', 'landing_circle']


def test_start_twice_does_not_spawn_twice():
    h = Harness()
    sup = h.build([GNG, CIRCLE])
    sup.start()
    sup.start()

    assert len(h.spawned) == 2


def test_empty_command_list_starts_nothing_and_is_not_running():
    h = Harness()
    sup = h.build([])
    sup.start()

    assert h.spawned == []
    assert sup.running is False


def test_running_is_true_only_after_a_successful_start():
    h = Harness()
    sup = h.build([GNG])
    assert sup.running is False
    sup.start()
    assert sup.running is True


# ---------------------------------------------------------------------------
# poll()
# ---------------------------------------------------------------------------

def test_poll_is_quiet_while_every_child_lives():
    h = Harness()
    sup = h.build([GNG, CIRCLE])
    sup.start()

    assert sup.poll() is None


def test_poll_names_the_command_that_died():
    """A mistyped command or an unbuilt package exits in milliseconds. Without
    this the mission would hover until scan_timeout_s and then blame the TF."""
    h = Harness()
    sup = h.build([GNG, CIRCLE])
    sup.start()
    h.procs[1].die(127)

    reason = sup.poll()
    assert reason is not None
    assert 'landing_circle' in reason
    assert '127' in reason


def test_poll_before_start_is_quiet():
    h = Harness()
    assert h.build([GNG]).poll() is None


# ---------------------------------------------------------------------------
# stop()
# ---------------------------------------------------------------------------

def test_stop_signals_the_process_group_not_the_pid():
    """`ros2 run` is a wrapper: the node itself is its child. Signalling only
    the wrapper's pid leaves the node orphaned and still burning CPU."""
    h = Harness()
    sup = h.build([GNG])
    sup.start()
    pid = h.procs[0].pid
    sup.stop()

    assert (1000 + pid, signal.SIGINT) in h.signals
    assert (pid, signal.SIGINT) not in h.signals


def test_stop_sends_sigint_to_every_child():
    h = Harness()
    sup = h.build([GNG, CIRCLE])
    sup.start()
    sup.stop()

    sigints = [s for s in h.signals if s[1] == signal.SIGINT]
    assert len(sigints) == 2


def test_stop_escalates_to_sigkill_only_when_the_grace_expires():
    h = Harness(hang_pids=[100])
    sup = h.build([GNG, CIRCLE])
    sup.start()
    sup.stop(grace_s=0.5)

    kills = [s for s in h.signals if s[1] == signal.SIGKILL]
    assert kills == [(1100, signal.SIGKILL)]      # only the hung one
    assert h.procs[0].waits[0] == 0.5


def test_stop_does_not_signal_a_child_that_already_exited():
    h = Harness()
    sup = h.build([GNG])
    sup.start()
    h.procs[0].die(0)
    sup.stop()

    assert h.signals == []


def test_stop_is_idempotent():
    """LANDED and shutdown() can both fire. The second call must be a no-op,
    not a second round of signals at a recycled pid."""
    h = Harness()
    sup = h.build([GNG, CIRCLE])
    sup.start()
    sup.stop()
    before = len(h.signals)
    sup.stop()

    assert len(h.signals) == before
    assert sup.running is False


def test_stop_before_start_does_nothing():
    h = Harness()
    h.build([GNG]).stop()          # must not raise


def test_stop_tolerates_a_group_that_vanished():
    """The child can exit between poll() and killpg(). ProcessLookupError is
    the normal outcome of that race, not a failure."""
    h = Harness()
    sup = h.build([GNG])
    sup.start()
    h.missing_groups.add(1000 + h.procs[0].pid)

    sup.stop()                     # must not raise
    assert sup.running is False


def test_start_after_stop_spawns_again():
    h = Harness()
    sup = h.build([GNG])
    sup.start()
    sup.stop()
    sup.start()

    assert len(h.spawned) == 2


# ---------------------------------------------------------------------------
# malformed input
# ---------------------------------------------------------------------------

def test_blank_entries_are_ignored():
    h = Harness()
    sup = h.build(['', '   ', GNG])
    sup.start()

    assert len(h.spawned) == 1


def test_an_unparsable_command_is_reported_not_swallowed():
    h = Harness()
    sup = h.build(['ros2 run pkg "unterminated'])

    with pytest.raises(ValueError):
        sup.start()
