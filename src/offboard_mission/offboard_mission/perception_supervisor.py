"""Start and stop the perception chain as child processes.

The landing-point pipeline (`dbl_gng_cpu_node` -> `landing_circle`) is only
useful once the vehicle is hovering over the area it should scan. Running it
from launch means it burns CPU through arming, takeoff and the whole transit,
and `dbl_gng_cpu_node` alone was measured at ~701% CPU on a 12-core machine —
enough to starve Gazebo, the camera bridge and RViz.

So the mission node owns those processes and starts them on arrival. This
module holds that supervision on its own, away from the flight state machine:
process groups, signal escalation and the duplicate guard are fiddly enough to
deserve their own tests, and none of it should need a live ROS graph to
exercise.

Every collaborator that touches the operating system is injected, which is
what makes the tests hermetic.
"""
import os
import re
import shlex
import signal
import subprocess

# `name: command`. The name is the ROS node the command is expected to create,
# used for the duplicate guard. Anchored and restricted to an identifier so a
# ROS argument like `input_topic:=/plane_cpu` can never be read as a prefix.
_NAME_PREFIX = re.compile(r'^([A-Za-z_][A-Za-z0-9_]*):\s+(.*)$')


class _Child:
    __slots__ = ('name', 'command', 'proc')

    def __init__(self, name, command, proc):
        self.name = name
        self.command = command
        self.proc = proc

    def label(self):
        return self.name or self.command


class PerceptionSupervisor:
    """Own the perception child processes for one mission.

    `extra_args` are appended, inside their own `--ros-args` block, to every
    command — the caller uses this for settings it must keep as the single
    source of truth (the scan-centre frame name, use_sim_time).

    `commands` are shell-ish strings, optionally prefixed with the ROS node
    name they produce:

        dbl_gng_cpu: ros2 run gng_node dbl_gng_cpu_node

    The prefix is what lets `start()` skip a node somebody already has running
    in another terminal, which matters here: a second GNG would double an
    already ruinous CPU load rather than fail loudly.
    """

    def __init__(self, commands, extra_args=None, spawn=None,
                 killpg=None, getpgid=None, list_running_nodes=None,
                 logger=None):
        self._commands = [c for c in (commands or []) if c and c.strip()]
        self._extra_args = list(extra_args or [])
        self._spawn = spawn or self._default_spawn
        self._killpg = killpg or os.killpg
        self._getpgid = getpgid or os.getpgid
        self._list_running_nodes = list_running_nodes or (lambda: [])
        self._logger = logger
        self._children = []

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Spawn every command that is not already running.

        Returns `(started, skipped)`: the commands actually launched, and the
        node names that were left alone because they were already up.
        """
        if self._children:
            return ([c.command for c in self._children], [])

        running = set(self._list_running_nodes())
        started, skipped = [], []

        for command in self._commands:
            name, bare = self._split_name(command)
            if name and name in running:
                skipped.append(name)
                self._log('info', "'%s' is already running — not starting a "
                                  'second one.' % name)
                continue

            argv = self._argv(bare)
            self._children.append(_Child(name, bare, self._spawn(argv)))
            started.append(bare)
            self._log('info', 'Started perception process: %s' % ' '.join(argv))

        return (started, skipped)

    def poll(self):
        """None while every child is alive, else why the first dead one died.

        A mistyped command or an unbuilt package exits within milliseconds.
        Reporting that immediately is the difference between "landing_circle
        exited with code 127" and a mission that hovers for the full scan
        timeout and then blames a missing TF.
        """
        for child in self._children:
            code = child.proc.poll()
            if code is not None:
                return ("perception process '%s' exited with code %s (%s)"
                        % (child.label(), code, child.command))
        return None

    def stop(self, grace_s=3.0):
        """SIGINT the children, then SIGKILL whatever outlives the grace.

        Signals go to the process GROUP. `ros2 run` is a wrapper that runs the
        node as its own child, so signalling just the pid we hold would reap
        the wrapper and orphan the node — which would leave GNG running at
        700% CPU with no terminal that owns it.

        Safe to call more than once: LANDED and shutdown() can both reach it.
        """
        pending = []
        for child in self._children:
            if child.proc.poll() is not None:
                continue
            if self._signal(child, signal.SIGINT):
                pending.append(child)

        for child in pending:
            try:
                child.proc.wait(timeout=grace_s)
            except subprocess.TimeoutExpired:
                self._log('warn', "'%s' ignored SIGINT for %.1fs — killing."
                                  % (child.label(), grace_s))
                if self._signal(child, signal.SIGKILL):
                    try:
                        child.proc.wait(timeout=grace_s)
                    except subprocess.TimeoutExpired:
                        self._log('error', "'%s' survived SIGKILL."
                                           % child.label())

        self._children = []

    @property
    def running(self):
        return bool(self._children)

    # ── internals ────────────────────────────────────────────────────────────

    @staticmethod
    def _default_spawn(argv):
        # A new session puts the child in its own process group, which is what
        # makes the group-wide signal in stop() possible.
        return subprocess.Popen(argv, start_new_session=True)

    @staticmethod
    def _split_name(command):
        match = _NAME_PREFIX.match(command.strip())
        if match is None:
            return (None, command.strip())
        return (match.group(1), match.group(2).strip())

    def _argv(self, command):
        argv = shlex.split(command)
        if self._extra_args:
            # rcl accepts several --ros-args blocks on one command line
            # (verified against a running node), so this appends rather than
            # trying to merge into whatever block the operator already wrote.
            #
            # The same block goes to every command even when a setting only
            # means something to one of them. An override for a parameter a
            # node never declares is simply unused — verified against both the
            # C++ and the rclpy node — and that costs far less than letting a
            # frame name live in two places that can silently disagree.
            argv += ['--ros-args'] + self._extra_args
        return argv

    def _signal(self, child, signum):
        """Signal the child's process group. False if it was already gone."""
        try:
            self._killpg(self._getpgid(child.proc.pid), signum)
        except (ProcessLookupError, OSError):
            return False
        return True

    def _log(self, level, message):
        if self._logger is not None:
            getattr(self._logger, level)(message)
