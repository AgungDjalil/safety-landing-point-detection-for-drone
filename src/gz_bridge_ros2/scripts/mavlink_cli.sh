#!/bin/sh
# mavlink_cli.sh — interactive MAVProxy CLI for PX4 SITL ("commander")
#
# Bridges you into the PX4 SITL flight stack via MAVLink so you can send
# commander commands (arm, takeoff, land, mode, params, etc.) from the shell.
#
# Default endpoint: UDP 127.0.0.1:14550 (the PX4 SITL GCS broadcast port,
# see ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink — MAVLink "GCS link"
# instance px4_instance=0; PX4 listens on 18570, broadcasts to 14550).
#
# Usage:
#   ros2 run gz_bridge_ros2 mavlink_cli                         # default port 14550
#   ros2 run gz_bridge_ros2 mavlink_cli --master=udp:127.0.0.1:14550    # explicit
#   ros2 run gz_bridge_ros2 mavlink_cli -- --map --console      # pass-through extra opts
#
# Pass --help to see MAVProxy options. Exit with `Ctrl-D` or `exit`.
#
# Prerequisites:
#   - MAVProxy + pymavlink installed. On this machine it is already at
#     /home/alphaone/.local/bin/mavproxy.py. Otherwise install with:
#         python3 -m pip install --user MAVProxy pymavlink
#   - PX4 SITL running (e.g. via `ros2 launch gz_bridge_ros2 depth_bridge_launch.py`)
#     — the MAVLink port is only open once PX4 has booted.
#
# Why a script and not a launch file: MAVProxy is interactive and needs a TTY
# for stdin; `ros2 launch` does not reliably forward stdin to spawned processes,
# so we expose this as a `ros2 run` executable which keeps the terminal
# attached.

PORT="${MAVLINK_CLI_PORT:-14550}"

# Locate mavproxy.py: honour an explicit override, else search PATH.
if [ -n "${MAVPY:-}" ]; then
    MAVP="$MAVPY"
else
    MAVP="$(command -v mavproxy.py || true)"
    # Fall back to the user-local install on this machine.
    if [ -z "$MAVP" ] && [ -x "/home/alphaone/.local/bin/mavproxy.py" ]; then
        MAVP="/home/alphaone/.local/bin/mavproxy.py"
    fi
fi

if [ -z "$MAVP" ] || [ ! -x "$MAVP" ]; then
    echo "mavlink_cli: mavproxy.py not found." >&2
    echo "  Install it with:  python3 -m pip install --user MAVProxy pymavlink" >&2
    echo "  Or set MAVPY=/path/to/mavproxy.py" >&2
    exit 1
fi

# If the caller passed an explicit --master (or any MAVProxy option that should
# override the default), forward verbatim. Otherwise inject the default master.
has_master=0
for arg in "$@"; do
    case "$arg" in
        --master=*) has_master=1 ;;
        --master)   has_master=1 ;;
    esac
done

if [ "$has_master" -eq 1 ]; then
    exec "$MAVP" "$@"
else
    exec "$MAVP" --master="udp:127.0.0.1:${PORT}" "$@"
fi