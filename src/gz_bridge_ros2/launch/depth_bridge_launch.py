"""
Bring up the PX4 SITL depth-camera stack for ROS 2.

Starts the microXRCE-DDS agent, the Gazebo->ROS 2 depth-camera bridge and the
TF tree, and optionally PX4 SITL itself, a plane-segmentation front-end and
RViz2.

Operating modes
---------------
start_px4:=false (DEFAULT — interactive pxh> prompt)
    Launch only starts the ROS 2 side:
      1. microXRCE-DDS agent            (MicroXRCEAgent udp4 -p 8888)
      2. gz clock bridge                (/clock gz.msgs.Clock -> ROS 2 /clock)
      3. drone_kinematic node           (map -> odom -> base_link [dynamic] +
                                         base_link -> camera_link [static];
                                         use_sim_time:=true)
      4. ros_gz_bridge parameter_bridge (/depth_camera, /depth_camera/points)
      5. (optional) plane segmentation  (perception:=gng_cpu|gng_gpu|ransac;
                                         use_sim_time:=true)
      6. (optional) rviz2               (-d <config>/x500_depth.rviz; use_sim_time:=true)
    Run gz sim + PX4 manually in separate terminals (the launch prints the
    exact copy-paste commands on startup):
      A) gz sim -r -s <px4_dir>/Tools/simulation/gz/worlds/<world>.sdf
      B) cd <px4_dir> && PX4_GZ_WORLD=<world> PX4_UXRCE_DDS_PORT=<port> \
         make px4_sitl gz_<model>        -> interactive pxh> prompt
      C) gz sim -g                       (optional GUI)
    PX4 auto-detects the running gz server (px4-rc.gzsim checks
    /world/<name>/clock) and attaches — spawn model + start gz_bridge — so
    the manual command is identical to the bundled one minus the gz
    auto-launch. Agent + bridges below wait-and-pickup gz/PX4 topics.

start_px4:=true (legacy — non-interactive)
    Launch additionally runs `PX4_GZ_WORLD=<world> make px4_sitl gz_<model>`
    as a child process. PX4 auto-launches Gazebo (px4-rc.gzsim) and the
    uxrce_dds_client on UDP localhost:<dds_port>. PX4's pxh> prompt does
    NOT accept stdin under ros2 launch — for commander interaction use
    start_px4:=false instead.

Resulting TF tree (all published from this single launch):
    map -> odom -> base_link -> camera_link

The depth pointcloud frame_id is "camera_link" (polos, no instance prefix —
confirmed at runtime via ros_gz_bridge), which directly matches the
base_link -> camera_link TF leg from drone_kinematic, so no frame alias is
needed.

The depth-camera Gazebo topics are the explicit short names declared by the
OakD-Lite model (Tools/simulation/gz/models/OakD-Lite/model.sdf:<topic>depth_camera</topic>),
i.e. /depth_camera (gz.msgs.Image, R_FLOAT32) and /depth_camera/points
(gz.msgs.PointCloudPacked). The bridged ROS 2 messages have frame_id
"camera_link" (polos, no instance prefix — confirmed at runtime).

Usage
-----
    cd ~/ros2_ws
    colcon build --packages-select gz_bridge_ros2
    source install/setup.bash
    ros2 launch gz_bridge_ros2 depth_bridge_launch.py

Overrides
---------
    ros2 launch gz_bridge_ros2 depth_bridge_launch.py \
        world:=rubicon model:=x500_depth instance:=x500_depth_0 \
        dds_port:=8888 agent_bin:=MicroXRCEAgent \
        headless:=false rviz:=true px4_dir:=~/PX4-Autopilot \
        start_px4:=false perception:=gng_cpu \
        perception_input:=/depth_camera/points voxel_leaf:=0.15

Plane segmentation front-end (`perception`)
-------------------------------------------
One argument, one choice — two front-ends can never be running at once.

    perception:=none      (default) nothing started
    perception:=gng_cpu   gng_node/dbl_gng_cpu_node    -> /plane_cpu, /outlier_cpu
                          (numpy + ThreadPoolExecutor, no torch/GPU)
    perception:=gng_gpu   gng_node/dbl_gng_node        -> /plane_cpu, /outlier_cpu
                          (torch; auto cuda/cpu fallback)
    perception:=ransac    segmentation_node/plane_segmentation_ransac
                                                       -> /plane, /outlier

    perception_input  input PointCloud2 topic, default '/depth_camera/points'
                      (this bridge). Set to
                      '/zed/zed_node/point_cloud/cloud_registered' for a real
                      ZED camera.
    voxel_leaf        voxel cell size in metres, default 0.15. Passed to BOTH
                      front-ends -- see below.

Every front-end runs with use_sim_time:=true so its output stamps are on the
gz sim clock the TF tree uses; otherwise RViz2's MessageFilter drops every
message ("timestamp earlier than all data in the transform cache").

Feeding both front-ends the same cloud
--------------------------------------
The point of one launch offering both is to compare them, and that comparison
is only about the algorithms if they receive the same input. Three things used
to differ; all three are settled here.

    stage           GNG (cpu)              RANSAC (as shipped)
    topic           /depth_camera/points   /depth_camera/points
    drop NaN        yes                    yes
    cylinder crop   none                   cylinder_crop in front (node deleted)
    spatial filter  none                   PassThrough z in [-5, 5]
    voxel           voxel_leaf = 0.15      leaf_size = 0.07

- cylinder_crop is GONE from this launch, and the node itself has since been
  DELETED from segmentation_node. It kept points within a radius of
  the x-y origin, but in camera_link x is the DEPTH axis: measured in flight
  at an 11 m scan altitude, /depth_camera/points spans x 0.21..14.81 m
  (median 12.51) while the ground plane lies in y-z -- which is why
  landing_circle is configured plane_axes=yz. A 5 m cylinder on x-y therefore
  throws the ground away: 19200 points in, 4 out, and RANSAC then publishes
  nothing at all, so safety_point never appears and the mission times out
  holding at the waypoint. It only appears to work with the camera close to
  the ground. ransac_pipeline.launch.py still has that chain if it is ever
  wanted back.
- The PassThrough is DISABLED here (z_min/z_max +-1000). z is a lateral axis
  in camera_link, spanning -8.15..7.40 m at that altitude, so [-5, 5] cuts the
  outer band of a field of view GNG keeps. That is an input filter, not part
  of the algorithm.
- `voxel_leaf` reaches BOTH: GNG calls the parameter voxel_leaf, RANSAC calls
  it leaf_size, and this launch maps one argument onto both names so they can
  never drift apart. Without it the two ran at 19200->6445 (RANSAC) against
  19200->~2000 (GNG), and the computation times were not comparable.

What is NOT touched: RANSAC's setAxis(1,0,0) + setEpsAngle(5 deg). That is a
model constraint -- part of the algorithm being compared, not of its input.

`gng_gpu` has no voxel stage at all (dbl_gng_node declares no voxel_leaf), so
voxel_leaf is not passed to it and it is not comparable with ransac on input
load. The valid comparison is gng_cpu against ransac.

The landing-point search is NOT started here
--------------------------------------------
`landing_circle` is started by offboard_mission/waypoint_node when the drone
arrives, via its `perception_commands` parameter — whose default hard-codes
`input_topic:=/plane_cpu`, the GNG topic. So `perception:=ransac` alone
publishes a /plane nothing reads. The launch prints the matching
`ros2 run offboard_mission waypoint_node` line for whichever mode was chosen;
copy it.

Notes
-----
- First run builds PX4 (`make px4_sitl gz_x500_depth`); subsequent runs are cached.
- The agent must be Micro-XRCE-DDS-Agent v2.x (not v3.x) to match PX4's client.
- PX4's interactive console does not accept stdin while it is a child of
  `ros2 launch`. For commander interaction (arming, mode changes) use the
  DEFAULT mode (start_px4:=false) and run PX4 manually in another terminal.

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node

# Every accepted value of `perception`. Kept as a tuple so the "unknown mode"
# guard below can be written as one `not in` test rather than a chain of
# comparisons that has to be edited in two places whenever a mode is added.
PERCEPTION_MODES = ("none", "gng_cpu", "gng_gpu", "ransac")

# Voxel cell size handed to whichever front-end runs. GNG's own default, so
# that "same input as GNG" is what you get without saying anything.
VOXEL_LEAF_DEFAULT = "0.15"

# Wide enough to disable plane_segmentation_ransac's PassThrough. Its default
# z in [-5, 5] cuts a lateral band of the field of view that GNG keeps; see
# the module docstring for the measurement.
NO_PASSTHROUGH_M = 1000.0


def _looks_like_px4_root(path):
    """
    Report whether `path` looks like the PX4-Autopilot repo root.

    The heuristic: a directory holding both a top-level Makefile and a
    boards/ tree.
    """
    return os.path.isfile(os.path.join(path, "Makefile")) and os.path.isdir(
        os.path.join(path, "boards")
    )


def _px4_source_dir():
    """
    Resolve the PX4-Autopilot repo root.

    This package is independent of the PX4 tree, so the root must come from
    an explicit source. Resolution order:
      1. $PX4_SOURCE_DIR env var (explicit user override, preferred).
      2. The conventional checkout at ~/PX4-Autopilot, if it looks like a PX4
         root.
      3. Walk up from this file's real location looking for the PX4 root
         marker (Makefile + boards/); covers the case where this workspace
         happens to live inside a PX4 checkout.
      4. The conventional default ~/PX4-Autopilot (lets the user
         see a meaningful error from `make` rather than a Python import-time
         failure).
    """
    env_dir = os.environ.get("PX4_SOURCE_DIR")
    if env_dir and os.path.isdir(env_dir) and _looks_like_px4_root(env_dir):
        return env_dir

    default = os.path.expanduser("~/PX4-Autopilot")
    if _looks_like_px4_root(default):
        return default

    here = os.path.dirname(os.path.realpath(__file__))
    parent = here
    while parent and parent != os.path.dirname(parent):
        if _looks_like_px4_root(parent):
            return parent
        parent = os.path.dirname(parent)

    return default


# ---- perception mode plumbing ---------------------------------------------
# `perception` is a launch ARGUMENT, so its value is not known while this
# module runs — every test on it has to be a substitution evaluated later, at
# launch time. The three helpers below are the only place that happens, which
# is also what makes the mode table testable without starting a ROS graph.

def mode_is(perception, *wanted):
    """
    Build a condition that holds when `perception` is one of `wanted`.

    `repr` of the tuple is embedded literally, so the expression that gets
    evaluated reads e.g. `'ransac' in ('gng_cpu', 'gng_gpu')`.
    """
    return IfCondition(
        PythonExpression(["'", perception, "' in ", repr(tuple(wanted))])
    )


def gng_params(perception_input, voxel_leaf, with_voxel):
    """
    Parameters for a DBL-GNG node.

    `with_voxel` is False for the GPU node, which has no downsampling stage
    and declares no `voxel_leaf`. An override for a parameter a node never
    declares is discarded without a word, so passing it there would look like
    the two backends were equalised when only one of them was.
    """
    params = {
        "use_sim_time": True,
        "pointcloud_topic": perception_input,
    }
    if with_voxel:
        params["voxel_leaf"] = voxel_leaf
    return params


def ransac_params(perception_input, voxel_leaf):
    """
    Parameters for plane_segmentation_ransac.

    `input_topic` is never left unset: the node's own default is
    /zed/zed_node/point_cloud/cloud_registered, a leftover from the real ZED
    camera, and a topic that does not exist in the simulation. A node
    subscribed to it stays alive and silent forever — the exact failure this
    chain has already suffered once.

    `leaf_size` is RANSAC's spelling of `voxel_leaf`, and z_min/z_max disable
    the PassThrough. Both exist so this node sees the same cloud GNG does;
    the module docstring carries the measurements.
    """
    return {
        "use_sim_time": True,
        "input_topic": perception_input,
        "leaf_size": voxel_leaf,
        "z_min": -NO_PASSTHROUGH_M,
        "z_max": NO_PASSTHROUGH_M,
    }


def perception_nodes(perception, perception_input, voxel_leaf):
    """
    Build every plane-segmentation front-end, keyed by ROS node name.

    Every node carries its own IfCondition, so the ones the chosen mode does
    not want are constructed but never run. The node NAMES matter beyond
    RViz: offboard_mission's PerceptionSupervisor skips starting a node that
    is already in `ros2 node list`, matching on exactly these names. That is
    why `dbl_gng_cpu` here is spelled the same as in waypoint_node's default
    `perception_commands` — a GNG started from this launch is then reused by
    the mission instead of being started a second time.
    """
    return {
        "dbl_gng_cpu": Node(
            package="gng_node",
            executable="dbl_gng_cpu_node",
            name="dbl_gng_cpu",
            output="screen",
            condition=mode_is(perception, "gng_cpu"),
            parameters=[gng_params(perception_input, voxel_leaf, True)],
        ),
        "dbl_gng": Node(
            package="gng_node",
            executable="dbl_gng_node",
            name="dbl_gng",
            output="screen",
            condition=mode_is(perception, "gng_gpu"),
            parameters=[gng_params(perception_input, voxel_leaf, False)],
        ),
        "plane_segmentation_ransac": Node(
            package="segmentation_node",
            executable="plane_segmentation_ransac",
            name="plane_segmentation_ransac",
            output="screen",
            condition=mode_is(perception, "ransac"),
            parameters=[ransac_params(perception_input, voxel_leaf)],
        ),
    }


def generate_launch_description():
    pkg_share = get_package_share_directory("gz_bridge_ros2")
    rviz_config = PathJoinSubstitution([pkg_share, "config", "x500_depth.rviz"])

    # ---- Declare all user-tunable arguments --------------------------------
    declare_world = DeclareLaunchArgument(
        "world",
        default_value="rubicon",
        description="Gazebo world name (e.g. rubicon, baylands, default, windy).",
    )
    declare_model = DeclareLaunchArgument(
        "model",
        default_value="x500_depth",
        description="PX4 Gazebo model name (without the gz_ prefix).",
    )
    declare_instance = DeclareLaunchArgument(
        "instance",
        default_value="x500_depth_0",
        description="Spawned model instance name (for TF frames).",
    )
    declare_dds_port = DeclareLaunchArgument(
        "dds_port",
        default_value="8888",
        description="microXRCE-DDS agent UDP port (must match UXRCE_DDS_PRT).",
    )
    declare_agent_bin = DeclareLaunchArgument(
        "agent_bin",
        default_value="MicroXRCEAgent",
        description="microXRCE-DDS agent binary name "
        "(MicroXRCEAgent or micro-xrce-dds-agent).",
    )
    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo headless (HEADLESS=1). true|false",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Start RViz2 with the x500_depth preset. true|false",
    )
    declare_px4_dir = DeclareLaunchArgument(
        "px4_dir",
        default_value=_px4_source_dir(),
        description="PX4-Autopilot repo root (where `make px4_sitl` is run).",
    )
    declare_start_px4 = DeclareLaunchArgument(
        "start_px4",
        default_value="false",
        description="true  = launch runs `make px4_sitl gz_<model>` bundled "
                    "(PX4 auto-launches gz sim). "
                    "false (default) = run gz sim + PX4 manually in separate "
                    "terminals for an interactive pxh> prompt; launch only "
                    "starts agent + clock/depth bridges + TF + perception + RViz.",
    )
    declare_perception = DeclareLaunchArgument(
        "perception",
        default_value="none",
        description="Plane-segmentation front-end to start: "
                    "'none' (default, nothing) | "
                    "'gng_cpu' (dbl_gng_cpu_node, numpy, no torch/GPU) | "
                    "'gng_gpu' (dbl_gng_node, torch with cuda/cpu fallback) | "
                    "'ransac' (plane_segmentation_ransac). "
                    "One choice, so two front-ends can never fight for the CPU.",
    )
    declare_perception_input = DeclareLaunchArgument(
        "perception_input",
        default_value="/depth_camera/points",
        description="Input PointCloud2 topic for the chosen front-end. "
                    "Default '/depth_camera/points' (Gazebo depth bridge). "
                    "Set to '/zed/zed_node/point_cloud/cloud_registered' for a "
                    "real ZED camera.",
    )
    declare_voxel_leaf = DeclareLaunchArgument(
        "voxel_leaf",
        default_value=VOXEL_LEAF_DEFAULT,
        description="Voxel cell size in metres, passed to BOTH front-ends: "
                    "GNG declares it as `voxel_leaf`, RANSAC as `leaf_size`. "
                    "One argument, so the two can never drift apart and make "
                    "their computation times incomparable. Default 0.15 is "
                    "GNG's own, so 'same input as GNG' needs no flag. Has no "
                    "effect on perception:=gng_gpu, which has no voxel stage.",
    )
    declare_path_trail = DeclareLaunchArgument(
        "path_trail",
        default_value="true",
        description="Start path_trail_node (publishes the drone's trajectory "
                    "as nav_msgs/Path on /drone_path for RViz2). true|false",
    )

    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    dds_port = LaunchConfiguration("dds_port")
    agent_bin = LaunchConfiguration("agent_bin")
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    px4_dir = LaunchConfiguration("px4_dir")
    start_px4 = LaunchConfiguration("start_px4")
    perception = LaunchConfiguration("perception")
    perception_input = LaunchConfiguration("perception_input")
    voxel_leaf = LaunchConfiguration("voxel_leaf")
    path_trail = LaunchConfiguration("path_trail")

    # ---- (1) microXRCE-DDS agent ------------------------------------------
    # Must start before PX4; PX4's uxrce_dds_client retries until it connects.
    agent_proc = ExecuteProcess(
        cmd=[agent_bin, "udp4", "-p", dds_port],
        output="screen",
    )

    # ---- (2) PX4 SITL + Gazebo --------------------------------------------
    # Two operating modes controlled by `start_px4`:
    #
    # start_px4:=true  (legacy) — launch runs `PX4_GZ_WORLD=<world>
    #   make px4_sitl gz_<model>`, which auto-launches Gazebo (px4-rc.gzsim)
    #   and the uxrce_dds_client on UDP localhost:<dds_port>. PX4's pxh>
    #   prompt does NOT accept stdin under ros2 launch — for interactive
    #   commander use start_px4:=false instead.
    #
    # start_px4:=false (default) — launch does NOT start PX4 nor Gazebo.
    #   Run these manually in separate terminals for an interactive pxh>:
    #     A) gz sim -r -s <px4_dir>/Tools/simulation/gz/worlds/<world>.sdf
    #     B) cd <px4_dir> && PX4_GZ_WORLD=<world> \
    #        PX4_UXRCE_DDS_PORT=<dds_port> make px4_sitl gz_<model>
    #     C) gz sim -g   (optional GUI)
    #   PX4 auto-detects the running gz server (px4-rc.gzsim:34-35 via
    #   /world/<name>/clock) and attaches to it (spawn model + gz_bridge),
    #   so the manual command is identical to the bundled one minus the
    #   auto-launch. The agent + clock/depth bridges started below will
    #   wait-and-pickup gz/PX4 topics once they appear.
    headless_env = PythonExpression(
        ["'1' if '", headless, "' == 'true' else ''"]
    )
    px4_proc = ExecuteProcess(
        cmd=["make", "px4_sitl", PythonExpression(["'gz_' + '", model, "'"])],
        cwd=px4_dir,
        additional_env={
            "PX4_GZ_WORLD": world,
            "HEADLESS": headless_env,
            "PX4_UXRCE_DDS_PORT": dds_port,
        },
        output="screen",
        condition=IfCondition(start_px4),
    )
    # When start_px4:=false, print the exact manual commands (copy-paste ready).
    # LogInfo.msg accepts a list of strings/substitutions that get concatenated
    # at launch-time — no PythonExpression needed (avoids quote-escaping issues).
    manual_px4_info = LogInfo(
        msg=[
            "\n=== Manual PX4 workflow (start_px4:=false) ===\n",
            "Terminal A (gz server):\n",
            "  gz sim -r -s ", px4_dir,
            "/Tools/simulation/gz/worlds/", world, ".sdf\n",
            "Terminal B (PX4, interactive pxh> prompt):\n",
            "  cd ", px4_dir, " && PX4_GZ_WORLD=", world,
            " PX4_UXRCE_DDS_PORT=", dds_port,
            " make px4_sitl gz_", model, "\n",
            "Terminal C (optional gz GUI):\n",
            "  gz sim -g\n",
            "Terminal D (this launch -- already running):\n",
            "  ros2 launch gz_bridge_ros2 depth_bridge_launch.py\n",
            "=== PX4 auto-detects the running gz server and attaches. ===\n",
        ],
        condition=IfCondition(PythonExpression(["'", start_px4, "' == 'false'"])),
    )

    # ---- (3) Gazebo /clock -> ROS 2 /clock (enables use_sim_time) ---------
    # Without this, the depth pointcloud stamps (gz sim time, e.g. ~318 s) drift
    # far from TF stamps (wall clock), causing RViz2 MessageFilter to drop every
    # message with "timestamp earlier than all data in the transform cache".
    # Gz publishes a global /clock topic (gz.msgs.Clock); bridging it to ROS 2
    # /clock lets all nodes with use_sim_time:=true share the gz sim clock.
    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # ---- (4) drone_kinematic node (drone + camera TF) ---------------------
    # Built from this package (src/drone_kinematic.cpp). Subscribes to
    # /fmu/out/vehicle_odometry (PX4 -> ROS 2 via microXRCE-DDS) and broadcasts:
    #   map      -> odom       (static, identity)
    #   odom     -> base_link  (dynamic, ~200 Hz, NED/FRD -> ENU/FLU converted)
    #   base_link -> camera_link (static, SDF-exact mount from x500_depth.sdf)
    # use_sim_time:=true so its TF stamps use the gz clock, matching the
    # pointcloud stamps (which are gz sim time from ros_gz_bridge).
    drone_kinematic_node = Node(
        package="gz_bridge_ros2",
        executable="drone_kinematic",
        name="drone_kinematic",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ---- (5) Depth-camera Gazebo -> ROS 2 bridge -------------------------
    # ros_gz_bridge parameter_bridge syntax:
    #   <ros_topic>@<ros_type>[<gz_type>
    # OakD-Lite SDF sets <topic>depth_camera</topic> so Gz publishes explicit
    # short topics /depth_camera (gz.msgs.Image) and /depth_camera/points
    # (gz.msgs.PointCloudPacked). The bridged ROS 2 messages have frame_id
    # "camera_link" (polos, no instance prefix — confirmed at runtime) which
    # matches the base_link -> camera_link TF that drone_kinematic publishes,
    # so no extra frame alias is needed. Stamps are gz sim time (passed through).
    depth_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="depth_camera_bridge",
        arguments=[
            "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    # ---- (6) Optional plane-segmentation front-end ------------------------
    # `perception` picks exactly one of GNG (cpu/gpu) or RANSAC; see the module
    # docstring for the topic each publishes. Only the chosen mode's nodes run
    # — the rest are built with a condition that evaluates false.
    perception_group = perception_nodes(perception, perception_input, voxel_leaf)

    # A typo in `perception` would otherwise start nothing at all, silently,
    # and look exactly like a healthy launch until the mission times out
    # waiting for a landing point. Say so at startup instead.
    unknown_perception_warning = LogInfo(
        msg=[
            "\n*** perception:='", perception, "' is not a known mode. ***\n",
            "    Nothing was started for plane segmentation.\n",
            "    Valid values: " + " ".join(PERCEPTION_MODES) + "\n",
        ],
        condition=IfCondition(
            PythonExpression(["'", perception, "' not in ", repr(PERCEPTION_MODES)])
        ),
    )

    # `landing_circle` is NOT started here — waypoint_node starts it on arrival
    # (see offboard_mission/perception_supervisor.py for why: GNG alone was
    # measured at ~701% CPU, so it must not run through arming and transit).
    # waypoint_node's default `perception_commands` hard-codes the GNG topics,
    # so the RANSAC mode needs that parameter overridden. Print the line rather
    # than leave the user with a /plane nobody reads.
    ransac_mission_info = LogInfo(
        msg=[
            "\n=== perception:=ransac — mission command ===\n",
            "RANSAC is already running from this launch, so the mission only\n",
            "needs to start landing_circle, pointed at the RANSAC topics:\n\n",
            "  ros2 run offboard_mission waypoint_node --ros-args \\\n",
            "      -p perception_commands:=\"['landing_circle: ros2 run "
            "segmentation_node landing_circle --ros-args "
            "-p input_topic:=/plane -p outlier_topic:=/outlier']\"\n\n",
            "Without it landing_circle subscribes to /plane_cpu (GNG) and\n",
            "starts a second GNG alongside RANSAC.\n",
        ],
        condition=mode_is(perception, "ransac"),
    )
    gng_mission_info = LogInfo(
        msg=[
            "\n=== perception:=gng_cpu — mission command ===\n",
            "  ros2 run offboard_mission waypoint_node\n",
            "waypoint_node's default perception_commands already matches this\n",
            "mode; its duplicate guard sees node 'dbl_gng_cpu' running and\n",
            "starts only landing_circle.\n",
        ],
        condition=mode_is(perception, "gng_cpu"),
    )

    # ---- (7) path_trail node (drone trajectory -> nav_msgs/Path) -----------
    # Listens to TF map -> base_link (published by drone_kinematic) and
    # accumulates the drone's ENU position at 10 Hz into a nav_msgs/Path on
    # /drone_path, which RViz2 renders as a green line trail. use_sim_time:=true
    # so the TF buffer is on the gz sim clock (otherwise lookups return empty).
    # Default on so the trail appears automatically; disable with path_trail:=false.
    path_trail_node = Node(
        package="path_trail",
        executable="path_trail_node",
        name="path_trail",
        output="screen",
        condition=IfCondition(path_trail),
        parameters=[{"use_sim_time": True}],
    )

    # ---- (8) Optional RViz2 -----------------------------------------------
    # use_sim_time:=true so RViz2 queries TF at gz sim time (matches the
    # pointcloud/Image message stamps), otherwise the MessageFilter drops
    # every message because of the clock-domain mismatch.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_depth",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(rviz),
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            declare_world,
            declare_model,
            declare_instance,
            declare_dds_port,
            declare_agent_bin,
            declare_headless,
            declare_rviz,
            declare_px4_dir,
            declare_start_px4,
            declare_perception,
            declare_perception_input,
            declare_voxel_leaf,
            declare_path_trail,
            LogInfo(msg="=== gz_bridge_ros2: x500_depth launch "
                        "(agent + clock + TF + depth bridge "
                        "+ optional perception/PX4) ==="),
            manual_px4_info,
            unknown_perception_warning,
            ransac_mission_info,
            gng_mission_info,
            agent_proc,
            px4_proc,
            clock_bridge_node,
            drone_kinematic_node,
            depth_bridge_node,
            *perception_group.values(),
            path_trail_node,
            rviz_node,
        ]
    )
