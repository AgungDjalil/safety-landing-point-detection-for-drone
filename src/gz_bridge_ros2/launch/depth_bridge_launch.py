"""
Launch file for PX4 SITL (gz_x500_depth) + microXRCE-DDS agent + Gazebo->ROS 2
depth-camera bridge.

What it starts
---------------
1. microXRCE-DDS agent            (MicroXRCEAgent udp4 -p 8888)
2. PX4 SITL + Gazebo              (PX4_GZ_WORLD=<world> make px4_sitl gz_<model>)
3. gz clock bridge                (/clock gz.msgs.Clock -> ROS 2 /clock; enables
                                  use_sim_time across nodes so pointcloud &
                                  deep TF share the gz simulation clock)
4. drone_kinematic node           (map -> odom -> base_link [dynamic, ~200 Hz from
                                  /fmu/out/vehicle_odometry] + base_link -> camera_link
                                  [static, SDF-exact mount]; use_sim_time:=true)
5. ros_gz_bridge parameter_bridge (/depth_camera, /depth_camera/points)
6. (optional) rviz2               (-d <config>/x500_depth.rviz; use_sim_time:=true)

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
    cd /home/alphaone/Documents/safety-landing-point-detection-for-drone
    colcon build --packages-select gz_bridge_ros2
    source install/setup.bash
    ros2 launch gz_bridge_ros2 depth_bridge_launch.py

Overrides
---------
    ros2 launch gz_bridge_ros2 depth_bridge_launch.py \
        world:=baylands model:=x500_depth instance:=x500_depth_0 \
        dds_port:=8888 agent_bin:=MicroXRCEAgent \
        headless:=false rviz:=true px4_dir:=/home/alphaone/PX4-Autopilot

Notes
-----
- First run builds PX4 (`make px4_sitl gz_x500_depth`); subsequent runs are cached.
- The agent must be Micro-XRCE-DDS-Agent v2.x (not v3.x) to match PX4's client.
- PX4's interactive console does not accept stdin under launch; for commander
  interaction run PX4 in a separate terminal instead.
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


def _looks_like_px4_root(path):
    """Heuristic: a directory containing both a top-level Makefile and a
    boards/ tree is the PX4-Autopilot repo root."""
    return os.path.isfile(os.path.join(path, "Makefile")) and os.path.isdir(
        os.path.join(path, "boards")
    )


def _px4_source_dir():
    """Resolve the PX4-Autopilot repo root.

    This package is independent of the PX4 tree, so the root must come from
    an explicit source. Resolution order:
      1. $PX4_SOURCE_DIR env var (explicit user override, preferred).
      2. The sibling PX4-Autopilot checkout at /home/alphaone/PX4-Autopilot
         (the conventional location on this machine) if it looks like a PX4
         root.
      3. Walk up from this file's real location looking for the PX4 root
         marker (Makefile + boards/); covers the case where this workspace
         happens to live inside a PX4 checkout.
      4. The conventional default /home/alphaone/PX4-Autopilot (lets the user
         see a meaningful error from `make` rather than a Python import-time
         failure).
    """
    env_dir = os.environ.get("PX4_SOURCE_DIR")
    if env_dir and os.path.isdir(env_dir) and _looks_like_px4_root(env_dir):
        return env_dir

    default = "/home/alphaone/PX4-Autopilot"
    if _looks_like_px4_root(default):
        return default

    here = os.path.dirname(os.path.realpath(__file__))
    parent = here
    while parent and parent != os.path.dirname(parent):
        if _looks_like_px4_root(parent):
            return parent
        parent = os.path.dirname(parent)

    return default


def generate_launch_description():
    pkg_share = get_package_share_directory("gz_bridge_ros2")
    rviz_config = PathJoinSubstitution([pkg_share, "config", "x500_depth.rviz"])

    # ---- Declare all user-tunable arguments --------------------------------
    declare_world = DeclareLaunchArgument(
        "world",
        default_value="baylands",
        description="Gazebo world name (e.g. baylands, default, windy).",
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

    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    instance = LaunchConfiguration("instance")
    dds_port = LaunchConfiguration("dds_port")
    agent_bin = LaunchConfiguration("agent_bin")
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    px4_dir = LaunchConfiguration("px4_dir")

    # ---- (1) microXRCE-DDS agent ------------------------------------------
    # Must start before PX4; PX4's uxrce_dds_client retries until it connects.
    agent_proc = ExecuteProcess(
        cmd=[agent_bin, "udp4", "-p", dds_port],
        output="screen",
    )

    # ---- (2) PX4 SITL + Gazebo --------------------------------------------
    # `PX4_GZ_WORLD=<world> make px4_sitl gz_<model>` builds (cached) and runs
    # px4, which auto-launches Gazebo (px4-rc.gzsim) and the uxrce_dds_client
    # on UDP localhost:<dds_port> (ROMFS/.../rcS:293-324).
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

    # ---- (6) Optional RViz2 -----------------------------------------------
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
            LogInfo(msg="=== gz_bridge_ros2: x500_depth launch (agent + PX4 SITL + clock + TF + depth bridge) ==="),
            agent_proc,
            px4_proc,
            clock_bridge_node,
            drone_kinematic_node,
            depth_bridge_node,
            rviz_node,
        ]
    )