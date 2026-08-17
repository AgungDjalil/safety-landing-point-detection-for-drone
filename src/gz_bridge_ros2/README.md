# gz_bridge_ros2

ROS 2 (Humble) package for the PX4 Gazebo `x500_depth` depth camera. It builds the
`drone_kinematic` TF node and provides a single `ros2 launch` that starts the
whole stack: microXRCE-DDS agent + PX4 SITL + Gazebo + drone/camera TF tree +
depth-camera Gazebo->ROS 2 bridge.

## What it starts

| # | Process | Command |
|---|---------|---------|
| 1 | microXRCE-DDS agent | `MicroXRCEAgent udp4 -p 8888` |
| 2 | PX4 SITL + Gazebo   | `PX4_GZ_WORLD=baylands make px4_sitl gz_x500_depth` |
| 3 | `drone_kinematic` node | subscribes `/fmu/out/vehicle_odometry`, broadcasts `map->odom->base_link` (dynamic) + `base_link->camera_link` (static) |
| 4 | Depth-camera bridge | `ros_gz_bridge parameter_bridge` `/depth_camera` + `/depth_camera/points` |
| 5 | Frame alias TF | `static_transform_publisher camera_link -> <instance>/camera_link` (identity) |
| 6 | RViz2 (optional)    | `rviz2 -d <pkg>/config/x500_depth.rviz` |

## TF tree (all published from one launch)

```
map ── odom ── base_link ── camera_link ── <instance>/camera_link
 │      │         │            │                 │
 static  ~200Hz   ~200Hz       static            static identity
 (node)  (node TransformBroadcaster)  (node StaticBr.)  (launch)
```

- `map -> odom`: static identity (`drone_kinematic`).
- `odom -> base_link`: dynamic ~200 Hz from `/fmu/out/vehicle_odometry` with
  NED/FRD -> ENU/FLU conversion (`drone_kinematic`).
- `base_link -> camera_link`: static, **SDF-exact** mount from
  `Tools/simulation/gz/models/x500_depth/model.sdf` (`t=(0.22, 0.03, 0.242)`,
  `rpy=(0, 1.5708, 0)`) (`drone_kinematic`).
- `camera_link -> <instance>/camera_link`: static identity alias so the bridged
  pointcloud (whose gz `frame_id` is `<instance>/camera_link`) resolves in tf2.

PX4 SITL auto-starts the `uxrce_dds_client` on UDP localhost:8888
(`ROMFS/px4fmu_common/init.d-posix/rcS`) and auto-launches Gazebo via
`px4-rc.gzsim`.

## Depth-camera topics

The depth camera is the `StereoOV7251` sensor in
`Tools/simulation/gz/models/OakD-Lite/model.sdf` (160x120, `R_FLOAT32`, 15 Hz,
0.2-20 m). It uses an explicit `<topic>depth_camera</topic>`, so Gazebo publishes
the **short** topics:

- `/depth_camera`        -> `gz.msgs.Image`            (float32 depth) -> ROS 2 `sensor_msgs/Image`
- `/depth_camera/points` -> `gz.msgs.PointCloudPacked`                  -> ROS 2 `sensor_msgs/PointCloud2`

## Prerequisites

- ROS 2 Humble with `ros_gz_bridge` (Harmonic variant) installed.
  Check with: `ros2 pkg executables ros_gz_bridge` (should list `parameter_bridge`).
- Gazebo Harmonic (`gz sim --version` >= 8.0).
- microXRCE-DDS Agent **v2.x** (e.g. v2.4.3). PX4's client is NOT compatible
  with v3.x. Build from source:
  ```sh
  git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  cd Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make && sudo make install && sudo ldconfig
  ```
  Then the binary `MicroXRCEAgent` should be on your `PATH`. If you installed it
  via snap instead, pass `agent_bin:=micro-xrce-dds-agent`.
- PX4-Autopilot built at least once for SITL:
  ```sh
  cd /home/alphaone/PX4-Autopilot && make px4_sitl
  ```

## Build

This package lives in the `safety-landing-point-detection-for-drone` ROS 2
workspace (`src/gz_bridge_ros2/`). The `drone_kinematic` node compiles C++ and
depends on `px4_msgs` (also in this workspace), so build both the first time:

```sh
cd /home/alphaone/Documents/safety-landing-point-detection-for-drone
colcon build --packages-select px4_msgs gz_bridge_ros2
source install/setup.bash
# later (px4_msgs already installed):
# colcon build --packages-select gz_bridge_ros2
```

## Run

Default world `baylands`, model `x500_depth`, port `8888`:

```sh
ros2 launch gz_bridge_ros2 depth_bridge_launch.py
```

With overrides (e.g. headless Gazebo + RViz + snap agent binary):

```sh
ros2 launch gz_bridge_ros2 depth_bridge_launch.py \
    world:=baylands model:=x500_depth instance:=x500_depth_0 \
    dds_port:=8888 agent_bin:=micro-xrce-dds-agent \
    headless:=true rviz:=true px4_dir:=/home/alphaone/PX4-Autopilot
```

> `px4_dir` defaults to `/home/alphaone/PX4-Autopilot`. If your PX4 checkout is
> elsewhere, set `PX4_SOURCE_DIR` or pass `px4_dir:=<path>`.

## Verify

In another terminal (with `source install/setup.bash`):

```sh
ros2 topic list | grep -E "depth|tf"
ros2 topic echo /depth_camera --field header     # sensor_msgs/Image header
ros2 topic hz /depth_camera/points               # ~15 Hz

# TF tree
ros2 topic hz /tf                                 # ~200 Hz (odom -> base_link)
ros2 topic echo /tf_static --once                 # map->odom, base_link->camera_link,
                                                  # camera_link->x500_depth_0/camera_link
ros2 run tf2_tools view_frames                    # generates a frames.pdf
```

## Cara masuk ke commander (MAVLink CLI)

PX4 SITL yang dijalankan via `ros2 launch` tidak menerima stdin (prompt `pxh>`
tampil tapi tidak bisa diketik). Untuk mengirim commander command (arm/takeoff/
land/mode/params) dari shell, pakai wrapper MAVProxy yang sudah disediakan:

```sh
# Terminal 1: jalankan stack (agent + PX4 SITL + Gazebo + TF + depth bridge)
ros2 launch gz_bridge_ros2 depth_bridge_launch.py

# Terminal 2 (setelah PX4 boot, ~10-20 dtk): masuk ke commander
source install/setup.bash
ros2 run gz_bridge_ros2 mavlink_cli
# muncul prompt MAVProxy> ; ketik command, exit/ctrl-D untuk keluar.
```

Default konek ke `udp:127.0.0.1:14550` (port broadcast GCS PX4 SITL,
lihat `ROMFS/.../px4-rc.mavlink`). Override: `MAVLINK_CLI_PORT=14540 ros2 run
gz_bridge_ros2 mavlink_cli`, atau `ros2 run gz_bridge_ros2 mavlink_cli --
--master=udp:127.0.0.1:<port>`. Mau `--map`/`--console`: `ros2 run
gz_bridge_ros2 mavlink_cli -- --map`.

Prasyarat: `mavproxy.py` di PATH (sudah terpasang di mesin ini di
`~/.local/bin/`; kalau hilang: `python3 -m pip install --user MAVProxy
pymavlink`).

### Cheat-sheet minimal

| Command             | Efek                              |
|---------------------|-----------------------------------|
| `arm throttle`      | Arm motor                         |
| `disarm`            | Disarm                            |
| `mode POSCTL`       | Switch ke mode position control   |
| `mode AUTO`         | Switch ke mode auto mission      |
| `mode OFFBOARD`     | Switch ke mode offboard          |
| `takeoff 5`         | Takeoff ke 5 m                    |
| `land`              | Landing di tempat                 |
| `param show <NAME>` | Lihat nilai param                 |
| `param set <NAME> <VAL>` | Set param (efektif setelah reboot) |
| `exit` / `Ctrl-D`   | Keluar dari MAVProxy              |

Argumen pertama boleh diberi prefix `--` jika membawa opsi MAVProxy:
`ros2 run gz_bridge_ros2 mavlink_cli -- --map`.

## Arguments

| Argument    | Default                       | Description |
|-------------|-------------------------------|-------------|
| `world`     | `baylands`                    | Gazebo world name (Tools/simulation/gz/worlds/<world>.sdf) |
| `model`     | `x500_depth`                  | PX4 Gz model name (without `gz_` prefix) |
| `instance`  | `x500_depth_0`               | Spawned model instance name (for TF frames) |
| `dds_port`  | `8888`                        | microXRCE-DDS agent UDP port (matches UXRCE_DDS_PRT) |
| `agent_bin` | `MicroXRCEAgent`              | Agent binary (`MicroXRCEAgent` or `micro-xrce-dds-agent`) |
| `headless`  | `false`                       | Run Gazebo headless (`HEADLESS=1`) |
| `rviz`      | `false`                       | Start RViz2 with the x500_depth preset |
| `px4_dir`   | `$PX4_SOURCE_DIR` or repo root | Where `make px4_sitl` is run |

## Caveats

- The first `make px4_sitl gz_x500_depth` invocations build PX4 and can be slow;
  subsequent runs use the build cache (`build/px4_sitl_default/`).
- PX4's interactive console does not accept stdin while it is a child of
  `ros2 launch`. For commander interaction (arming, mode changes) either use
  MAVLink from another tool or run PX4 SITL in another terminal and limit this
  launch to the agent + depth bridge.
- `Tools/simulation/gz` is a git submodule of the PX4 repo, so this package
  intentionally lives **outside** it, in this workspace's `src/gz_bridge_ros2/`.