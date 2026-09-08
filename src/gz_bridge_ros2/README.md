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
  cd ~/PX4-Autopilot && make px4_sitl
  ```

## Build

This package lives in the `safety-landing-point-detection-for-drone` ROS 2
workspace (`src/gz_bridge_ros2/`). The `drone_kinematic` node compiles C++ and
depends on `px4_msgs` (also in this workspace), so build both the first time:

```sh
cd ~/ros2_ws
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
    headless:=true rviz:=true px4_dir:=~/PX4-Autopilot
```

> `px4_dir` defaults to `~/PX4-Autopilot`. If your PX4 checkout is
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

## Cara masuk ke commander (pxh> prompt langsung)

Default launch (`start_px4:=false`) **tidak** menjalankan PX4 maupun Gazebo —
Anda menjalankannya manual di terminal terpisah, sehingga mendapat prompt
interaktif `pxh>` dari PX4 (bisa `commander takeoff`, `param set`, dll).
Saat launch start, ia mencetak 3 command copy-paste ready di log.

```sh
# Terminal 1: ROS 2 stack (agent + clock/depth bridge + TF + perception + RViz)
ros2 launch gz_bridge_ros2 depth_bridge_launch.py rviz:=true perception:=gng_cpu
# ...or the RANSAC front-end instead:
# ros2 launch gz_bridge_ros2 depth_bridge_launch.py rviz:=true perception:=ransac
# -> launch prints the exact Terminal A/B/C commands below; copy-paste them.

# Terminal A: gz server (sim physics, no GUI)
gz sim -r -s ~/PX4-Autopilot/Tools/simulation/gz/worlds/rubicon.sdf

# Terminal B: PX4 SITL — dapat prompt pxh> interaktif
cd ~/PX4-Autopilot && PX4_GZ_WORLD=rubicon PX4_UXRCE_DDS_PORT=8888 \
    make px4_sitl gz_x500_depth
# PX4 auto-detects gz server (via /world/rubicon/clock) -> spawn model +
# start gz_bridge. agent + ros_gz_bridge dari launch auto-connect.

# Terminal C: gz GUI (opsional)
gz sim -g
```

`PX4_GZ_WORLD` harus match `world` launch (default `rubicon`).
`PX4_UXRCE_DDS_PORT` harus match `dds_port` launch (default `8888`).

### Cheat-sheet minimal (prompt `pxh>`)

| Command             | Efek                              |
|---------------------|-----------------------------------|
| `commander arm`     | Arm motor                         |
| `commander disarm`  | Disarm                            |
| `commander takeoff` | Takeoff (altitude default)        |
| `commander land`    | Landing di tempat                 |
| `mode POSCTL`       | Switch ke mode position control   |
| `mode AUTO`         | Switch ke mode auto mission      |
| `mode OFFBOARD`     | Switch ke mode offboard          |
| `param show <NAME>` | Lihat nilai param                 |
| `param set <NAME> <VAL>` | Set param (efektif setelah reboot) |

## Alternatif: MAVLink CLI (MAVProxy)

Kalau Anda memakai `start_px4:=true` (PX4 dijalankan launch, prompt `pxh>`
tidak menerima stdin), atau ingin kontrol dari terminal lain tanpa
menghentikan PX4, pakai wrapper MAVProxy yang sudah disediakan:

```sh
# Terminal 1: stack dengan PX4 bundled
ros2 launch gz_bridge_ros2 depth_bridge_launch.py start_px4:=true rviz:=true

# Terminal 2 (setelah PX4 boot, ~10-20 dtk): masuk ke MAVProxy
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

| Command MAVProxy    | Efek                              |
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

| Argument        | Default                       | Description |
|-----------------|-------------------------------|-------------|
| `world`         | `rubicon`                     | Gazebo world name (Tools/simulation/gz/worlds/<world>.sdf) |
| `model`         | `x500_depth`                  | PX4 Gz model name (without `gz_` prefix) |
| `instance`      | `x500_depth_0`               | Spawned model instance name (for TF frames) |
| `dds_port`      | `8888`                        | microXRCE-DDS agent UDP port (matches UXRCE_DDS_PRT) |
| `agent_bin`     | `MicroXRCEAgent`              | Agent binary (`MicroXRCEAgent` or `micro-xrce-dds-agent`) |
| `headless`      | `false`                       | Run Gazebo headless (`HEADLESS=1`, bundled mode only) |
| `rviz`          | `false`                       | Start RViz2 with the x500_depth preset |
| `px4_dir`       | `$PX4_SOURCE_DIR` or repo root | Where `make px4_sitl` is run |
| `start_px4`     | `false`                       | `true` = launch runs `make px4_sitl` bundled (gz auto-launch, non-interactive). `false` (default) = run gz sim + PX4 manually for interactive `pxh>` prompt; launch only starts agent + bridges + TF + perception + RViz |
| `perception`    | `none`                        | Plane-segmentation front-end: `gng_cpu` \| `gng_gpu` \| `ransac` \| `none` (off). See below |
| `perception_input` | `/depth_camera/points`     | Input PointCloud2 topic for the chosen front-end (override to `/zed/zed_node/point_cloud/cloud_registered` for real ZED) |
| `voxel_leaf`    | `0.15`                        | Voxel cell size in metres, passed to **both** front-ends — see below |
| `path_trail`    | `true`                        | Start `path_trail_node` (drone trajectory as `nav_msgs/Path` on `/drone_path`) |

### Choosing the plane-segmentation front-end

One argument, one choice — two front-ends can never run at once and fight for
the CPU.

| `perception` | nodes started | publishes |
|---|---|---|
| `none` (default) | — | — |
| `gng_cpu` | `dbl_gng_cpu` (numpy, no torch) | `/plane_cpu`, `/outlier_cpu` |
| `gng_gpu` | `dbl_gng` (torch, cuda/cpu fallback) | `/plane_cpu`, `/outlier_cpu` |
| `ransac` | `plane_segmentation_ransac` | `/plane`, `/outlier` |

### Feeding both front-ends the same cloud

The point of one launch offering both is to compare them, and that comparison
is only about the algorithms if they receive the same input. Three things used
to differ; all three are settled here.

| stage | GNG (cpu) | RANSAC (as shipped) |
|---|---|---|
| topic | `/depth_camera/points` | `/depth_camera/points` |
| drop NaN | yes | yes |
| **cylinder crop** | none | `cylinder_crop` in front *(node since deleted)* |
| **spatial filter** | none | PassThrough `z ∈ [-5, 5]` |
| **voxel** | `voxel_leaf` = 0.15 | `leaf_size` = 0.07 |

**`voxel_leaf` reaches both.** GNG calls the parameter `voxel_leaf`, RANSAC
calls it `leaf_size`; this launch maps one argument onto both names so they
can never drift apart. Before that, the two ran at 19200→6445 (RANSAC) against
19200→~2000 (GNG), and their computation times were not comparable.

**`cylinder_crop` is gone from this launch — and, since this was written, the
node has been deleted from `segmentation_node` entirely.** It kept points within a radius
of the x-y origin, but in `camera_link` **x is the depth axis**. Measured in
flight at an 11 m scan altitude:

```
/depth_camera/points  frame=camera_link  n=19200
    x:   0.21 ..  14.81   median  12.51     <- depth
    y:  -8.22 ..   9.92   median   0.04
    z:  -8.15 ..   7.40   median   0.03
/circle_cloud         frame=camera_link  n=4
```

The ground plane lies in y-z — which is why `landing_circle` is configured
`plane_axes=yz`. A 5 m cylinder on x-y therefore throws the ground away:
19200 points in, 4 out, and RANSAC then publishes nothing at all, so
`safety_point` never appears and the mission times out holding at the
waypoint. It only appears to work with the camera close to the ground.
`src/segmentation_node/launch/ransac_pipeline.launch.py` still has that chain
if it is ever wanted back.

**The RANSAC PassThrough is disabled** (`z_min`/`z_max` ±1000). `z` is a
lateral axis in `camera_link`, spanning −8.15..7.40 m at that altitude, so
`[-5, 5]` cuts the outer band of a field of view GNG keeps. That is an input
filter, not part of the algorithm.

Not touched: RANSAC's `setAxis(1,0,0)` + `setEpsAngle(5°)`. That is a model
constraint — part of the algorithm being compared.

`gng_gpu` has no voxel stage at all (`dbl_gng_node` declares no `voxel_leaf`),
so `voxel_leaf` is not passed to it and it is not comparable with `ransac` on
input load. The valid comparison is **`gng_cpu` against `ransac`**.

**The landing-point search is not started here.** `landing_circle` is started
by `offboard_mission/waypoint_node` when the drone arrives, and its default
`perception_commands` hard-codes `input_topic:=/plane_cpu` — the GNG topic. So
`perception:=ransac` on its own publishes a `/plane` nothing reads. The launch
prints the matching `waypoint_node` command for whichever mode you chose; copy
it from the log:

```sh
ros2 run offboard_mission waypoint_node --ros-args \
    -p perception_commands:="['landing_circle: ros2 run segmentation_node \
       landing_circle --ros-args -p input_topic:=/plane -p outlier_topic:=/outlier']"
```

## Caveats

- The first `make px4_sitl gz_x500_depth` invocations build PX4 and can be slow;
  subsequent runs use the build cache (`build/px4_sitl_default/`).
- **PX4's interactive console does not accept stdin while it is a child of
  `ros2 launch`** (prompt `pxh>` tampil tapi tidak bisa diketik). For commander
  interaction use the DEFAULT mode `start_px4:=false` and run PX4 manually in
  another terminal, or use the MAVProxy wrapper (`ros2 run gz_bridge_ros2
  mavlink_cli`) with `start_px4:=true`.
- `Tools/simulation/gz` is a git submodule of the PX4 repo, so this package
  intentionally lives **outside** it, in this workspace's `src/gz_bridge_ros2/`.