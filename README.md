# 🚁 Autonomous Landing Drone Algorithm

Sistem drone otonom berbasis ROS2 + PX4 yang mampu mendeteksi zona pendaratan aman menggunakan depth camera, algoritma GNG (Growing Neural Gas), segmentasi bidang RANSAC, dan deteksi lingkaran landing secara otomatis.

---

## 📋 Daftar Isi

- [Prasyarat](#prasyarat)
- [Arsitektur Sistem](#arsitektur-sistem)
- [Cara Menjalankan](#cara-menjalankan)
- [Penjelasan Tiap Node](#penjelasan-tiap-node)
- [Struktur Direktori](#struktur-direktori)
- [Troubleshooting](#troubleshooting)

---

## Prasyarat

Pastikan semua dependensi berikut sudah terinstal sebelum menjalankan proyek ini:

| Komponen | Versi yang Direkomendasikan |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| PX4-Autopilot | main / v1.14+ |
| Gazebo | Garden (gz-garden) |
| MicroXRCE-DDS Agent | latest |
| Python | 3.10+ |
| NVIDIA Driver | 520+ (untuk GPU rendering) |

---

## Arsitektur Sistem

```
┌─────────────────────────────────────────────┐
│           Lingkungan Simulasi               │
│  Gazebo (rubicon.sdf) + PX4 SITL x500_depth │
└────────────────────┬────────────────────────┘
                     │
         ┌───────────┴───────────┐
         │    Jembatan Komunikasi │
         │  ros_gz_bridge         │
         │  MicroXRCEAgent UDP    │
         └───────────┬───────────┘
                     │
         ┌───────────┴───────────┐
         │   Kendali Penerbangan  │
         │   drone_kinematic      │
         └───────────┬───────────┘
                     │
    ┌────────────────┴──────────────────┐
    │         Pipeline Persepsi AI       │
    │  GNG Node → RANSAC → landing_circle│
    └────────────────┬──────────────────┘
                     │
         ┌───────────┴───────────┐
         │   Autonomous Landing   │
         └───────────────────────┘
```

---

## Cara Menjalankan

Jalankan setiap langkah di **terminal terpisah** secara berurutan.

### Terminal 1 — Jalankan Simulasi Gazebo (Server)

```bash
cd PX4-Autopilot

export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds

__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  gz sim -r -s ~/PX4-Autopilot/Tools/simulation/gz/worlds/rubicon.sdf
```

> Tunggu hingga simulasi server siap (biasanya 3–5 detik) sebelum lanjut ke langkah berikutnya.

---

### Terminal 2 — Jalankan PX4 SITL

```bash
cd PX4-Autopilot

sleep 5 && \
  PX4_GZ_WORLD=rubicon \
  PX4_GZ_MODEL_POSE="0,3,8,0,0,0" \
  PX4_GZ_STANDALONE=1 \
  make px4_sitl gz_x500_depth
```

> Parameter `PX4_GZ_MODEL_POSE="0,3,8,0,0,0"` berarti drone dimulai di posisi x=0, y=3, z=8 (meter), tanpa rotasi.

---

### Terminal 3 — Jalankan Gazebo GUI (Visualisasi)

```bash
export MESA_GL_VERSION_OVERRIDE=4.5
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only

gz sim -g
```

> Terminal ini khusus untuk menampilkan visualisasi 3D simulasi menggunakan GPU NVIDIA.

---

### Terminal 4 — Jalankan ROS-Gazebo Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /world/rubicon/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /world/rubicon/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

> Bridge ini menghubungkan topik sensor Gazebo (kamera, IMU, point cloud) ke topik ROS2.

---

### Terminal 5 — Jalankan MicroXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

> Agent ini menjembatani komunikasi antara PX4 internal (uORB) dengan ROS2 melalui UDP port 8888.

---

### Terminal 6 — Setup & Jalankan Workspace ROS2

```bash
cd ~/Project/drone*/

source install/setup.bash
```

Setelah itu jalankan node-node berikut (masing-masing di terminal baru atau gunakan `&` untuk background):

#### Node Kendali Drone

```bash
ros2 run offboard_control drone_kinematic
```

#### Node GNG (Growing Neural Gas — Pemetaan 3D)

```bash
ros2 run gng_node dbl_gng_node
```

#### Node Segmentasi Bidang (RANSAC)

```bash
ros2 run plane_segmentation_ransac plane_segmentation_ransac
```

#### Node Deteksi Zona Landing

```bash
ros2 run landing_circle landing_circle
```

---

## Penjelasan Tiap Node

### `drone_kinematic`
Node offboard control yang mengendalikan pergerakan drone secara kinematik. Mengirimkan setpoint posisi/kecepatan ke PX4 melalui protokol MAVLink via MicroXRCE-DDS.

### `dbl_gng_node`
Implementasi algoritma **Double Growing Neural Gas (GNG)**. Memproses data point cloud dari depth camera (sensor IMX214) untuk membangun representasi topologis permukaan 3D di lingkungan sekitar drone.

### `plane_segmentation_ransac`
Melakukan segmentasi bidang datar dari data point cloud menggunakan algoritma **RANSAC (Random Sample Consensus)**. Mengidentifikasi permukaan horizontal yang berpotensi menjadi zona pendaratan.

### `landing_circle`
Menganalisis bidang datar hasil segmentasi dan mendeteksi area lingkaran yang cukup besar dan aman untuk pendaratan. Mengirimkan koordinat target pendaratan ke node kendali drone.

---

## Struktur Direktori

```
Project/drone*/
├── src/
│   ├── offboard_control/            # Node kendali penerbangan
│   ├── gng_node/                    # Algoritma Growing Neural Gas
│   ├── segmentation_node/           
|   ├──── plane_segmentation_ransac  # Segmentasi bidang RANSAC
│   └──── landing_circle/            # Deteksi zona landing
├── install/                         # Hasil build ROS2
├── build/
└── log/
```

---

## Troubleshooting

### Gazebo tidak terbuka / crash saat launch
Pastikan variabel lingkungan GPU sudah di-export dengan benar:
```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

### PX4 tidak terkoneksi ke Gazebo
Pastikan Gazebo server (Terminal 1) sudah berjalan sepenuhnya sebelum menjalankan SITL. Tambahkan delay jika perlu:
```bash
sleep 10 && make px4_sitl gz_x500_depth
```

### ROS2 tidak menerima topik dari Gazebo
Cek apakah `ros_gz_bridge` berjalan dengan:
```bash
ros2 topic list
```
Topik seperti `/camera_info` dan `/depth_camera/points` harus muncul.

### MicroXRCEAgent gagal connect
Pastikan PX4 SITL sudah berjalan sebelum menjalankan agent, dan tidak ada proses lain yang menggunakan port 8888:
```bash
sudo lsof -i :8888
```

### Node ROS2 tidak ditemukan
Pastikan workspace sudah di-source:
```bash
source ~/Project/drone*/install/setup.bash
```

---

## Lisensi

Proyek ini dikembangkan untuk keperluan riset robotika otonom. Lihat file `LICENSE` untuk detail lebih lanjut.