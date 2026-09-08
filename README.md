# 🚁 Autonomous Landing Drone Algorithm

Sistem drone otonom berbasis ROS2 + PX4 yang mampu mendeteksi zona pendaratan aman menggunakan depth camera, algoritma GNG (Growing Neural Gas), segmentasi bidang RANSAC, dan deteksi lingkaran landing secara otomatis.

---

## 📋 Daftar Isi

- [Prasyarat](#prasyarat)
  - [Kalau perangkat Anda tidak punya GPU](#kalau-perangkat-anda-tidak-punya-gpu)
- [Modifikasi PX4-Autopilot yang Diperlukan](#modifikasi-px4-autopilot-yang-diperlukan)
- [Build dan Pengujian](#build-dan-pengujian)
- [Hasil penerbangan terverifikasi](#hasil-penerbangan-terverifikasi)
- [Arsitektur Sistem](#arsitektur-sistem)
- [Cara Menjalankan](#cara-menjalankan)
- [Penjelasan Tiap Node](#penjelasan-tiap-node)
- [Node jalur lama dan diagnostik](#node-jalur-lama-dan-diagnostik)
- [Struktur Direktori](#struktur-direktori)
- [Cara Mereproduksi Verifikasi](#cara-mereproduksi-verifikasi)
- [Masalah yang Diketahui](#masalah-yang-diketahui)
- [Troubleshooting](#troubleshooting)

---

## Prasyarat

Pastikan semua dependensi berikut sudah terinstal sebelum menjalankan proyek ini:

| Komponen | Versi yang diverifikasi | Catatan |
|---|---|---|
| Ubuntu | 22.04.5 LTS | ✅ diuji |
| ROS 2 | Humble | ✅ diuji |
| Python | 3.10.12 | ✅ diuji |
| Gazebo Sim | **8.15.0 (Harmonic)** | ✅ diuji — `gz sim --versions` |
| PX4-Autopilot | main / v1.14+ | **butuh modifikasi** — lihat [bagian khusus](#modifikasi-px4-autopilot-yang-diperlukan) |
| MicroXRCE-DDS Agent | v2.x (bukan v3.x) | `/usr/local/bin/MicroXRCEAgent` |
| PCL | 1.12 | dipakai `segmentation_node` |
| NumPy | 2.2.6 | dipakai `dbl_gng_cpu_node` |
| QGroundControl | AppImage terbaru | opsional, hanya untuk pemantauan |
| NVIDIA Driver | — | **tidak diperlukan** |
| PyTorch + CUDA | — | **hanya** untuk `dbl_gng_node` (jalur GPU, tidak dipakai) |

> **Jalur produksi berjalan sepenuhnya di CPU.** Mesin rujukan yang dipakai
> memverifikasi dokumen ini tidak punya GPU NVIDIA sama sekali, dan seluruh
> pipeline yang diterbangkan (`dbl_gng_cpu_node` → `landing_circle`) tetap
> berjalan. Baris NVIDIA di README versi lama menyesatkan: itu syarat untuk
> `dbl_gng_node`, bukan untuk misi.

> **Peringatan lingkungan.** `scipy` 1.8.0 dan `cv2` bawaan sistem rusak oleh
> NumPy 2.2.6 (`_ARRAY_API not found`). Keduanya **tidak** dipakai oleh node
> mana pun di workspace ini, jadi tidak memblokir — tapi jangan kaget kalau
> skrip pihak ketiga gagal mengimpornya.

### Kalau perangkat Anda tidak punya GPU

**Jawaban singkat: jalankan semuanya kecuali satu node.** Dari 12 executable di
workspace ini, **11 berjalan tanpa GPU** dan sudah diverifikasi begitu pada
mesin rujukan yang memang tidak punya GPU NVIDIA sama sekali. Hanya
`dbl_gng_node` yang tidak bisa.

| Node | Executable | Tanpa GPU? |
|---|---|---|
| `drone_kinematics` | `drone_kinematic` | ✅ bisa |
| `dbl_gng_cpu` | `dbl_gng_cpu_node` | ✅ bisa — **ini front-end persepsi yang dipakai** |
| `plane_segmentation_ransac` | `plane_segmentation_ransac` | ✅ bisa |
| `landing_circle` | `landing_circle` | ✅ bisa |
| `waypoint_mission` | `waypoint_node` | ✅ bisa |
| `keyboard_offboard_node` | `keyboard_offboard_node` | ✅ bisa |
| `path_trail` | `path_trail_node` | ✅ bisa |
| `stats_logger` | `logger_stats` | ✅ bisa |
| `cloud_size_node` | `cloud_size_node` | ✅ bisa (diagnostik saja) |
| `plane_segmentation_gng` | `plane_segmentation_gng` | ✅ bisa (jalur lama) |
| `plane_segmentation_gng` | `ddgng_node` | ✅ bisa (jalur lama) |
| `dbl_gng` | **`dbl_gng_node`** | ❌ **tidak bisa** — butuh `torch` |

**Satu-satunya yang hilang tanpa GPU adalah `dbl_gng_node`**, dan itu tidak
merugikan: node tersebut bukan bagian dari misi. `waypoint_node` menyalakan
`dbl_gng_cpu_node`, bukan versi GPU-nya. Jadi **misi pendaratan otonom lengkap
tetap utuh** di mesin tanpa GPU.

Karena `dbl_gng_node` gagal saat `import torch` — sebelum node ROS-nya sempat
terbentuk — konsekuensinya di tingkat launch:

| `perception:=` | Tanpa GPU |
|---|---|
| `gng_cpu` | ✅ **pakai ini** (baku untuk misi) |
| `ransac` | ✅ bisa, untuk penerbangan pembanding |
| `none` | ✅ bisa |
| `gng_gpu` | ❌ proses langsung mati |

Perintah yang sudah diverifikasi berjalan di mesin tanpa GPU:

```bash
# Menyalakan agen DDS + bridge + TF + persepsi CPU + jejak lintasan
ros2 launch gz_bridge_ros2 depth_bridge_launch.py \
  perception:=gng_cpu start_px4:=false rviz:=false
```

Node yang muncul: `/drone_kinematic`, `/dbl_gng_cpu`, `/gz_clock_bridge`,
`/depth_camera_bridge`, `/path_trail`.

> **Kalau Anda memang ingin memakai `dbl_gng_node`**, pasang PyTorch lebih dulu
> (`pip install torch`). Versi CPU-nya pun cukup untuk membuat node hidup, sebab
> kodenya memang jatuh kembali ke CPU sendiri — lihat
> `src/gng_node/gng_node/dbl_gng.py:54-55`:
> `device if device else ("cuda" if torch.cuda.is_available() else "cpu")`.
> Tetapi lakukan itu hanya untuk eksperimen, **bukan untuk pengukuran
> perbandingan**: `dbl_gng_node` tidak punya tahap voxel downsample sama sekali,
> sehingga beban masukannya tidak setara dengan RANSAC. Perbandingan yang sah
> tetap `gng_cpu` lawan `ransac`.

> **Grafik, bukan komputasi.** Gazebo GUI dan RViz2 tetap butuh kartu grafis
> apa pun yang bisa OpenGL, tetapi **tidak harus NVIDIA** — keduanya berjalan
> dengan rendering perangkat lunak, hanya lebih lambat. Jalankan simulasi tanpa
> jendela dengan `headless:=true` dan `rviz:=false` bila mesinnya lemah.
> Keduanya belum diuji pada mesin rujukan karena Gazebo tidak dijalankan.

---

## Modifikasi PX4-Autopilot yang Diperlukan

> **Baca ini sebelum mencoba menjalankan simulasi.** Proyek ini **tidak berjalan
> di atas PX4-Autopilot bawaan.** Simulasi bergantung pada model wahana dan world
> yang sudah dimodifikasi.

Tanpa modifikasi ini hasilnya bukan sekadar berbeda — pipeline tidak akan bekerja
sama sekali, karena pada PX4 bawaan **kamera kedalaman menghadap ke depan**,
bukan ke bawah.

Berkas-berkas yang dimodifikasi **sudah disertakan di dalam repositori ini**,
di direktori [`px4_assets/`](px4_assets/). Anda tinggal meng-clone PX4 dari
repositori resminya lalu menyalinnya ke sana.

### Pemasangan

```bash
# 1. Clone PX4 dari repositori resminya
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot

# 2. Pasang aset yang dimodifikasi dari repo ini
cd ~/ros2_ws
./px4_assets/install.sh ~/PX4-Autopilot

# 3. Bangun PX4 SITL supaya airframe yang baru terbaca
cd ~/PX4-Autopilot && make px4_sitl
```

Tambahkan `--dry-run` untuk melihat apa yang akan berubah tanpa menulis apa pun.
Skrip menolak berjalan bila direktori tujuan tidak memuat `Makefile` dan
`boards/`, dan mencadangkan setiap berkas yang ditimpa menjadi `<nama>.orig`.
Menyalin manual juga bisa, karena susunan `px4_assets/` meniru pohon PX4:

```bash
cp -r px4_assets/Tools px4_assets/ROMFS ~/PX4-Autopilot/
```

Rincian tiap berkas beserta alasannya ada di
[`px4_assets/README.md`](px4_assets/README.md). Ringkasan perubahannya:

### 1. Kamera menunduk 90° — `Tools/simulation/gz/models/x500_depth/model.sdf`

```diff
-      <pose>.12 .03 .242 0 0 0</pose>
+      <pose>0.22 0.03 0.242 0 1.5708 0</pose>
```

`1.5708` rad = 90°, membuat kamera **menghadap ke bawah**. Inilah asal seluruh
konvensi sumbu proyek ini: karena kamera menunduk, di frame `camera_link`
**x menjadi sumbu kedalaman** dan bidang tanah terbentang di **y–z** — yang
menjelaskan mengapa `landing_circle` memakai `plane_axes=yz` dan mengapa GNG
serta RANSAC memakai sumbu normal `(1, 0, 0)`.

Nilai `(0.22, 0.03, 0.242)` inilah yang di-hardcode `drone_kinematic.cpp` sebagai
TF `base_link → camera_link`. Kalau Anda mengubah pose kamera di SDF, **ubah
juga di sana** — keduanya tidak saling membaca.

### 2. Resolusi kamera kedalaman — `Tools/simulation/gz/models/OakD-Lite/model.sdf`

```diff
   <sensor name="StereoOV7251" type="depth_camera">
-    <pose>0.01233 -0.03 .01878 0 0 0</pose>
+    <pose>0.01233 -0.03 .01878 0 0.0708 0</pose>
     <camera>
       <image>
-        <width>640</width>
-        <height>480</height>
+        <width>160</width>
+        <height>120</height>
       <clip>
-        <far>19.1</far>
+        <far>20.0</far>
-    <update_rate>30</update_rate>
+    <update_rate>15</update_rate>
```

Kamera RGB `IMX214` juga **dinonaktifkan** (dikomentari) di berkas yang sama.

**Angka 160 × 120 = 19.200 itu penting.** Setiap kali dokumen ini atau komentar
di kode menyebut "19.200 titik", angkanya berasal langsung dari sini. Pada PX4
bawaan resolusinya 640 × 480 = **307.200 titik**, enam belas kali lebih banyak —
seluruh angka `computation_time_ms`, ukuran voxel, dan kalibrasi grid
`landing_circle` tidak lagi berlaku pada resolusi itu.

### 3. World buatan sendiri — `Tools/simulation/gz/worlds/`

| Berkas | Status di PX4 bawaan |
|---|---|
| `rubicon.sdf` | **tidak ada** — dibuat sendiri |
| `forest_x4.sdf` | **tidak ada** — dibuat sendiri |

`rubicon` adalah world yang dipakai di seluruh contoh perintah dokumen ini.
Karena world ini tidak ada di PX4 bawaan, `PX4_GZ_WORLD=rubicon` akan gagal pada
checkout PX4 yang bersih. `forest.sdf` (tanpa `_x4`) memang bawaan PX4 dan
merupakan berkas yang berbeda.

### 4. Pose spawn per world — `ROMFS/px4fmu_common/init.d-posix/airframes/4002_gz_x500_depth`

Airframe menambahkan pose spawn baku per world:

```sh
if [ "${PX4_GZ_WORLD}" = "rubicon" ]
then
	PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE:=11.689,-5.5473,4.382,0,0,1.2315}
	export PX4_GZ_MODEL_POSE
fi

if [ "${PX4_GZ_WORLD}" = "forest_x4" ]
then
	PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE:=10.2066,20.2602,0,0,0,0}
	export PX4_GZ_MODEL_POSE
fi
```

Alasannya: `rubicon` memakai *terrain*, bukan bidang datar. Heightmap-nya naik
sampai 5 m, sehingga spawn di titik asal menanam wahana **di dalam** tanah.
Perintah di [Cara Menjalankan](#cara-menjalankan) menimpanya secara eksplisit
dengan `PX4_GZ_MODEL_POSE="0,3,8,0,0,0"`.

### Memeriksa apakah pemasangan berhasil

```bash
./px4_assets/install.sh ~/PX4-Autopilot --dry-run
```

Semua baris harus berbunyi `sama`. Kalau ada yang `TIMPA` atau `BARU`, aset
belum terpasang. Pemeriksaan langsung ke berkasnya:

```bash
grep 1.5708 ~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf
ls ~/PX4-Autopilot/Tools/simulation/gz/worlds/rubicon.sdf
```

Baris pertama harus menemukan pose kamera menunduk; baris kedua harus menemukan
world `rubicon`.

> ✅ **Terverifikasi.** `install.sh` diuji terhadap checkout PX4 asli (melaporkan
> `sama` untuk kelima berkas), terhadap PX4 tiruan yang bersih (menyalin
> kelimanya dan mencadangkan berkas lama menjadi `.orig`), dan menolak direktori
> yang bukan akar PX4 dengan kode keluar 1. Simulasi kemudian **benar-benar
> dijalankan** dengan aset ini sampai wahana mendarat — lihat
> [Hasil penerbangan terverifikasi](#hasil-penerbangan-terverifikasi).
>
> Angka-angka dari simulasi yang berjalan mengonfirmasi modifikasinya:
> `/depth_camera/points` terbit pada **15,1 Hz** (sesuai `update_rate` 15) dengan
> `width` **160** (sesuai 160×120 = 19.200 titik), dan `map → camera_link`
> terbaca dengan pitch ≈1,46 rad — kamera memang menunduk.

---

## Build dan Pengujian

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> Di sepanjang dokumen ini `~/ros2_ws` adalah **placeholder** untuk akar
> workspace colcon Anda — yaitu direktori yang memuat `src/`. Ganti dengan
> lokasi kloning Anda sendiri. Begitu pula `~/PX4-Autopilot`: kalau checkout PX4
> Anda ada di tempat lain, tunjuk lewat variabel lingkungan `PX4_SOURCE_DIR`
> atau argumen launch `px4_dir:=`.

Membangun **9 paket**. Semua executable ikut terpasang, `logger_stats`
termasuk — tidak perlu `--packages-select` khusus.

### Menjalankan test

```bash
colcon test --packages-skip px4_msgs
colcon test-result --all
python3 -m pytest scripts/
```

> Lewati `px4_msgs`. Paket itu berisi definisi pesan PX4 dari hulu; test lint-nya
> memproses ribuan berkas hasil generate dan **menggantung lebih dari 10 menit**
> tanpa memberi informasi apa pun tentang kode proyek ini.

### Hasil yang terverifikasi

Seluruh test **perilaku** lulus:

| Suite | Hasil |
|---|---|
| `test_landing_registry` (gtest) | **128 lulus**, 0 gagal |
| `test_depth_bridge_launch` | **19 lulus**, 0 gagal |
| `offboard_mission` | **53 lulus** |
| `px4_offboard_lib` | **39 lulus** |
| `logger_stats` | **20 lulus** (1 dilewati) |
| `keyboard_offboard_control` | **14 lulus** |
| `path_trail` | **9 lulus** |
| `scripts/` (pytest) | **50 lulus** |

**26 kegagalan yang tersisa seluruhnya soal gaya penulisan, bukan perilaku:**
`uncrustify` (16 di `segmentation_node`, 1 di `gz_bridge_ros2`), `lint_cmake`
(4 — "mismatching spaces inside ()" dan "line ends in whitespace"), serta
`flake8` dan `pep257` di `gng_node`. Tidak satu pun menyentuh logika.

> ⚠️ **Algoritma GNG sendiri tidak punya test.** `src/gng_node/test/` hanya berisi
> tiga stub lint bawaan ament. Ini celah yang patut disadari, mengingat GNG
> justru objek yang dibandingkan dalam penelitian ini.

---

## Hasil penerbangan terverifikasi

Seluruh prosedur di [Cara Menjalankan](#cara-menjalankan) dijalankan sungguhan
pada mesin rujukan **tanpa GPU**, dengan world `rubicon` dan front-end GNG.
Wahana lepas landas, memindai, mengunci titik, turun, dan **mendarat**.

| Besaran | Hasil |
|---|---|
| Hasil akhir | `landed` |
| **Galat pendaratan** | **0,0697 m** dari titik yang dipilih |
| Titik terpilih (ENU) | `(2.354, 5.033, −0.252)` |
| Titik sentuh (ENU) | `(2.380, 4.969, −0.208)` |
| Waktu ke commit | 12,2 s |
| Waktu ke waypoint | 24,7 s |
| Total misi | 90,9 s |
| Percobaan scan | 2 |
| Abort | 1 |

Durasi tiap state: `WAIT_POSITION` 0,45 s · `ARMING` 0,1 s · `TAKEOFF` 8,3 s ·
`GOTO` 15,8 s · `SCAN` 26,0 s · `APPROACH` 5,9 s · `DESCEND` 29,2 s ·
`LANDING` 5,1 s.

Metrik persepsi pada penerbangan yang sama: **19.200 → 7.699** titik sesudah
voxel, bidang 1.780 titik, GNG **138,5 ms** median (≈3,3 Hz), `landing_circle`
**12,3 ms** median, 37 kandidat, roughness median 0,0182 m.

### Abort itu bukan kegagalan

Percobaan pertama **sengaja dibatalkan**, dan itu memperlihatkan dua pengaman
bekerja persis seperti rancangannya:

```
Landing point locked at map/ENU (4.75, 6.50, -0.49) after 12.2s in SCAN
APPROACH -> DESCEND
[WARN] Melepas titik pendaratan (4.75, 6.50, -0.49): 59 titik outlier selama 1.6s
[WARN] safety_point TF is 3.5s old (limit 3.5s) — treating the landing target as lost
[WARN] Aborting DESCEND: no fresh safety_point TF
Not returning to map/ENU (4.75, 6.50, -0.49) for the next 30s
DESCEND -> SCAN   (attempt 2/3)
```

Saat mendekat, penjaga rintangan melihat 59 titik `/outlier_cpu` di dalam
silinder di atas titik target, melepas targetnya, dan mengosongkan registry.
TF `safety_point` pun berhenti terbit; node misi membaca TF basi itu sebagai
"target hilang", naik lagi, dan menandai titik tersebut lewat frame
`reject_point` supaya tidak dipilih ulang. Percobaan kedua memilih titik lain
dan mendarat.

> Karena itu **`abort_count` = 1 bukan cacat**. Yang justru mencurigakan adalah
> misi yang tidak pernah membatalkan apa pun.

### Yang ditemukan saat menjalankannya

Menjalankan prosedur ini benar-benar menemukan **satu bug di README** yang tidak
mungkin terlihat dari membaca kode: Terminal 1 tidak menyetel
`GZ_SIM_SERVER_CONFIG_PATH`, sehingga `gz-sim-sensors-system` tidak dimuat dan
`/depth_camera/points` **tidak pernah ada**. Gejalanya menyesatkan — Gazebo
sehat, PX4 sehat, model ter-spawn, hanya kamera yang diam. Sudah diperbaiki di
Terminal 1.

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

> ### Status verifikasi langkah-langkah di bawah
>
> Langkah-langkah ini **sudah dijalankan sungguhan** dari awal sampai wahana
> mendarat, di mesin tanpa GPU. Hasilnya ada di
> [Hasil penerbangan terverifikasi](#hasil-penerbangan-terverifikasi).
>
> | Langkah | Status |
> |---|---|
> | Terminal 1 — Gazebo server | ✅ **terverifikasi** — perlu `GZ_SIM_SERVER_CONFIG_PATH`, lihat peringatan di langkahnya |
> | Terminal 2 — PX4 SITL | ✅ **terverifikasi** — model `x500_depth_0` ter-spawn di world `rubicon` |
> | Terminal 3 — Gazebo GUI | ⚠️ belum dijalankan — murni visualisasi, sengaja dilewati |
> | Terminal 4 — QGroundControl | ⚠️ belum dijalankan — tidak terpasang di mesin rujukan, dan tidak dibutuhkan misi |
> | Terminal 5 — launch jembatan/DDS/TF | ✅ **terverifikasi** — `/depth_camera/points` 15,1 Hz, `/clock` 250 Hz, TF lengkap; `perception:=gng_cpu` dan `:=ransac` juga terverifikasi menyala |
> | Terminal 6 — misi | ✅ **terverifikasi** — takeoff → scan → approach → descend → landing, galat 0,07 m |
> | Terminal 7 — logger metrik | ✅ **terverifikasi** — 10 berkas CSV berisi data penerbangan |
>
> Hanya Terminal 3 dan 4 yang belum dijalankan. Keduanya tidak dibutuhkan misi:
> Terminal 3 hanya menggambar, Terminal 4 hanya memantau.

### Terminal 1 — Jalankan Simulasi Gazebo (Server)

```bash
cd PX4-Autopilot

export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds

# WAJIB: tanpa ini sistem sensor Gazebo tidak dimuat dan kamera kedalaman
# tidak akan pernah menerbitkan apa pun.
export GZ_SIM_SERVER_CONFIG_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/server.config

# Mesin tanpa GPU NVIDIA (jalur baku, ini yang diverifikasi):
gz sim -r -s ~/PX4-Autopilot/Tools/simulation/gz/worlds/rubicon.sdf

# Hanya bila Anda memang punya GPU NVIDIA dengan PRIME render offload:
# __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
#   gz sim -r -s ~/PX4-Autopilot/Tools/simulation/gz/worlds/rubicon.sdf
```

> Dua variabel `__NV_*` itu **tidak ada gunanya tanpa GPU NVIDIA** dan pada
> sebagian sistem justru membuat `gz sim` gagal membuat konteks GL. Pakai baris
> pertama kecuali Anda tahu Anda butuh yang kedua.
> Lihat [Kalau perangkat Anda tidak punya GPU](#kalau-perangkat-anda-tidak-punya-gpu)
> untuk daftar lengkap node yang tetap bisa dijalankan.

> Tunggu hingga simulasi server siap (biasanya 3–5 detik) sebelum lanjut ke langkah berikutnya.

> ### ⚠️ `GZ_SIM_SERVER_CONFIG_PATH` itu wajib — tanpa ini drone tidak bisa *arm*
>
> Konfigurasi server Gazebo bawaan (`~/.gz/sim/8/server.config`) hanya memuat
> **tiga** sistem: Physics, UserCommands, SceneBroadcaster. Berkas `server.config`
> milik PX4 memuat **sebelas**, termasuk semua sistem sensor:
>
> | Sistem | Bawaan Gazebo | `server.config` PX4 |
> |---|---|---|
> | physics, user-commands, scene-broadcaster | ✅ | ✅ |
> | **imu** | ❌ | ✅ |
> | **air-pressure** (barometer) | ❌ | ✅ |
> | **magnetometer** (kompas) | ❌ | ✅ |
> | **navsat** (GPS) | ❌ | ✅ |
> | **sensors** (kamera/depth) | ❌ | ✅ |
> | contact, air-speed, apply-link-wrench | ❌ | ✅ |
>
> Tanpa sistem-sistem itu, topik sensornya **tetap muncul** di `gz topic -l` —
> karena memang dideklarasikan di SDF — tetapi **tidak punya publisher dan tidak
> pernah mengalirkan data**. Gazebo terlihat sehat, PX4 terlihat sehat, model
> ter-spawn, dan tidak ada satu pun pesan kesalahan. Yang muncul justru ini:
>
> ```
> WARN [health_and_arming_checks] Preflight Fail: Accel Sensor 0 missing
> WARN [health_and_arming_checks] Preflight Fail: barometer 0 missing
> WARN [health_and_arming_checks] Preflight Fail: ekf2 missing data
> WARN [health_and_arming_checks] Preflight Fail: Gyro Sensor 0 missing
> WARN [health_and_arming_checks] Preflight Fail: Found 0 compass (required: 1)
> WARN [commander] Arming denied: Resolve system health failures first
> ```
>
> Jadi kegagalannya **bukan** sekadar kamera diam — **drone tidak bisa lepas
> landas sama sekali.** `waypoint_node` pun tidak akan pernah keluar dari
> `WAIT_POSITION`.
>
> **Memastikan berhasil.** Setelah Terminal 2 berjalan, ketiganya harus berhasil:
>
> ```bash
> # 1. variabel benar-benar terbawa ke proses gz sim
> tr '\0' '\n' < /proc/$(pgrep -f "gz sim -r -s" | head -1)/environ | grep GZ_SIM_SERVER_CONFIG_PATH
>
> # 2. topik kamera kedalaman ada
> gz topic -l | grep depth_camera
>
> # 3. IMU benar-benar menerbitkan data (bukan sekadar topiknya ada)
> gz topic -e -t /world/rubicon/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu -n 1
> ```
>
> Perintah ketiga itu yang menentukan. Perintah `gz topic -l` saja **menipu**:
> topiknya terdaftar meski tidak ada yang menerbitkannya. Kalau perintah ketiga
> menggantung tanpa keluaran, sistem sensor tidak dimuat.
>
> **`export` berlaku per terminal.** Menyetelnya di terminal lain tidak
> berpengaruh — variabel ini harus ada di terminal yang menjalankan `gz sim`.
> Kalau Anda terlanjur menjalankan `gz sim` tanpa variabel ini, **hentikan
> Terminal 1 dan Terminal 2, lalu ulangi keduanya**; menyetelnya belakangan tidak
> memperbaiki server yang sudah berjalan.

---

### Terminal 2 — Jalankan PX4 SITL

```bash
cd PX4-Autopilot

sleep 5 && \
  PX4_GZ_WORLD=rubicon \
  PX4_GZ_STANDALONE=1 \
  make px4_sitl gz_x500_depth
```

> Parameter `PX4_GZ_MODEL_POSE="0,3,8,0,0,0"` berarti drone dimulai di posisi x=0, y=3, z=8 (meter), tanpa rotasi.

> ⚠️ **`PX4_GZ_WORLD=rubicon` hanya bekerja bila checkout PX4 Anda sudah
> dimodifikasi.** World `rubicon` tidak ada di PX4 bawaan, dan pada PX4 bersih
> kamera kedalaman menghadap ke depan sehingga seluruh pipeline diam. Baca
> [Modifikasi PX4-Autopilot yang Diperlukan](#modifikasi-px4-autopilot-yang-diperlukan)
> lebih dulu.

---

### Terminal 3 — Jalankan Gazebo GUI (Visualisasi)

```bash
# Mesin tanpa GPU NVIDIA (jalur baku):
gz sim -g
```

Kalau GUI gagal membuka karena versi OpenGL, paksa lewat Mesa — ini **tidak**
butuh NVIDIA:

```bash
MESA_GL_VERSION_OVERRIDE=4.5 gz sim -g
```

Hanya bila Anda memang punya GPU NVIDIA dengan PRIME render offload:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
gz sim -g
```

> Terminal ini **murni visualisasi** dan boleh dilewati sepenuhnya. Persepsi,
> misi, dan pencatatan metrik berjalan tanpa GUI. Pada mesin lemah, lewati
> terminal ini — atau pakai `headless:=true` bila memakai launch file — supaya
> beban render tidak mencemari `computation_time_ms` yang sedang Anda ukur.

---

### Terminal 4 — Jalankan QGroundControl (GCS)

```bash
chmod +x ~/QGroundControl-x86_64.AppImage
~/QGroundControl-x86_64.AppImage
```

> QGroundControl akan otomatis terhubung ke PX4 SITL melalui UDP port 14550.
> Pastikan PX4 SITL (Terminal 2) sudah berjalan sebelum membuka QGroundControl.

---

### Terminal 5 — Jembatan, Agen DDS, TF, dan (opsional) Persepsi

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 launch gz_bridge_ros2 depth_bridge_launch.py \
  world:=rubicon start_px4:=false rviz:=false
```

Satu perintah ini menggantikan empat proses yang dulu punya terminal
sendiri-sendiri, dan bisa sekalian menyalakan front-end segmentasinya.
Terverifikasi menyalakan:

| Node / proses | Perannya |
|---|---|
| `MicroXRCEAgent udp4 -p 8888` | jembatan PX4 ↔ ROS 2 |
| `/gz_clock_bridge` | `/clock` dari Gazebo — **wajib** |
| `/depth_camera_bridge` | `/depth_camera/points` dari Gazebo |
| `/drone_kinematic` | rantai TF `map → odom → base_link → camera_link` — **wajib** |
| `/path_trail` | jejak lintasan untuk RViz (matikan dengan `path_trail:=false`) |

Tambahkan `rviz:=true` bila ingin RViz2 ikut terbuka.

> `/clock` **wajib**: seluruh rantai berjalan dengan `use_sim_time:=true`, dan
> tanpanya jam ROS tidak pernah maju sehingga setiap timeout langsung
> tersandung. Tanpa `drone_kinematic`, seluruh pipeline persepsi diam karena
> tidak ada TF yang menghubungkan `camera_link` ke `map`.

#### Memilih front-end persepsi di launch

Launch ini juga bisa langsung menyalakan front-end segmentasinya lewat argumen
`perception:`. Inilah tombol yang dipakai untuk perbandingan GNG lawan RANSAC.

| `perception:=` | Yang dinyalakan | Topic keluaran | Status |
|---|---|---|---|
| `none` *(baku)* | tidak ada | — | ✅ |
| `gng_cpu` | `dbl_gng_cpu_node` (nama node `dbl_gng_cpu`) | `/plane_cpu`, `/outlier_cpu` | ✅ terverifikasi |
| `ransac` | `plane_segmentation_ransac` | `/plane`, `/outlier` | ✅ terverifikasi |
| `gng_gpu` | `dbl_gng_node` | `/plane`, `/outlier` | ❌ butuh `torch` |

Hanya **satu** yang bisa dipilih, supaya dua front-end tidak pernah berebut CPU.

Parameter yang diberikan launch sudah **disetarakan** untuk kedua algoritma —
satu argumen `voxel_leaf` dipetakan ke `voxel_leaf` milik GNG *dan* ke
`leaf_size` milik RANSAC, dan PassThrough RANSAC dimatikan. Terverifikasi pada
`perception:=ransac`:

```
input_topic = /depth_camera/points     leaf_size = 0.15
z_min       = -1000.0                  z_max     = 1000.0
```

**Bagaimana ini berinteraksi dengan node misi.** `waypoint_node` juga menyalakan
front-end-nya sendiri saat tiba di waypoint, tetapi keduanya **tidak
bertabrakan**: `PerceptionSupervisor` mencocokkan **nama node** yang sudah hidup
lewat `get_node_names()` dan melewati perintah yang namanya sudah ada, sambil
mencatat *"already running — not starting a second one."* Karena launch sengaja
menamai node GNG-nya `dbl_gng_cpu` — nama yang sama dengan yang dicari node misi
— menjalankan `perception:=gng_cpu` lebih dulu berarti misi hanya perlu
menyalakan `landing_circle`.

> Untuk alur misi biasa, `perception:=none` (baku) sudah cukup dan itulah yang
> dipakai pada [penerbangan terverifikasi](#hasil-penerbangan-terverifikasi) —
> node misi menyalakan GNG sendiri. Menyalakannya dari launch berguna kalau Anda
> ingin melihat hasil segmentasi di RViz **sebelum** misi dimulai.

> ⚠️ **Untuk penerbangan pembanding RANSAC, `perception:=ransac` saja tidak
> cukup.** Nilai baku `perception_commands` pada node misi menunjuk
> `landing_circle` ke `/plane_cpu`, sedangkan RANSAC menerbitkan ke `/plane`.
> Tanpa menimpa parameter itu, RANSAC akan berjalan dan tidak ada yang membaca
> keluarannya. Lihat [perbandingan dua penerbangan](#hasil-penerbangan-terverifikasi)
> dan bagian `waypoint_node`.

> ⚠️ **`gng_backend:=cpu` bukan argumen yang ada.** Yang dideklarasikan hanya:
> `world`, `model`, `instance`, `dds_port`, `agent_bin`, `headless`, `rviz`,
> `px4_dir`, `start_px4`, `perception`, `perception_input`, `voxel_leaf`,
> `path_trail`. Argumen yang tidak dikenal **diabaikan diam-diam** — tanpa error
> dan tanpa peringatan. Diuji: menjalankan launch dengan `gng_backend:=cpu`
> membuat `/dbl_gng_cpu` **tidak menyala sama sekali**, dan tidak ada satu pun
> pesan yang memberitahu Anda. Periksa daftar argumen dengan:
>
> ```bash
> ros2 launch gz_bridge_ros2 depth_bridge_launch.py --show-args
> ```

Bila Anda perlu menjalankan ketiganya sendiri tanpa launch file:

```bash
MicroXRCEAgent udp4 -p 8888

ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

ros2 run gz_bridge_ros2 drone_kinematic --ros-args -p use_sim_time:=true
```

---

### Terminal 6 — Jalankan Misi

```bash
cd ~/ros2_ws
source install/setup.bash
```

#### Jalur normal — satu perintah

```bash
ros2 launch gz_bridge_ros2 depth_bridge_launch.py world:=rubicon rviz:=true perception:=gng_cpu
```

Itu saja. `waypoint_node` **menyalakan sendiri** `dbl_gng_cpu_node` dan
`landing_circle` saat drone tiba di waypoint, lalu mematikan keduanya setelah
mendarat. Rantai persepsinya tidak berjalan selama lepas landas dan
perjalanan, karena tidak ada yang memakai keluarannya sebelum drone sampai —
dan GNG sendirian terukur ~700% CPU.

Urutan yang harus terlihat di log:

```
WAIT_POSITION -> ARMING -> TAKEOFF -> GOTO -> SCAN -> APPROACH -> DESCEND
  -> LANDING -> LANDED
Landed and disarmed at map/ENU (-1.65, 6.71, -1.77), 0.28 m dari titik terpilih
```

> ⚠️ **Koordinatnya frame `map` (ENU: x timur, y utara, z ke atas)**, sama
> seperti RViz — **bukan** NED seperti `/fmu/out/vehicle_local_position`. Angka
> dari topik itu harus ditukar sumbunya dan dibalik tanda z-nya: NED
> `(8.6, -0.17, -11)` menjadi ENU `(-0.17, 8.6, 11.0)`. Cara termudah membaca
> posisi drone langsung dalam ENU:
> `ros2 run tf2_ros tf2_echo map base_link`.

#### ⚠️ Jangan jalankan GNG manual

Ini bukan soal selera. `dbl_gng_cpu_node` yang dibiarkan hidup lama bisa
**macet tanpa mati**: prosesnya tetap berjalan di ~350% CPU dan tetap
menerbitkan `/plane_cpu` pada 5 Hz, tapi **isinya kosong** — padahal
`/depth_camera/points` masih 100% valid. Gejalanya di sisi hilir:

```
[WARN] [landing_circle]: Input cloud empty
[INFO] [landing_circle]: Comp: 0.08 ms | Valid: 0 / 0 (0.0%) | Candidates: 0
```

Dua uji terbang gagal karena ini sebelum ketahuan. Penjaga duplikat di node
misi **tidak menolong**: ia memeriksa apakah node itu **hidup**, bukan apakah
keluarannya waras — jadi ia justru melewati GNG yang sudah macet dan memakai
yang rusak itu.

Pastikan bersih sebelum terbang:

```bash
pgrep -af "dbl_gng_cpu_node|landing_circle|keyboard_offboard_node"   # harus kosong
```

`keyboard_offboard_node` ikut diperiksa karena ia dan `waypoint_node`
sama-sama menyiarkan setpoint ke drone yang sama; menjalankan keduanya
bersamaan membuat drone bergerak tidak menentu.

#### Visualisasi (RViz)

```bash
rviz2 -d src/gz_bridge_ros2/config/x500_depth.rviz
```

Silinder pada `/landing_candidates`: **hijau** = boleh dipilih, **oranye** =
sedang terhalang (entrinya tetap disimpan, tapi tidak boleh jadi target).
`Fixed Frame` sudah diset ke `map`.

#### Jalur debug — tiap node di terminal sendiri

Dipakai kalau ingin membaca log satu node terpisah, atau sengaja mematikan
salah satunya untuk memicu jalur abort:

```bash
# Terminal A
ros2 run gng_node dbl_gng_cpu_node --ros-args -p use_sim_time:=true

# Terminal B
ros2 run segmentation_node landing_circle --ros-args \
  -p input_topic:=/plane_cpu -p use_sim_time:=true \
  -p base_frame:=scan_center -p sticky_target:=true \
  -p select_radius_m:=8.0 -p reject_frame:=reject_point

# Terminal C
ros2 run offboard_mission waypoint_node --ros-args \
  -p target_x:=-0.17 -p target_y:=8.6 -p target_z:=11.0 \
  -p use_sim_time:=true -p manage_perception:=false
```

Perhatikan parameter `landing_circle` di Terminal B: pada jalur normal
semuanya diisi otomatis oleh node misi. Kalau dijalankan tangan dan parameter
itu dilupakan, seleksinya kembali memakai posisi drone sebagai acuan dan TF
`safety_point` akan berpindah tiap kali drone bergerak — yang dibaca node misi
sebagai "target hilang".

#### Parameter misi

Semua angka di bawah hasil ukur, bukan tebakan:

| Parameter | Default | Alasan |
|---|---|---|
| `enable_landing` | `true` | `false` → berhenti melayang di atas titik aman, tidak turun. Dipakai untuk uji terbang pertama. |
| `scan_radius_m` | `8.0` | Jejak kamera menghadap bawah = 1,475 × tinggi; radius di luar itu tidak akan pernah cocok dengan apa pun. |
| `abort_floor_m` | `4.0` | Di 3,6 m jejaknya tinggal 5,3 m — sekitar sembilan sel grid 0,6 m, dan uji fill-ratio tidak lagi sanggup menilai cakram 1,3 m. |
| `commit_track_tol_m` | `1.5` | Lantai komitmen hanya berlaku bila drone benar-benar mengikuti setpoint-nya. |
| `reject_hold_s` | `30.0` | Lama sebuah titik yang gagal tidak boleh dipilih lagi. Artinya "baru saja gagal di sana", bukan vonis permanen. |
| `target_timeout_s` | `3.5` | `safety_point` terbit 0,79 Hz dengan jeda maksimum terukur 1,68 s. |
| `scan_timeout_s` | `45.0` | Termasuk waktu startup dua proses persepsi. |
| `max_scan_attempts` | `3` | Berapa kali pendekatan boleh dibatalkan sebelum menyerah dan melayang. |
| `manage_perception` | `true` | `false` → jalur debug di atas. |

> **Harga dari `abort_floor_m`, disebut terang-terangan:** di bawah 4 meter
> misi tidak akan bereaksi terhadap objek yang baru masuk ke zona pendaratan.
> Perlindungan itu bukan dilepas — memang **tidak tersedia** di ketinggian
> segitu, karena kamera yang menghadap ke bawah sudah tidak sanggup menilai
> cakram pendaratan. Lantai yang lebih rendah tidak melindungi apa pun; ia
> hanya membatalkan pendaratan yang sebenarnya baik. Yang membuatnya aman:
> lantai ini hanya berlaku bila pelacakan posisinya bagus
> (`commit_track_tol_m`) — kalau drone melenceng jauh dari setpoint-nya,
> abort tetap aktif berapa pun ketinggiannya.

---

### Terminal 7 — Logger Metrik (opsional)

```bash
ros2 run logger_stats logger_stats --ros-args -p run_id:=gng -p use_sim_time:=true
```

> Klaim README lama bahwa paket ini "tidak ikut terbangun secara default" sudah
> **tidak berlaku**: `colcon build` polos membangun kesembilan paket,
> `logger_stats` termasuk. Tidak perlu `--packages-select`.

> Nama executable-nya `logger_stats`, **bukan** `stats_logger_node`.
> `stats_logger_node` adalah nama modul Python-nya, bukan entry point-nya.

**`run_id` wajib diisi dan node menolak jalan tanpanya.** Ia dicap ke tiap
baris dan ke nama tiap berkas. Perbandingan GNG lawan RANSAC dilakukan
bergantian — satu penerbangan masing-masing — jadi tanpa penanda penerbangan
kedua set data bercampur di berkas yang sama dan tidak bisa dipisahkan lagi
sesudahnya. Sebuah penerbangan tidak bisa diulang dengan murah; lebih baik
node ini menolak menyala daripada menghasilkan data yang tidak bisa dipakai.

Keluaran tersimpan di `~/ros2_logs/` (ubah dengan `-p output_dir:=...`), enam
berkas per penerbangan, **satu baris per pesan**:

| berkas | isi |
|---|---|
| `<run>_<stamp>_gng.csv` | `/segmentation_stats_cpu` — satu baris per frame GNG |
| `<run>_<stamp>_ransac.csv` | `/segmentation_stats` — satu baris per frame RANSAC |
| `<run>_<stamp>_landing.csv` | `/landing_circle_stats`, field skalar |
| `<run>_<stamp>_candidates.csv` | **format panjang**: satu baris per kandidat per frame |
| `<run>_<stamp>_center.csv` | `/safe_circle_center_coords` |
| `<run>_<stamp>_trajectory.csv` | TF `map → base_link`, disampel 10 Hz |

Ditambah **satu berkas per jenis peristiwa** pada `/mission_events` —
`<run>_<stamp>_mission_transition.csv`, `_mission_commit.csv`,
`_mission_summary.csv` — bukan satu berkas `_mission.csv` gabungan.

`candidates.csv` yang membuat sebaran `roughness_m` dan `score` bisa diplot.
`mission_summary.csv` memuat `landing_error_m`, `time_to_commit_s`,
`scan_attempts`, `abort_count`, dan durasi tiap state — angka-angka yang
sebelumnya hanya muncul di log konsol dan hilang begitu terminal ditutup.
Peristiwa misi dipisah per jenis (`mission_transition`, `mission_abort`,
`mission_commit`, `mission_summary`) karena bentuk barisnya berbeda-beda dan
CSV menuntut baris yang seragam.

`<run>_trajectory.csv` merekam posisi drone dari TF `map → base_link` pada
10 Hz: `x`, `y`, `z`, `yaw_deg`, dan `stamp_ns`. Ia direkam di sini, bukan di
`path_trail`, supaya `path_trail:=false` hanya mematikan gambar di RViz tanpa
ikut menghapus datanya.

#### Membaca hasilnya

```bash
python3 scripts/analyze_metrics.py    ~/ros2_logs      # tabel ringkasan
python3 scripts/plot_flight.py        ~/ros2_logs gng  # lintasan + ketinggian
python3 scripts/plot_stats.py         ~/ros2_logs --list   # kolom apa pun
python3 scripts/plot_trajectory3d.py  ~/ros2_logs gng  # lintasan 3D
```

##### `plot_trajectory3d.py` — lintasan dalam tiga dimensi

```bash
python3 scripts/plot_trajectory3d.py ~/ros2_logs terbang_pertama
python3 scripts/plot_trajectory3d.py ~/ros2_logs terbang_kedua \
    --azim 60 --elev 20 --out ~/lintasan.png
```

| opsi | arti |
|---|---|
| `--azim` | putaran terhadap sumbu z, derajat (default 45) |
| `--elev` | kemiringan pandangan, derajat (default 25) |
| `--all-frames` | jangan batasi ke jendela udara |
| `--out` | berkas PNG keluaran |

Proyeksinya dihitung sendiri dengan matplotlib 2D, **tidak memakai Axes3D**.
Bukan pilihan gaya: di mesin ini `mpl_toolkits` berasal dari paket apt untuk
matplotlib 3.5.1 sedangkan matplotlib-nya versi pip 3.10.5, dan yang lama
mengimpor `matplotlib.docstring` yang sudah tidak ada. Penyebabnya bukan urutan
`sys.path` melainkan aturan impor Python — versi pip adalah *namespace package*
tanpa `__init__.py` sedangkan versi apt paket biasa, dan paket biasa selalu
menang. Menggambar proyeksinya sendiri melewati seluruh persoalan itu.

Dua hal yang membuat gambarnya terbaca sebagai ruang, bukan kurva datar:

- **Skala ketiga sumbu disamakan.** Pada penerbangan yang diuji, x membentang
  1,7 m sedangkan y 15,6 m dan z 14,0 m. Meregangkan tiap sumbu agar memenuhi
  kanvas akan menggambar goyangan menyamping selebar perjalanan majunya —
  gerakan yang tidak pernah terjadi.
- **Kedalamannya digambar**, karena gambar diam tidak bisa diputar: ada
  bayangan lintasan di lantai, garis jatuh berkala, dan kisi lantai yang
  sekaligus membawa skalanya. Sumbu 2D-nya disembunyikan — koordinat hasil
  proyeksi tidak punya arti fisik, dan mencetak angkanya mengundang salah baca.

Lintasannya diwarnai menurut urutan waktu (terang → gelap), sehingga arah
perjalanan terbaca tanpa panah.

##### `plot_stats.py` — plot kolom apa pun

Tidak perlu menghafal nama kolom; tanyakan saja:

```bash
python3 scripts/plot_stats.py ~/ros2_logs --list          # run & berkas
python3 scripts/plot_stats.py ~/ros2_logs --list landing  # kolom numeriknya
```

Lalu plot apa saja:

```bash
# satu variabel
python3 scripts/plot_stats.py ~/ros2_logs landing computation_time_ms

# beberapa variabel -> beberapa panel bertumpuk
python3 scripts/plot_stats.py ~/ros2_logs landing \
    computation_time_ms num_candidates selected_score --events

# sebaran
python3 scripts/plot_stats.py ~/ros2_logs candidates roughness_m --hist

# satu besaran terhadap besaran lain
python3 scripts/plot_stats.py ~/ros2_logs landing obstacle_points \
    --x target_alt_m

# bandingkan dua penerbangan
python3 scripts/plot_stats.py ~/ros2_logs gng computation_time_ms \
    --runs 074807 081836

# dua besaran, satu grafik: kiri waktu komputasi, kanan ketinggian
python3 scripts/plot_stats.py ~/ros2_logs landing computation_time_ms \
    --right target_alt_m --runs 081836 --events

# kolom kanan boleh dari berkas lain
python3 scripts/plot_stats.py ~/ros2_logs landing num_candidates \
    --right z --right-kind trajectory --runs 081836

# SATU garis: waktu komputasi GNG sebagai fungsi ketinggian drone
python3 scripts/plot_stats.py ~/ros2_logs gng computation_time_ms \
    --x z --x-kind trajectory --runs freeze

# penskalaan: waktu komputasi terhadap jumlah titik, sebagai garis tren
python3 scripts/plot_stats.py ~/ros2_logs gng computation_time_ms \
    --x downsampled_points --trend
```

**Kapan `--trend`, kapan sebar.** Tanpa `--trend`, sumbu x berupa kolom
digambar sebagai titik — bentuk yang jujur untuk data mentah. `--trend`
membin sumbu x lalu menggambar **median per bin** sebagai garis, dengan pita
**p10–p90** di belakangnya.

Menyambungkan titik mentah apa adanya bukan pilihan yang tersedia, dan itu
disengaja: banyak frame berbagi nilai x berdekatan, sehingga garisnya menjadi
zig-zag vertikal yang tidak berarti apa pun.

Pitanya wajib ikut, bukan hiasan. Pada data GNG, di bin 6.476–7.546 titik
p10 dan p90 berjarak 72 sampai 283 ms — garis median telanjang akan
menyatakan ketepatan yang tidak dimiliki datanya. Bin dengan kurang dari tiga
sampel dilewati, dan jumlah yang dilewati dicetak.

`--trend` ditolak bila sumbu x adalah waktu: deret waktu sudah berupa garis,
dan membin ulangnya hanya menghaluskan sesuatu yang sudah benar.

`--x-kind` menjodohkan baris dua berkas menurut `recv_ns`, jam yang dicap
perekam yang sama untuk semua topik. Pasangan yang terpaut lebih dari 0,5
detik **ditolak**, dan jumlah baris yang tidak terjodohkan dicetak —
menjodohkan sampel berjarak beberapa detik menghasilkan titik yang terlihat
rapi tapi memasangkan ketinggian dengan frame yang bukan miliknya.

Bedakan dua "ketinggian" yang tersedia: `z` dari `trajectory` adalah posisi z
drone dalam frame `map` (bisa negatif, karena tanah di titik pendaratan bisa
di bawah origin), sedangkan `target_alt_m` dari `landing` adalah tinggi drone
**di atas cakram terpilih** — itu yang biasanya dimaksud "ketinggian di atas
tanah".

| opsi | arti |
|---|---|
| `--list [JENIS]` | daftar run, atau kolom numerik satu berkas |
| `--runs SUB...` | saring run lewat substring (`081836` cukup) |
| `--x AXIS` | `time` (default), `stamp_ns`, atau nama kolom → sebar |
| `--trend [N]` | garis tren: median per bin + pita p10–p90 (default 20 bin) |
| `--hist` / `--bins N` | sebaran, bukan deret waktu |
| `--right KOLOM` | kolom kedua pada sumbu y **kanan**, skala terpisah |
| `--right-kind JENIS` | berkas asal `--right` bila beda dari yang kiri |
| `--events` | tandai transisi state (hanya bila satu run dipilih) |
| `--all-frames` | jangan batasi ke jendela udara |
| `--out PATH` | berkas PNG keluaran |

Dua hal yang **tidak** bisa diubah lewat argumen, karena keduanya soal benar
atau salah:

- **Beberapa variabel menjadi beberapa panel, bukan dua sumbu y.** Waktu
  komputasi (puluhan milidetik) dan jumlah kandidat (puluhan buah) tidak
  sesatuan; menumpuknya di satu sumbu ganda membuat bentuk kurvanya bisa
  diatur mengatakan apa saja tergantung skala mana yang dipilih.
- **Sumbu waktu dihitung sejak lepas landas**, memakai `recv_ns` yang ada di
  setiap baris setiap berkas. Memakai `stamp_ns` untuk data dan `recv_ns`
  untuk peristiwa akan menggambar garis TAKEOFF di t=0 padahal `landing.csv`
  baru mulai saat drone tiba — rapi, dan salah.

`--right` adalah pengecualian yang disengaja dari aturan pertama, untuk kasus
"kiri waktu komputasi, kanan ketinggian, bawah detik sejak lepas landas".
**Kedua skala berdiri sendiri**, jadi di mana kedua kurva berpotongan atau
seberapa jauh jaraknya tidak berarti apa-apa — geser salah satu batas sumbu
dan hubungan yang tampak ikut berubah. Yang tetap sahih hanya bentuk
masing-masing kurva terhadap sumbu x. Skripnya mencetak catatan itu tiap kali
`--right` dipakai. Pengamannya: tiap kurva diwarnai sama dengan sumbunya, dan
yang kanan digaris putus-putus sehingga pasangannya tidak bergantung pada
warna saja. `--right` menolak berjalan bila lebih dari satu run terpilih:
empat kurva pada dua skala tidak bisa dibaca siapa pun.

`analyze_metrics.py` hanya menghitung frame **antara lepas landas dan
mendarat**. Rekaman biasanya mencakup menit-menit saat drone masih di tanah,
dan pada frame itu kamera hanya beberapa sentimeter dari tanah sehingga awan
titiknya nyaris kosong — terukur satu penerbangan dengan 60% frame berisi
kurang dari 50 titik, yang menarik median `downsampled_points` dari 1.845 ke 8.
Angka seperti itu tidak salah membaca berkasnya; ia salah menjawab
pertanyaannya.

`plot_flight.py` menghasilkan dua panel: tampak atas berisi lintasan, seluruh
kandidat diwarnai menurut `score`, titik terpilih dan titik mendarat; serta
profil ketinggian terhadap waktu dengan batas antar state ditandai.

> **Muatan topik stats sekarang JSON**, bukan teks berformat. Versi sebelumnya
> mengurainya dengan regex, dan regex itu patah senyap setiap kali ada field
> baru: tidak ada pesan galat, hanya berkas CSV kosong. Kalau logger mencetak
> "Muatan bukan JSON yang sah", berarti ada node penerbit versi lama yang masih
> berjalan — build ulang workspace-nya.

Menjalankan untuk metrik, urutan yang dianjurkan:

```bash
# Penerbangan 1 — GNG (jalur normal)
ros2 run logger_stats logger_stats --ros-args -p run_id:=gng
ros2 run offboard_mission waypoint_node --ros-args \
  -p target_x:=-0.17 -p target_y:=8.6 -p target_z:=11.0 \
  -p use_sim_time:=true -p run_id:=gng

# Penerbangan 2 — RANSAC (GNG dimatikan sepenuhnya)
# Alternatif yang lebih rapi: nyalakan RANSAC dari launch Terminal 5 dengan
#   ros2 launch gz_bridge_ros2 depth_bridge_launch.py \
#     world:=rubicon start_px4:=false perception:=ransac
# lalu lewati baris plane_segmentation_ransac di bawah ini.
ros2 run segmentation_node plane_segmentation_ransac --ros-args \
  -p input_topic:=/depth_camera/points -p z_min:=-1000.0 -p z_max:=1000.0 \
  -p leaf_size:=0.15 -p use_sim_time:=true
ros2 run segmentation_node landing_circle --ros-args \
  -p input_topic:=/plane -p outlier_topic:=/outlier -p use_sim_time:=true \
  -p base_frame:=scan_center -p sticky_target:=true \
  -p select_radius_m:=8.0 -p reject_frame:=reject_point
```

> Keluaran RANSAC **jauh lebih padat** daripada `/plane_cpu`, sedangkan setelan
> grid `landing_circle` (`grid_cell` 0,6 m, `min_pts_per_cell` 2) dikalibrasi
> untuk kerapatan GNG ~5 titik/m². Penerbangan RANSAC perlu setelan grid
> sendiri. Metrik segmentasi tetap sebanding antar penerbangan; metrik
> pendaratan **tidak** boleh diklaim setara tanpa menyebut setelannya — dan
> setelan yang dipakai ikut tercatat di tiap baris (`grid_cell_m`,
> `min_pts_per_cell`).

**`use_sim_time:=true` bukan hiasan.** Kolom `latency_ms` mengurangi waktu
terbit dengan stempel awan masukan; stempel itu berasal dari jam simulasi
Gazebo. Tanpa `use_sim_time`, pengurangan itu mencampur jam dinding dengan jam
simulasi. Node-nya menerbitkan `latency_ms` kosong dalam keadaan itu — sengaja
tidak melaporkan apa pun daripada melaporkan angka yang salah dan tampak wajar.

---

## Penjelasan Tiap Node

Workspace ini punya **12 node executable**. Tabel di bawah adalah ringkasan
statusnya; rinciannya menyusul per node.

| Node | Executable | Paket | Peran | Status |
|---|---|---|---|---|
| `drone_kinematics` ¹ | `drone_kinematic` | `gz_bridge_ros2` | penerbit TF | ✅ |
| `dbl_gng_cpu` | `dbl_gng_cpu_node` | `gng_node` | segmentasi GNG (produksi) | ✅ |
| `dbl_gng` | `dbl_gng_node` | `gng_node` | segmentasi GNG (GPU) | ❌ butuh `torch` |
| `plane_segmentation_ransac` | idem | `segmentation_node` | segmentasi RANSAC (pembanding) | ✅ |
| `landing_circle` | idem | `segmentation_node` | pencari titik pendaratan | ✅ |
| `waypoint_mission` ¹ | `waypoint_node` | `offboard_mission` | misi otonom | ✅ ² |
| `keyboard_offboard_node` | idem | `keyboard_offboard_control` | terbang manual | ✅ |
| `path_trail` | `path_trail_node` | `path_trail` | jejak lintasan RViz | ✅ |
| `stats_logger` ¹ | `logger_stats` | `logger_stats` | perekam metrik | ✅ |
| `cloud_size_node` | idem | `segmentation_node` | diagnostik | ⚠️ topic ZED mati |
| `plane_segmentation_gng` | idem | `segmentation_node` | GNG C++ lama | ⚠️ nama bertabrakan |
| `plane_segmentation_gng` | `ddgng_node` | `segmentation_node` | GNG C++ lama | ⚠️ nama bertabrakan |

¹ Nama node berbeda dari nama executable — perhatikan saat memakai
`ros2 node info` / `ros2 param get`.
² Diverifikasi dua kali: tanpa PX4 node menunggu dan menolak arming dengan
benar; dengan PX4 SITL misi penuh berjalan sampai mendarat, galat 0,07 m.

**Arti lambang:** ✅ dijalankan dan diamati pada mesin rujukan ·
⚠️ berjalan tetapi berperilaku bermasalah · ❌ tidak dapat dijalankan atau
terbukti tidak berfungsi.


### `drone_kinematic` (paket `gz_bridge_ros2`) — ✅ terverifikasi

**Penerbit TF, bukan node kendali.** Versi README sebelumnya menggambarkannya
sebagai node offboard control di paket `offboard_control` — keduanya keliru.
Kendali offboard ada di `waypoint_node` dan `keyboard_offboard_node`.

Yang dikerjakannya: membaca `/fmu/out/vehicle_odometry` (NED) dan menerbitkan
rantai TF dalam ENU, sehingga seluruh pipeline persepsi punya frame `map` yang
sama untuk mengakumulasi kandidat lintas frame.

| Arah | Topic / TF | Tipe |
|------|-----------|------|
| Subscribe | `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` |
| TF Publish | `map` → `odom` | identitas — origin `map` = origin lokal PX4 |
| TF Publish | `odom` → `base_link` | pose drone, hasil konversi NED → ENU |
| TF Publish | `base_link` → `camera_link` | pemasangan kamera (menghadap bawah) |

Karena `map → odom` adalah identitas, koordinat ENU node misi dan NED PX4
berbagi titik asal yang sama — konversinya cukup tukar sumbu dan balik tanda z,
tanpa offset.

**Konversi NED → ENU terverifikasi.** Dengan `/fmu/out/vehicle_odometry` palsu
berisi NED `(n, e, d) = (3, 2, −11)`, TF `map → base_link` terbaca persis
`[2.000, 3.000, 11.000]` — yaitu `(e, n, −d)`.

**Offset kamera terverifikasi terhadap SDF.** `base_link → camera_link` di kode
adalah `t = (0.22, 0.03, 0.242)`, `rpy = (0, 1.5708, 0)`, sama persis dengan
`<pose>0.22 0.03 0.242 0 1.5708 0</pose>` pada model `x500_depth`, yang
menyertakan kamera **OakD-Lite** (bukan IMX214 seperti klaim README lama).

| Parameter | Baku | Keterangan |
|---|---|---|
| `mode` | `odometry` | `odometry` = rantai penuh; `static` = hanya `map→odom` + `base_link→camera_link` |
| `odometry_topic` | `/fmu/out/vehicle_odometry` | sumber pose PX4 |

> ⚠️ **Nama node ≠ nama executable.** Dijalankan lewat `ros2 run gz_bridge_ros2
> drone_kinematic`, nama node yang muncul di `ros2 node list` adalah
> **`drone_kinematics`** (berakhiran "s"). Lewat launch file namanya di-remap
> menjadi `drone_kinematic`. Kalau `ros2 node info drone_kinematic` menjawab
> "Unable to find node", node-nya kemungkinan besar hidup — hanya namanya lain.

> Dengan `mode:=static` rantai TF sengaja **tidak lengkap**: `odom → base_link`
> tidak diterbitkan, sehingga `map → camera_link` tidak bisa diselesaikan dan
> seluruh pipeline persepsi diam. Mode itu untuk memeriksa pemasangan kamera,
> bukan untuk terbang.

---

### `dbl_gng_cpu_node` (paket `gng_node`) — ✅ terverifikasi

**Ini front-end persepsi yang benar-benar diterbangkan.** README versi lama
mendokumentasikan `dbl_gng_node` (jalur GPU) di tempat ini lengkap dengan topic
`/plane` dan `/outlier` — keduanya keliru. Node yang dijalankan `waypoint_node`
adalah versi CPU, dan seluruh topic-nya berakhiran `_cpu`.

Implementasi **DBL-GNG** (*Double / Batch-Learning Growing Neural Gas*) dengan
NumPy dan `ThreadPoolExecutor`. Seluruh cloud diproses sebagai satu operasi
matriks per tick, bukan satu sampel per iterasi seperti GNG klasik. Node datar
ditentukan dari PCA lokal: `planarity = λ_min / Σλ < 1e-4` **dan** normalnya
dalam 10° dari sumbu `(1, 0, 0)`.

> Sumbu `(1, 0, 0)` bukan salah ketik. Di frame `camera_link` kamera menunduk
> 90°, sehingga **x adalah sumbu kedalaman** dan tanah terbentang di bidang
> **y–z**. Ini juga alasan `landing_circle` memakai `plane_axes=yz`.

| Arah | Topic | Tipe |
|------|-------|------|
| Subscribe | `/depth_camera/points` | `sensor_msgs/PointCloud2` |
| Publish | `/plane_cpu` | `sensor_msgs/PointCloud2` |
| Publish | `/outlier_cpu` | `sensor_msgs/PointCloud2` (diwarnai merah) |
| Publish | `/graph_markers_cpu` | `visualization_msgs/Marker` |
| Publish | `/segmentation_stats_cpu` | `std_msgs/String` (JSON) |

Parameter: `pointcloud_topic` (baku `/depth_camera/points`), `voxel_leaf`
(baku 0.15), `use_sim_time`.

**Nama node berbeda dari nama executable.** Lewat `ros2 run`, nama node adalah
`dbl_gng_cpu`. Launch file menamainya dengan sengaja sama (`name="dbl_gng_cpu"`)
karena penjaga duplikat di `waypoint_node` mencocokkan **nama node**, bukan nama
proses.

> ⚠️ **QoS.** Node ini berlangganan dengan QoS **RELIABLE** (baku rclpy),
> sedangkan `plane_segmentation_ransac` dan `landing_circle` memakai
> `SensorDataQoS` (**BEST_EFFORT**). Penerbit BEST_EFFORT **tidak akan terbaca**
> oleh node ini — ROS akan mencetak `incompatible QoS ... RELIABILITY` lalu diam
> selamanya. `ros_gz_bridge` menerbitkan RELIABLE sehingga di simulasi hal ini
> tidak muncul, tapi ini asimetri nyata antara kedua front-end yang dibandingkan.

<details>
<summary>Keluaran terukur pada cloud sintetis 14.641 titik</summary>

```json
{"source": "gng", "computation_time_ms": 776.24, "latency_ms": null,
 "input_points": 14641, "valid_points": 14641, "valid_percentage": 100.0,
 "downsampled_points": 6757, "plane_size": 5626, "outlier_size": 1131,
 "voxel_leaf_m": 0.15}
```

`latency_ms` bernilai `null` karena diukur tanpa `use_sim_time`. Itu memang
disengaja: stempel cloud berasal dari jam Gazebo, dan menguranginya dengan jam
dinding menghasilkan angka yang tampak masuk akal tapi salah.
</details>

---

### `dbl_gng_node` (paket `gng_node`) — ❌ tidak tersedia di mesin rujukan

Varian GPU dari algoritma yang sama, memakai `torch`. **Tidak dapat dijalankan
di mesin rujukan** dan tidak diverifikasi:

```
File ".../gng_node/dbl_gng_node.py", line 4, in <module>
    import torch
ModuleNotFoundError: No module named 'torch'
[ros2run]: Process exited with failure 1
```

Node ini gagal saat impor, sebelum sempat membuat node ROS apa pun. Perbedaan
penting dari versi CPU:

- **Tidak punya tahap voxel downsample sama sekali.** Karena itu beban masukannya
  tidak sebanding, dan **perbandingan yang sah hanya `gng_cpu` lawan `ransac`**.
- Topic-nya tanpa akhiran: `/plane`, `/outlier`, `/graph_markers`,
  `/segmentation_stats` — **bertabrakan dengan topic RANSAC**.
- Statistiknya berupa teks bebas, bukan JSON, sehingga `logger_stats` tidak bisa
  mengurainya.

Jangan menjalankannya untuk eksperimen tanpa memasang `torch` lebih dulu dan
menyetarakan tahap downsample-nya.

---

### `plane_segmentation_ransac` (paket `segmentation_node`) — ✅ terverifikasi

Pembanding untuk GNG: segmentasi bidang PCL dengan `SACMODEL_NORMAL_PLANE`.
Berjalan di thread terpisah dan **membuang frame** selagi frame sebelumnya masih
diproses, sehingga node tidak pernah menumpuk antrean.

| Arah | Topic | Tipe |
|------|-------|------|
| Subscribe | `input_topic` (**wajib di-override**) | `sensor_msgs/PointCloud2` |
| Publish | `/plane` | `sensor_msgs/PointCloud2` |
| Publish | `/outlier` | `sensor_msgs/PointCloud2` (diwarnai merah) |
| Publish | `/segmentation_stats` | `std_msgs/String` (JSON) |

> ⚠️ Nilai baku `input_topic` adalah `/zed/zed_node/point_cloud/cloud_registered`
> — sisa dari kamera ZED yang **tidak pernah ada di simulasi**. Node yang
> dijalankan tanpa override akan hidup, sehat, dan diam selamanya. Selalu berikan
> `-p input_topic:=/depth_camera/points`.

| Parameter | Baku | Keterangan |
|---|---|---|
| `input_topic` | topic ZED | wajib di-override |
| `z_min` / `z_max` | −5.0 / 5.0 | PassThrough sumbu z; launch mematikannya (±1000) |
| `leaf_size` | 0.07 | ukuran voxel; launch memetakan `voxel_leaf` ke sini |
| `max_iterations` | 650 | iterasi RANSAC |
| `k_search` | 30 | tetangga untuk estimasi normal |
| `normal_dist_weight` | 0.17 | bobot jarak normal |
| `distance_threshold` | 0.097 | ambang inlier (m) |

---

### Perbandingan terukur GNG vs RANSAC pada input identik

Diukur pada mesin rujukan dengan cloud sintetis **14.641 titik** yang sama
persis, `voxel_leaf` = `leaf_size` = 0.15 untuk keduanya:

| Besaran | `dbl_gng_cpu_node` | `plane_segmentation_ransac` |
|---|---|---|
| `input_points` | 14.641 | 14.641 |
| `downsampled_points` | 6.757 | 6.762 |
| `plane_size` | 5.626 | 6.302 |
| `outlier_size` | 1.131 | 460 |
| `computation_time_ms` | 375 – 776 | 39 – 55 |

Jumlah titik setelah downsample praktis sama (6.757 vs 6.762) — inilah bukti
bahwa penyetaraan input lewat satu argumen `voxel_leaf` benar-benar bekerja.
RANSAC sekitar **10–20× lebih cepat**, tetapi menandai jauh lebih sedikit titik
sebagai rintangan pada rintangan berbentuk kubah yang sama.

> Angka ini berasal dari cloud sintetis di darat, **bukan** dari penerbangan.
> Angka penerbangan yang sah tetap harus diambil lewat `logger_stats` pada dua
> penerbangan bergantian.

---

### `landing_circle` (paket `segmentation_node`) — ✅ terverifikasi
Menganalisis bidang datar hasil segmentasi dan mendeteksi area lingkaran yang cukup besar dan aman untuk pendaratan. Kandidat yang ditemukan dikumpulkan di `LandingRegistry` (frame `map`), kandidat yang terhalang objek bergerak ditandai `blocked` dan tidak boleh dipilih, lalu yang **skornya tertinggi** diterbitkan sebagai TF **dinamis** `map` → `safety_point`.

TF ini hanya diterbitkan **selama ada kandidat terpilih**. Kalau registry kosong atau semua kandidatnya terhalang, siarannya berhenti — dan itulah sinyal "target hilang" yang dibaca `waypoint_node`.

#### Bagaimana pemenangnya dipilih

Kandidat tidak diperingkat hanya dari jaraknya. Petak tanah terdekat belum
tentu petak yang paling layak didarati, dan sepuluh detik yang dihabiskan
drone untuk mengumpulkan kandidat tidak ada gunanya kalau yang dibandingkan
cuma jarak.

```
dist_term  = 1 − jarak_mendatar / select_radius_m      (diklem ke [0, 1])
rough_term = 1 − roughness_m    / rough_max_m          (diklem ke [0, 1])
skor       = score_w_dist × dist_term + score_w_rough × rough_term
```

`roughness_m` adalah **RMS jarak titik-titik cakram ke bidang paling pas**
(residual PCA), dirata-ratakan lintas frame. Ia invarian terhadap kemiringan —
lereng landai yang mulus terbaca mulus — dan butuh minimal empat titik, karena
tiga titik menentukan bidang secara persis sehingga residualnya selalu nol.
Kandidat yang belum pernah terukur mendapat `rough_term` **nol**, bukan nilai
penuh: yang belum terbukti tidak boleh mengalahkan yang sudah terbukti rata.

Terhalang, di luar radius, dan baru saja ditolak konsumen tetap **batas keras**
— kandidat yang gagal salah satunya tidak ikut dinilai sama sekali. Skor hanya
mengurutkan yang sudah lolos.

**Keputusannya diambil setelah mengumpulkan, bukan di frame pertama.** Selama
`commit_after_s` detik pertama seleksi bebas mengikuti skor tertinggi seiring
registry terisi; begitu gerbang commit terbuka dan TF terbit, target dikunci
(`sticky_target`) dan hanya lepas kalau terhalang, ditolak, atau keluar radius.
Sticky sejak frame pertama pernah menjadi bug: target terkunci pada kandidat
yang kebetulan terlihat lebih dulu, dan jendela pengumpulan berikutnya mengisi
daftar yang keputusannya sudah diambil.

#### Pencarian berhenti setelah commit

`freeze_after_commit` (default **true**). Begitu keputusan diumumkan, seluruh
pipa pencarian — grid, fill ratio, peta obstacle, distance transform, NMS,
centroid, pembaruan registry — **berhenti**. Yang tetap jalan hanyalah
pemeriksaan **satu** cakram terkunci terhadap `/outlier_cpu`, dan penerbitan
TF-nya.

Alasannya bukan sekadar hemat: seleksi sticky tidak akan berpindah ke kandidat
yang ditemukan sesudah commit, jadi mengumpulkannya sia-sia — dan lebih buruk,
kandidat itu dikumpulkan dari ketinggian yang terus berubah lalu disimpan
seolah setara dengan yang dikumpulkan saat hover.

Diukur A/B, dua penerbangan di waypoint yang sama, 128 lawan 129 frame sesudah
commit:

| sesudah commit | `freeze_after_commit:=true` | `:=false` |
|---|---|---|
| jumlah kandidat | **14 → 14** | 14 → **42** |
| `computation_time_ms` median | 8,46 | 9,69 |
| p90 | 19,50 | 23,36 |
| galat pendaratan | 0,060 m | 0,006 m |

**Penghematan CPU-nya kecil: 13% median, 17% p90.** Pipa pencarian itu sendiri
ternyata murah; yang mahal adalah memproyeksikan setiap titik ke `plane_frame`,
dan itu tetap dijalankan saat beku supaya cakram hijau tidak hilang dari RViz.
Manfaat sebenarnya bukan kecepatan melainkan registry yang berhenti membengkak
oleh kandidat yang tidak akan pernah dipakai.

Baris log menandainya `COMMITTED/NO-SEARCH`, dan kolom `search_frozen` di
`<run>_landing.csv` merekamnya per frame.

**Saat target dilepas, SELURUH registry dibuang** dan pengumpulan dimulai lagi
dari nol. Kandidat yang tersisa saat itu dinilai sebelum drone menukik, dari
ketinggian yang sama sekali berbeda; melompat ke salah satunya berarti
mendarat di tempat yang penilaiannya sudah usang. Konsekuensinya tiap halangan
menambah `commit_after_s` detik terbang — itu harga yang disengaja.

Rangkaian pemulihannya, tanpa satu pun perubahan di node misi: TF berhenti →
misi melihat TF basi setelah `target_timeout_s` → `Aborting DESCEND: no fresh
safety_point TF` → SCAN menyiarkan pose waypoint sehingga drone naik lagi →
`landing_circle` sedang mengumpulkan → TF baru muncul → misi mengunci.

Jeda pemulihannya sekitar 5 detik (`target_block_s` 1,5 + `target_timeout_s`
3,5), atau ~2 m penurunan pada `descend_speed_ms` 0,4. Karena `abort_floor_m`
4,0 m, itu tetap berakhir di udara.

| Parameter | Default | Arti |
|---|---|---|
| `score_w_dist` | 0.5 | bobot suku jarak |
| `score_w_rough` | 0.5 | bobot suku roughness |
| `rough_max_m` | 0.10 | roughness yang skornya sudah nol |
| `score_dist_ref_m` | 10.0 | penormalisasi jarak bila `select_radius_m` = 0 |
| `score_hysteresis` | 0.05 | margin skor sebelum target berpindah (hanya jalur non-sticky) |
| `roughness_min_points` | 4 | titik minimum agar residual berarti |
| `freeze_after_commit` | true | hentikan pencarian setelah keputusan diumumkan |

Semua kandidat beserta `roughness_m` dan `score`-nya diterbitkan ke
`/landing_circle_stats`, dan skornya juga muncul sebagai teks di atas tiap
cakram pada `/landing_candidates` — supaya pertanyaan "kenapa titik itu yang
menang" bisa dijawab dari data, bukan ditebak.

| Arah | Topic | Tipe |
|------|-------|------|
| Subscribe | `input_topic` (baku `/plane`, produksi `/plane_cpu`) | `sensor_msgs/PointCloud2` |
| Subscribe | `outlier_topic` (baku `/outlier_cpu`) | `sensor_msgs/PointCloud2` |
| Publish | `/safe_circle` | `sensor_msgs/PointCloud2` |
| Publish | `/safe_circle_center_coords` | `geometry_msgs/PointStamped` |
| Publish | `/landing_candidates` | `visualization_msgs/MarkerArray` |
| Publish | `/landing_circle_stats` | `std_msgs/String` (JSON) |
| TF Publish | `map` → `safety_point` | **Dynamic TF** |

> Nilai baku `input_topic` adalah `/plane` (keluaran RANSAC). Untuk jalur
> produksi GNG **wajib** diberikan `-p input_topic:=/plane_cpu`; inilah yang
> dilakukan `waypoint_node` lewat `perception_commands`.

#### ✅ Bukti verifikasi rantai penuh

Diuji dengan rantai `cloud sintetis → dbl_gng_cpu_node → landing_circle` dan TF
statis pada ketinggian 11 m:

| Yang diperiksa | Hasil terukur |
|---|---|
| Titik bidang yang masuk | 5.577 |
| `registry_size` / `num_candidates` | 26 / 26 |
| `blocked_candidates` | 9 |
| `committed` setelah | `collect_elapsed_s` = **10,02 s** (gerbang commit 10 s) |
| `search_frozen` setelah commit | `true` (`freeze_after_commit` bekerja) |
| Skor terpilih | 0,8177 |
| `computation_time_ms` | 26,5 |
| Roughness kandidat | 0,008 – 0,012 m |

TF `map → safety_point` terbaca `[-2.106, -1.696, 0.242]`, **sama persis**
dengan `selected_x/y/z` di `/landing_circle_stats`. TF baru mulai disiarkan
setelah gerbang commit terlewati, bukan sejak kandidat pertama muncul.

---

### `waypoint_node` (paket `offboard_mission`) — ✅ terverifikasi

Misi otonom penuh sebagai satu state machine:

```
WAIT_POSITION → ARMING → TAKEOFF → GOTO ─┐
                                          ▼
                    ┌──────────────────► SCAN ──(titik mengendap)──► APPROACH
                    │                      │                            │
                    │              (timeout / gagal)              (target hilang
                    │                      ▼                       / berpindah)
                    │                    HOLD                           │
                    └───────── (batal, naik lagi) ───────────────── DESCEND
                                                                        │
                                                                        ▼
                                                                LANDING → LANDED
```

Setibanya di waypoint, node menyalakan `dbl_gng_cpu_node` + `landing_circle`,
lalu menunggu TF `map → safety_point` berhenti bergerak sebelum menguncinya
sebagai target. Kalau target hilang atau `landing_circle` berpindah kandidat di
tengah pendekatan, misi naik kembali dan memindai ulang — kecuali bila drone
sudah lebih rendah dari `abort_floor_m` (**4,0 m**, nilai baku terverifikasi) di atas permukaan, karena di
ketinggian itu kamera yang menghadap ke bawah memang kehilangan bidangnya dan
membatalkan hanya akan membuat loop yang tak pernah mendarat.

Sentuhan terakhir diserahkan ke PX4 lewat `VEHICLE_CMD_NAV_LAND`; sejak saat
itu node **berhenti mengirim setpoint** supaya tidak berebut kendali dengan
pengendali pendaratan PX4.

| Arah | Topic / TF | Tipe |
|------|-----------|------|
| Subscribe | `/fmu/out/vehicle_local_position(_v1)` | `px4_msgs/VehicleLocalPosition` |
| Subscribe | `/fmu/out/vehicle_status(_v1)` | `px4_msgs/VehicleStatus` |
| TF Subscribe | `map` → `safety_point` | target pendaratan |
| TF Publish | `map` → `scan_center` | acuan seleksi (dynamic, 20 Hz) |
| TF Publish | `map` → `reject_point` | titik yang baru ditinggalkan (dynamic) |
| Publish | `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` |
| Publish | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` |
| Publish | `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` |
| Publish | `/mission_events` | `std_msgs/String` (JSON, TRANSIENT_LOCAL) |

> ⚠️ **Nama node ≠ nama executable.** `ros2 run offboard_mission waypoint_node`
> mendaftarkan node bernama **`waypoint_mission`**. Gunakan nama itu untuk
> `ros2 node info` dan `ros2 param get`.

#### ✅ Bukti verifikasi

**Dengan PX4 SITL:** misi penuh dijalankan sampai selesai — `WAIT_POSITION →
ARMING → TAKEOFF → GOTO → SCAN → APPROACH → DESCEND → SCAN → APPROACH →
DESCEND → LANDING → LANDED` — dan mendarat **0,07 m** dari titik pilihannya.
Rinciannya di [Hasil penerbangan terverifikasi](#hasil-penerbangan-terverifikasi).

**Tanpa PX4 sama sekali:** node berperilaku aman — ia menunggu, lalu
**menolak melakukan arming**.

```
[INFO]  WaypointMission: target map/ENU (-0.17, 8.60, 11.00)
        -> PX4 NED (8.60, -0.17, -11.00) | takeoff_alt=2.50m | tol=0.50m
[ERROR] No valid VehicleLocalPosition after 15.0s. NOT arming.
```

Konversi ENU → NED terbukti benar: `(x, y, z)` ENU menjadi `(y, x, −z)` NED.
Node mengekspos **38 parameter**; nilai baku yang diperiksa langsung:

| Parameter | Nilai baku terverifikasi |
|---|---|
| `takeoff_alt` | 2.5 |
| `scan_radius_m` | 8.0 |
| `scan_collect_s` | 10.0 |
| `abort_floor_m` | **4.0** |
| `target_timeout_s` | 3.5 |
| `enable_landing` | `true` |
| `manage_perception` | `true` |

Dua frame yang diterbitkan node ini pantas dijelaskan, karena keduanya lahir
dari kegagalan uji terbang yang nyata:

**`scan_center`** duduk di waypoint, dan `landing_circle` dijalankan dengan
`base_frame:=scan_center`. Tanpa ini acuan seleksinya adalah **drone**, dan
karena dronenya bergerak, "kandidat terdekat" berubah terus tanpa ada apa pun
yang terjadi pada targetnya. Node misi membaca perpindahan TF itu sebagai
"target terhalang" dan membatalkan pendaratan tiga kali berturut-turut.

**`reject_point`** duduk di titik yang baru saja ditinggalkan node misi.
Seleksi `landing_circle` bersifat sticky — itulah yang membuat TF berhenti
bergoyang — tapi berarti tidak ada apa pun yang membuatnya melepas titik yang
sudah dicoba dan gagal. Misi akan naik, memindai ulang, lalu mengunci titik
yang sama persis. Registry tahu kandidat mana yang *terhalang*; hanya node
misi yang tahu kandidat mana yang *sudah dicoba*. Frame ini jalur untuk
menyampaikannya, dan penolakannya kedaluwarsa setelah `reject_hold_s`.

> Jalankan **hanya satu** node offboard pada satu waktu. `keyboard_offboard_node`
> dan `waypoint_node` sama-sama menyiarkan setpoint ke drone yang sama, dan
> menjalankan keduanya bersamaan membuat drone bergerak tidak menentu.

---

### `path_trail_node` (paket `path_trail`) — ✅ terverifikasi

Menyampel TF `map → base_link` pada 10 Hz dan menumpuknya menjadi
`nav_msgs/Path` di `/drone_path`, yang digambar RViz2 sebagai garis jejak
lintasan drone. Dinyalakan otomatis oleh launch file; matikan dengan
`path_trail:=false`.

**Node ini untuk dilihat, bukan untuk dianalisis.** Lintasan sebagai *data*
direkam terpisah oleh `logger_stats` ke `<run>_trajectory.csv`, dengan sengaja
tidak di sini — mematikan gambarnya tidak boleh ikut menghapus angkanya.

| Arah | Topic | Tipe |
|------|-------|------|
| Subscribe | `/tf` (`map` → `base_link`) | TF |
| Publish | `/drone_path` | `nav_msgs/Path` |

**✅ Terverifikasi.** Dengan TF dinamis dari `drone_kinematic`, `/drone_path`
terbit pada **tepat 10,000 Hz** dan menumpuk 256 pose dalam ~26 detik.

> Dengan TF **statis** node ini sengaja tidak menerbitkan apa pun: stempel waktu
> TF statis tidak pernah berubah, dan sampel berstempel sama memang dilewati.
> Itu bukan kerusakan, melainkan proteksi "penerbit TF macet" yang bekerja.
> Untuk mengujinya Anda butuh TF dinamis, bukan `static_transform_publisher`.

Dua parameter yang jarang perlu disentuh tapi ada alasannya:

- `max_poses` (3000, = 5 menit pada 10 Hz). Seluruh Path diterbitkan **ulang**
  tiap tick, jadi panjangnya bukan soal memori melainkan bandwidth: tanpa
  batas, sepuluh menit terbang menjadi 6.000 pose yang dikirim ulang sepuluh
  kali per detik. Beban itu jatuh ke mesin yang sedang diukur waktu
  komputasinya.
- `sample_hz` (10.0). Sampel dengan stempel TF yang sama dilewati, sehingga
  penerbit TF yang berhenti tidak terlihat seperti drone yang melayang diam.

---

### `logger_stats` (paket `logger_stats`, nama node `stats_logger`) — ✅ terverifikasi

Perekam data eksperimen. **Menulis satu baris CSV per pesan yang diterima** —
tanpa deduplikasi, tanpa peringkasan. README versi lama mengklaim node ini
"hanya menulis baris baru ketika ada koordinat baru" dan menamai dua berkas yang
tidak pernah dibuatnya; keduanya keliru.

Node hanya melakukan `json.loads` lalu meratakan isinya. Semua penerbit
statistik di workspace ini menerbitkan **JSON** justru karena versi lama memakai
pengurai regex atas teks bebas yang gagal diam-diam dan menghasilkan CSV kosong
berbulan-bulan. Kunci yang tidak dikenal tidak dibuang, melainkan masuk ke kolom
`extra`.

| Arah | Topic / TF | Tipe |
|------|-----------|------|
| Subscribe | `/segmentation_stats_cpu` | `std_msgs/String` (GNG) |
| Subscribe | `/segmentation_stats` | `std_msgs/String` (RANSAC) |
| Subscribe | `/landing_circle_stats` | `std_msgs/String` |
| Subscribe | `/mission_events` | `std_msgs/String` (TRANSIENT_LOCAL) |
| Subscribe | `/safe_circle_center_coords` | `geometry_msgs/PointStamped` |
| TF Subscribe | `map` → `base_link` | disampel 10 Hz untuk lintasan |

| Parameter | Baku | Keterangan |
|---|---|---|
| `output_dir` | `~/ros2_logs` | direktori keluaran |
| **`run_id`** | *(kosong)* | **wajib** — node menolak start tanpa ini |
| `map_frame` / `base_frame` | `map` / `base_link` | sumber lintasan |
| `trajectory_hz` | 10.0 | laju sampel lintasan |

> ⚠️ `run_id` wajib, dan node sengaja gagal keras tanpa itu:
> ```
> RuntimeError: Parameter 'run_id' wajib diisi -- jalankan dengan
> --ros-args -p run_id:=gng (atau ransac). Tanpa itu data dua
> penerbangan bercampur dan tidak bisa dipisahkan lagi.
> ```
> Penerbangan GNG dan RANSAC dijalankan bergantian, satu penerbangan
> masing-masing. Data tanpa label tidak bisa dipulihkan lagi setelahnya.

**Berkas keluaran sesungguhnya** — pola `<run_id>_<stamp>_<jenis>.csv`.
Hasil satu perekaman uji 30 detik:

| Berkas | Baris | Kolom |
|---|---|---|
| `<run>_gng.csv` | 58 | `run_id,recv_ns,source,stamp_ns,computation_time_ms,latency_ms,input_points,valid_points,valid_percentage,downsampled_points,plane_size,outlier_size,voxel_leaf_m,extra` |
| `<run>_ransac.csv` | 0 | kolom sama; kosong bila RANSAC tidak dijalankan |
| `<run>_landing.csv` | 74 | `...,safe_size,registry_size,num_candidates,blocked_candidates,committed,collect_elapsed_s,...` |
| `<run>_candidates.csv` | 778 | `run_id,recv_ns,stamp_ns,candidate_index,x,y,z,hits,blocked_s,clearance_m,fill_ratio,roughness_m,rough_n,score,selectable,extra` |
| `<run>_center.csv` | 46 | `run_id,recv_ns,stamp_ns,frame_id,x,y,z,extra` |
| `<run>_trajectory.csv` | 294 | `run_id,recv_ns,stamp_ns,frame_id,x,y,z,yaw_deg,extra` |

`<run>_candidates.csv` berformat panjang: **satu baris per kandidat per frame**,
bukan satu baris per frame. Selain itu setiap jenis `kind` pada `/mission_events`
menghasilkan berkasnya sendiri (`<run>_mission_transition.csv`,
`<run>_mission_commit.csv`, `<run>_mission_summary.csv`).

> Jalankan dengan `use_sim_time:=true`. Tanpa itu kolom `latency_ms` sengaja
> dikosongkan, karena stempel cloud memakai jam Gazebo.

---

## Node jalur lama dan diagnostik

Executable berikut ikut terbangun tetapi **bukan bagian dari jalur pendaratan**.
Semuanya diverifikasi berjalan, dan hasilnya menjelaskan mengapa tidak dipakai.

### `cylinder_crop` — 🗑️ sudah dihapus

Node ini **tidak ada lagi**. Sumber, target CMake, dan berkas terpasangnya
sudah dibuang dari paket `segmentation_node`.

Alasannya sumbu. Node itu memotong cloud menjadi silinder pada bidang **x–y**,
padahal di `camera_link` **x adalah kedalaman** dan bidang tanah terbentang di
**y–z** — itulah sebabnya `landing_circle` memakai `plane_axes=yz`. Pada
ketinggian pindai hampir semua titik punya x besar sehingga jatuh di luar
silinder berjari-jari 2,5 m. Lognya sendiri mengakuinya:
`axis=Z (depth), filter bidang X-Y`.

**Terukur sebelum dihapus:** cloud uji **14.641 titik masuk → 0 titik keluar**,
dan node tetap menerbitkan cloud kosong pada 5 Hz sehingga *terlihat* sehat.
Pengukuran lapangan sebelumnya mencatat 19.200 → 4 titik pada ketinggian pindai
11 m, setelah itu RANSAC di hilirnya berhenti menerbitkan apa pun.

`depth_bridge_launch.py` sudah lebih dulu berhenti memakainya.
`ransac_pipeline.launch.py` kini juga tidak lagi memakainya: launch itu memberi
RANSAC cloud mentah `/depth_camera/points` yang sama dengan yang diterima GNG,
dengan PassThrough dimatikan dan `leaf_size` 0.15 — dan setelah perubahan itu
launch tersebut **terbukti menerbitkan `/plane` berisi 6.302 titik**, padahal
rantai lamanya menerbitkan nol.

### `cloud_size_node` (paket `segmentation_node`) — diagnostik saja

Menghitung titik hingga di sebuah cloud dan mencatat persentasenya. Berlangganan
topic ZED **yang tertulis mati di kode** —
`/zed/zed_node/point_cloud/cloud_registered` — tanpa parameter apa pun, sehingga
di simulasi node ini hidup dan diam selamanya. Terverifikasi lewat
`ros2 node info /cloud_size_node`.

### `plane_segmentation_gng` dan `ddgng_node` (paket `segmentation_node`) — ⚠️ bertabrakan

Dua implementasi GNG dalam C++ dari generasi sebelum jalur Python. **Keduanya
mendaftarkan nama node yang sama, `plane_segmentation_gng`.** Dijalankan
bersamaan, ROS 2 memperingatkan:

```
WARNING: Be aware that there are nodes in the graph that share an exact name,
which can have unintended side effects.
/plane_segmentation_gng
/plane_segmentation_gng
```

Header `include/segmentation_node/ddgng.hpp` tidak di-`#include` oleh berkas mana
pun — `ddgng_node.cpp` mendefinisikan kelasnya sendiri. Perlakukan seluruh jalur
ini sebagai kode mati sampai dibersihkan.

### `keyboard_offboard_node` (paket `keyboard_offboard_control`) — ✅ terverifikasi

Penerbangan manual untuk debugging, memakai helper yang sama dengan
`waypoint_node` (`px4_offboard_lib`). Pembaca papan ketik memakai termios mentah
pada thread daemon; keluar dengan Ctrl-C memulihkan terminal dan melakukan
auto-disarm.

```
HOLD to move: W/S fwd/back  A/D strafe  R/F up/down  Q/E yaw  Space=hover
PRESS once:   I=arm  K=disarm  T=takeoff  L=land  O=offboard  H=hold
```

| Arah | Topic | Tipe |
|------|-------|------|
| Subscribe | `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` |
| Subscribe | `/fmu/out/vehicle_status_v1` | `px4_msgs/VehicleStatus` |
| Subscribe | `/fmu/out/vehicle_command_ack` | `px4_msgs/VehicleCommandAck` |
| Publish | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` |
| Publish | `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` |
| Publish | `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` |

Parameter: `linear_speed` (0.8), `yaw_speed` (0.5), `takeoff_alt` (2.5),
`offboard_rate` (20).

> Jalankan **hanya satu** node offboard pada satu waktu — node ini atau
> `waypoint_node`, tidak pernah keduanya.

---

## Struktur Direktori

```
safety-landing-point-detection-for-drone/
├── src/
│   ├── gz_bridge_ros2/           # drone_kinematic (penerbit TF) + launch file
│   ├── gng_node/                 # segmentasi GNG  -> /plane_cpu, /outlier_cpu
│   ├── segmentation_node/        # RANSAC, landing_circle
│   ├── offboard_mission/         # waypoint_node — mesin state misi
│   ├── px4_offboard_lib/         # helper murni: konversi frame, deteksi tiba
│   ├── keyboard_offboard_control/# kendali manual, untuk debugging
│   ├── path_trail/               # jejak lintasan -> /drone_path (RViz)
│   ├── logger_stats/             # perekam metrik -> CSV
│   └── px4_msgs/                 # definisi pesan PX4
├── scripts/                      # alat pelaporan (bukan paket ROS)
│   ├── metrics_io.py             # pembacaan CSV + palet, dipakai bersama
│   ├── analyze_metrics.py        # CSV -> tabel ringkasan
│   ├── plot_flight.py            # CSV -> PNG lintasan + ketinggian
│   ├── plot_stats.py             # CSV -> PNG kolom apa pun
│   ├── plot_trajectory3d.py      # CSV -> PNG lintasan 3D
│   ├── test_metrics_io.py        # python3 -m pytest scripts/
│   ├── test_plot_stats.py
│   └── test_plot_trajectory3d.py
├── px4_assets/                   # aset PX4 yang dimodifikasi (salin ke PX4 Anda)
│   ├── install.sh                # pemasang; --dry-run untuk memeriksa saja
│   ├── Tools/simulation/gz/      # model x500_depth & OakD-Lite, world rubicon & forest_x4
│   └── ROMFS/.../airframes/      # 4002_gz_x500_depth (pose spawn per world)
├── install/  build/  log/        # hasil colcon — tidak dilacak git
└── ~/ros2_logs/                  # keluaran perekam metrik
```

`scripts/` sengaja **bukan** paket ROS: isinya alat pelaporan yang dijalankan
sesekali di meja, bukan kode yang ikut terbang. Ia tetap di dalam repo karena
versi sebelumnya hidup di direktori sementara dan terhapus bersama sesinya,
membawa serta satu-satunya cara membaca CSV itu menjadi tabel.

---

## Cara Mereproduksi Verifikasi

Verifikasi tingkat node di dokumen ini tidak memerlukan Gazebo maupun PX4. Yang
dibutuhkan node persepsi hanya dua: **point cloud** dan **TF**.

**1. Point cloud sintetis.** Terbitkan `sensor_msgs/PointCloud2` di frame
`camera_link` berisi bidang datar pada `x = ketinggian`, dengan `y` dan `z`
membentang mendatar. Geometrinya wajib begitu — di `camera_link` **x adalah
kedalaman**. Cloud yang dibangun di bidang x–y akan membuat seluruh pipeline
diam dan menghasilkan kesimpulan palsu.

Dua jebakan yang ditemui saat menyusun verifikasi ini, keduanya menghasilkan
"node tampak rusak" padahal harness-nya yang salah:

- **QoS.** Terbitkan dengan QoS **RELIABLE**. Penerbit BEST_EFFORT tidak akan
  terbaca `dbl_gng_cpu_node`, yang hanya mencetak satu peringatan
  `incompatible QoS` lalu diam.
- **Bentuk rintangan.** Rintangan berpuncak **datar** punya normal yang sama
  dengan tanah, sehingga digolongkan sebagai bidang dan `outlier_size` menjadi
  0. Pakai bentuk **kubah** agar normalnya menyimpang lebih dari 10°.

**2. Rantai TF.** Tiru keluaran `drone_kinematic`:

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 11 \
  --roll 0 --pitch 0 --yaw 0 --frame-id odom --child-frame-id base_link
ros2 run tf2_ros static_transform_publisher --x 0.22 --y 0.03 --z 0.242 \
  --roll 0 --pitch 1.5708 --yaw 0 --frame-id base_link --child-frame-id camera_link
```

TF statis cukup untuk `landing_circle`, tetapi **tidak** untuk `path_trail_node`
— node itu melewati sampel berstempel sama. Untuk mengujinya jalankan
`drone_kinematic` dengan penerbit `/fmu/out/vehicle_odometry` palsu.

**3. Periksa tiap node** dengan `ros2 node list`, `ros2 node info <nama>`,
`ros2 param list`, `ros2 topic hz`, dan `ros2 topic echo --once`.

> **Membunuh node uji.** `kill -INT` pada PID `ros2 run` **tidak** mematikan
> binary node-nya — pembungkusnya mati, nodenya tetap hidup dan mendaftar ulang
> di graf. Bunuh seluruh process group, persis seperti yang dilakukan
> `perception_supervisor.py`. Selalu periksa sisanya:
>
> ```bash
> pgrep -af "dbl_gng_cpu_node|landing_circle|keyboard_offboard_node|waypoint_node"
> ```

---

## Masalah yang Diketahui

Ditemukan saat verifikasi ini, **belum diperbaiki**:

| Masalah | Berkas | Dampak |
|---|---|---|
| Algoritma GNG tanpa test perilaku | `src/gng_node/test/` | objek utama penelitian tidak terlindungi regresi |
| Dua executable memakai satu nama node | `plane_segmentation_gng.cpp`, `ddgng_node.cpp` | tabrakan di graf ROS |
| Header `ddgng.hpp` tidak dipakai siapa pun | `include/segmentation_node/` | kode mati |
| `Ellipsis` harfiah di dalam list | `src/gng_node/gng_node/dbl_gng_node.py:34` | kode mati |
| `self.comp_ms` tidak diinisialisasi di `__init__` | `src/gng_node/gng_node/dbl_gng_node.py` | `AttributeError` bila init GNG gagal lebih dulu |
| Salinan struct parameter non-atomik | `plane_segmentation_ransac.cpp:190` | balapan dengan callback set-parameter |
| Asimetri QoS antar front-end | GNG RELIABLE vs RANSAC/landing_circle BEST_EFFORT | penerbit BEST_EFFORT membuat GNG senyap |
| PCL dan OpenMP tidak dideklarasikan | `src/segmentation_node/package.xml` | build bergantung pada dependensi transitif |
| `description` dan `license` masih `TODO` | `src/segmentation_node/package.xml` | metadata paket belum diisi |
| 26 kegagalan lint | `segmentation_node`, `gz_bridge_ros2`, `gng_node` | `colcon test` tidak pernah hijau bersih |
| Default `input_topic` menunjuk `/circle_cloud` | `ddgng_node.cpp`, `plane_segmentation_gng.cpp` | topik itu tak punya penerbit lagi sejak `cylinder_crop` dihapus; keduanya jalur lama dan memang wajib di-override |

**Sudah diperbaiki dalam pembaruan ini:** node `cylinder_crop` dihapus, dan
`ransac_pipeline.launch.py` yang bergantung padanya diperbaiki sehingga
berfungsi kembali.

---

## Troubleshooting

### Gazebo tidak terbuka / crash saat launch

**Jangan mulai dari variabel NVIDIA** — pada mesin tanpa GPU NVIDIA, meng-export
`__NV_PRIME_RENDER_OFFLOAD` dan `__GLX_VENDOR_LIBRARY_NAME=nvidia` justru bisa
membuat `gz sim` gagal membuat konteks GL. Urutan yang benar:

1. Coba polos dulu: `gz sim -g`.
2. Kalau keluhannya versi OpenGL, paksa lewat Mesa:
   `MESA_GL_VERSION_OVERRIDE=4.5 gz sim -g`.
3. Lewati GUI sama sekali — jalankan hanya server (`gz sim -r -s ...`). Seluruh
   pipeline persepsi dan misi tidak membutuhkan GUI.
4. Baru bila Anda **memang** punya GPU NVIDIA dengan PRIME offload, pakai
   variabel `__NV_*`.

### `Preflight Fail: Accel Sensor 0 missing` / `Arming denied`

Gejala lengkapnya biasanya begini, muncul di prompt `pxh>` Terminal 2:

```
WARN [health_and_arming_checks] Preflight Fail: Accel Sensor 0 missing
WARN [health_and_arming_checks] Preflight Fail: barometer 0 missing
WARN [health_and_arming_checks] Preflight Fail: ekf2 missing data
WARN [health_and_arming_checks] Preflight Fail: Gyro Sensor 0 missing
WARN [health_and_arming_checks] Preflight Fail: Found 0 compass (required: 1)
WARN [commander] Arming denied: Resolve system health failures first
```

**Penyebabnya hampir selalu satu: `GZ_SIM_SERVER_CONFIG_PATH` tidak disetel di
terminal yang menjalankan `gz sim`** (Terminal 1). Tanpa itu Gazebo tidak memuat
sistem Imu, AirPressure, Magnetometer, NavSat, maupun Sensors, sehingga tidak
ada satu pun sensor yang menerbitkan data — dan PX4 menolak arming.

Ini **bukan** masalah sensor PX4, bukan parameter kalibrasi, dan bukan model SDF
yang rusak. Jangan buang waktu di `param set` atau di `model.sdf`.

Cara memastikannya — periksa proses `gz sim` yang sedang berjalan:

```bash
tr '\0' '\n' < /proc/$(pgrep -f "gz sim -r -s" | head -1)/environ | grep GZ_SIM_SERVER_CONFIG_PATH
```

Kalau tidak ada keluaran, itulah penyebabnya.

**Perbaikannya:** hentikan Terminal 1 **dan** Terminal 2, lalu jalankan ulang
Terminal 1 dengan variabelnya di-export lebih dulu:

```bash
export GZ_SIM_SERVER_CONFIG_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/server.config
gz sim -r -s ~/PX4-Autopilot/Tools/simulation/gz/worlds/rubicon.sdf
```

Menyetel variabel itu setelah `gz sim` berjalan tidak ada gunanya — server hanya
membaca konfigurasinya saat start. PX4 juga harus diulang, karena EKF2-nya sudah
menyerah menunggu data sensor.

### PX4 tidak terkoneksi ke Gazebo
Pastikan Gazebo server (Terminal 1) sudah berjalan sepenuhnya sebelum menjalankan SITL. Tambahkan delay jika perlu:
```bash
sleep 10 && make px4_sitl gz_x500_depth
```

### QGroundControl tidak terhubung ke PX4
Pastikan PX4 SITL sudah berjalan dan MicroXRCE-DDS Agent aktif. QGroundControl mendengarkan di UDP port 14550 secara default — tidak perlu konfigurasi tambahan untuk koneksi lokal.