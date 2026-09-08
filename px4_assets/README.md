# Aset simulasi PX4 yang dimodifikasi

Direktori ini memuat berkas-berkas PX4-Autopilot yang **sudah dimodifikasi** dan
dibutuhkan proyek ini. Berkas-berkas ini bukan milik PX4 bawaan, jadi harus
disalin ke checkout PX4 Anda sendiri setelah meng-clone PX4 dari repositori
resminya.

Susunan direktori sengaja **meniru pohon PX4-Autopilot**, sehingga penyalinannya
bisa dilakukan sekali jalan.

## Pemasangan

```bash
# 1. Clone PX4 dari repositori resminya
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot

# 2. Pasang aset yang dimodifikasi
./px4_assets/install.sh ~/PX4-Autopilot

# 3. Bangun PX4 SITL supaya airframe yang baru terbaca
cd ~/PX4-Autopilot && make px4_sitl
```

Tanpa argumen, skrip memakai `$PX4_SOURCE_DIR` bila diset, selain itu
`~/PX4-Autopilot`. Tambahkan `--dry-run` untuk melihat apa yang akan berubah
tanpa menulis apa pun.

Skrip menolak berjalan bila direktori tujuan tidak memuat `Makefile` dan
`boards/`, dan setiap berkas yang ditimpa dicadangkan sekali menjadi
`<nama>.orig`.

Menyalin manual juga bisa:

```bash
cp -r px4_assets/Tools px4_assets/ROMFS ~/PX4-Autopilot/
```

## Isi dan alasannya

| Berkas | Perubahan | Mengapa penting |
|---|---|---|
| `Tools/simulation/gz/models/x500_depth/model.sdf` | pose kamera → `0.22 0.03 0.242 0 **1.5708** 0` | `1.5708` rad = 90°, membuat kamera **menunduk**. Di PX4 bawaan kamera menghadap ke depan dan pipeline tidak akan pernah bekerja. Nilai ini juga di-hardcode di `drone_kinematic.cpp` sebagai TF `base_link → camera_link` |
| `Tools/simulation/gz/models/OakD-Lite/model.sdf` | depth **640×480 → 160×120**, far 19.1 → 20.0 m, rate 30 → 15 Hz, kamera RGB `IMX214` dimatikan | `160 × 120 = 19.200`, persis angka "19.200 titik" yang dipakai di seluruh dokumentasi dan komentar kode. Resolusi bawaan menghasilkan 307.200 titik — 16× lebih banyak — sehingga seluruh angka waktu komputasi dan kalibrasi grid tidak lagi berlaku |
| `Tools/simulation/gz/worlds/rubicon.sdf` | **berkas baru** | World yang dipakai di seluruh contoh perintah. Tidak ada di PX4 bawaan |
| `Tools/simulation/gz/worlds/forest_x4.sdf` | **berkas baru** | World uji tambahan. Berbeda dari `forest.sdf` bawaan PX4 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4002_gz_x500_depth` | pose spawn baku per world | `rubicon` memakai terrain, bukan bidang datar; heightmap-nya naik sampai 5 m sehingga spawn di titik asal menanam wahana di dalam tanah |

## Mengembalikan ke asli

Setiap berkas yang ditimpa punya cadangan `.orig` di sebelahnya. Untuk berkas
yang memang baru (`rubicon.sdf`, `forest_x4.sdf`), cukup hapus. Alternatifnya,
di dalam checkout PX4: `git checkout -- <berkas>`.
