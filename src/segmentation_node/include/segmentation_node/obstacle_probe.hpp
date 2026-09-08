// file: obstacle_probe.hpp
//
// Bukti POSITIF adanya penghalang di atas titik pendaratan.
//
// Percobaan sebelumnya mencari LUBANG di /plane_cpu — "apakah cakramnya
// kehilangan titik bidang?" — dan gagal di udara. Lubang punya dua arti yang
// tidak bisa dipisahkan: ada yang menghalangi, atau frame itu kebetulan tidak
// menyentuhnya. Terukur `Safe: 0/2/1/5` pada cakram yang sebenarnya bersih,
// menghasilkan enam salah alarm dan tiga pendaratan baik yang dibatalkan.
// Menaikkan ambang ketinggian tidak menolong: ketiga pemicuan terjadi pada
// 13,0-13,2 m, justru saat pandangan kamera paling luas.
//
// Penghalang yang berdiri di tanah BUKAN permukaan datar, jadi titik-titiknya
// tersaring keluar dari /plane_cpu dan masuk ke /outlier_cpu. Menghitung
// KEHADIRAN titik di sana membalik arah pertanyaannya, dan itulah yang membuat
// jalur ini kebal: data yang jarang bisa membuat titik hilang, tapi tidak bisa
// membuat titik muncul di tempat yang kosong.
//
// Dipisah dari node supaya bisa diuji tanpa ROS maupun PCL.

#pragma once

#include <Eigen/Core>

#include <vector>

// Jumlah titik yang berada dalam SILINDER TEGAK di atas `center`:
//
//   - jarak MENDATAR ke sumbu silinder <= radius_m, dan
//   - tinggi di atas center.z antara min_h dan max_h (dua-duanya inklusif).
//
// Mendatar dan tegak sengaja dipisah. `center` adalah pusat permukaan
// pendaratan, dan yang ditanyakan adalah "apakah ada sesuatu BERDIRI di
// atasnya" — bukan "apakah ada sesuatu di dekatnya dalam 3D". Radius 3D justru
// akan meloloskan penghalang yang tinggi, persis kebalikan dari yang dimau.
//
//   min_h  menyaring rumput dan derau permukaan. Tanpa ini setiap zona
//          berumput akan terbaca sebagai terhalang.
//   max_h  menyaring dahan atau wahana lain yang melintas jauh di atas —
//          ada di atas cakram, tapi bukan penghalang pendaratan.
//
// `pts` harus sudah berada di frame yang sama dengan `center`, yaitu frame
// yang sumbu z-nya benar-benar ke atas (`map`), bukan frame kamera.
inline int countPointsAbove(const std::vector<Eigen::Vector3f>& pts,
                            const Eigen::Vector3f& center,
                            float radius_m, float min_h, float max_h)
{
  const float r2 = radius_m * radius_m;

  int n = 0;
  for (const auto& p : pts) {
    const float h = p.z() - center.z();
    if (h < min_h || h > max_h) continue;

    const float dx = p.x() - center.x();
    const float dy = p.y() - center.y();
    if (dx * dx + dy * dy <= r2) ++n;
  }
  return n;
}
