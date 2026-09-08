// file: landing_score.hpp
//
// Nilai sebuah kandidat titik pendaratan: gabungan JARAK dan ROUGHNESS.
//
// Sebelumnya seleksi memakai satu kriteria saja — kandidat terdekat menang.
// Itu tidak cukup: petak tanah terdekat belum tentu petak yang paling layak
// didarati, dan sepuluh detik yang dihabiskan drone untuk mengumpulkan
// kandidat tidak ada gunanya kalau yang dibandingkan cuma jaraknya.
//
// Bentuknya sengaja dibuat sederhana dan bisa dijelaskan: dua suku, masing-
// masing dinormalisasi ke [0, 1], dijumlahkan dengan bobot yang jumlahnya 1.
// Akibatnya skor juga berada di [0, 1], dan kedua suku bisa dibandingkan
// langsung — "kandidat ini kalah karena jaraknya atau karena permukaannya?"
// bisa dijawab dengan melihat angkanya, bukan dengan menebak.
//
// Ini BUKAN tempat menaruh syarat keselamatan. Terhalang, di luar radius, dan
// baru saja ditolak konsumen tetap batas keras di LandingRegistry: kandidat
// yang gagal salah satunya tidak ikut dinilai sama sekali. Skor hanya
// mengurutkan yang sudah lolos.
//
// Bebas ROS, PCL, dan Eigen supaya bisa diuji sendiri.

#pragma once

#include <algorithm>

struct ScoreWeights {
  // Jumlahnya sebaiknya 1 supaya skor tetap di [0, 1] dan bisa dibaca sebagai
  // persentase kelayakan. Tidak dipaksakan — pemanggil boleh memiringkannya.
  float w_dist  = 0.5f;
  float w_rough = 0.5f;

  // Jarak yang membuat suku jaraknya nol. Dipasang dari select_radius_m bila
  // radius itu aktif, karena di sanalah batas kerasnya memang berada:
  // kandidat di tepi area pindai bernilai nol, di titik acuan bernilai satu.
  float dist_ref_m = 10.f;

  // Roughness yang membuat suku permukaannya nol. 0,10 m berarti permukaan
  // dengan simpangan RMS 10 cm sudah dianggap tidak menyumbang kelayakan apa
  // pun — masih boleh dipilih bila tidak ada yang lain, tapi kalah dari petak
  // yang lebih rata.
  float rough_max_m = 0.10f;
};

// 1 di titik acuan, turun linear, 0 pada `ref_m` dan seterusnya.
//
// Linear, bukan eksponensial: batas kerasnya sudah ada di select_radius_m,
// jadi suku ini hanya perlu mengurutkan di dalam batas itu — dan bentuk linear
// membuat "setengah jalan ke tepi = setengah nilai" benar apa adanya.
inline float distanceTerm(float d_m, float ref_m)
{
  if (!(ref_m > 0.f)) return 0.f;
  return std::clamp(1.f - d_m / ref_m, 0.f, 1.f);
}

// 1 untuk permukaan rata sempurna, 0 pada `rough_max_m` dan seterusnya.
//
// `rough_n` adalah jumlah sampel roughness yang pernah terkumpul untuk
// kandidat ini. Nol berarti BELUM PERNAH TERUKUR, dan itu memberi nilai 0 —
// pesimis dengan sengaja. Memberi nilai penuh pada yang belum terukur akan
// membuat kandidat yang cakramnya terlalu miskin titik justru menang atas
// kandidat yang sudah terbukti rata.
inline float roughnessTerm(float roughness_m, int rough_n, float rough_max_m)
{
  if (rough_n <= 0)          return 0.f;
  if (!(rough_max_m > 0.f))  return 0.f;
  return std::clamp(1.f - roughness_m / rough_max_m, 0.f, 1.f);
}

// `d_horizontal_m` MENDATAR, bukan 3D. Acuan seleksi (`scan_center`) berada di
// ketinggian terbang sedangkan kandidat berada di permukaan, jadi jarak 3D
// antara keduanya selalu memuat selisih ketinggian yang sama untuk semua
// kandidat. Untuk pengurutan itu masih monoton, tapi untuk suku yang
// dinormalisasi ia merusak: semua nilainya berdesakan di dekat satu angka dan
// tidak membedakan apa-apa. "Seberapa dekat ke tempat yang diminta dipindai"
// memang pertanyaan mendatar.
inline float landingScore(float d_horizontal_m,
                          float roughness_m,
                          int   rough_n,
                          const ScoreWeights& w)
{
  return w.w_dist  * distanceTerm(d_horizontal_m, w.dist_ref_m)
       + w.w_rough * roughnessTerm(roughness_m, rough_n, w.rough_max_m);
}
