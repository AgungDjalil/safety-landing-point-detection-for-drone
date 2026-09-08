// file: grid_coverage.hpp
//
// Satu pertanyaan, dipisah supaya bisa diuji tanpa ROS/PCL: berapa bagian dari
// sebuah cakram yang benar-benar TERSENTUH data pada frame ini.
//
// Ini berbeda dari "berapa bagian yang aman". Sebuah sel dihitung tercakup
// begitu ada satu titik pun di dalamnya, karena yang ditanyakan adalah apakah
// sensor melihat ke sana — bukan apakah yang dilihatnya layak didarati.
//
// Kegunaannya: memutuskan apakah kandidat lama yang tidak terdeteksi lagi
// pantas kena strike. Diukur saat uji terbang, /plane_cpu berayun 243-3378
// titik per frame dan cakram terpilih kosong sama sekali pada 26% frame.
// Tanpa uji cakupan, frame yang kebetulan tidak menyentuh sebuah zona terbaca
// sebagai "zona itu terhalang" — dan pada 0,79 Hz dengan block_after_s 3
// detik, tiga frame sial berturut-turut sudah cukup memvonis zona yang aman.
//
// Prinsipnya sama dengan penjaga bidang pandang yang sudah ada: jangan
// menghukum kandidat atas bukti yang memang tidak dimiliki. Tidak ada data
// bukan berarti ada penghalang — penghalang sungguhan justru MENGHASILKAN
// titik, jadi ia tetap membuat cakupan tinggi dan tetap terdeteksi.

#pragma once

#include <cstddef>
#include <utility>
#include <vector>

// Bagian sel cakram yang punya data pada frame ini, dalam [0, 1].
//
//   counts        jumlah titik per sel, baris-mayor, ukuran rows*cols.
//                 Pakai hitungan MENTAH — hasil close-gaps adalah tebakan,
//                 dan tebakan bukan bukti bahwa sensor melihat ke sana.
//   ix, iy        pusat cakram dalam koordinat sel.
//   disk_offsets  offset (dx, dy) yang membentuk cakram.
//
// Sel di luar grid dihitung TIDAK tercakup: tepi grid memang tidak
// terobservasi, dan menganggapnya sebaliknya akan menyembunyikan hal itu.
inline float diskCoverage(const std::vector<int>& counts,
                          int rows, int cols,
                          int ix, int iy,
                          const std::vector<std::pair<int, int>>& disk_offsets)
{
  if (disk_offsets.empty()) return 0.0f;

  int covered = 0;
  for (const auto& o : disk_offsets) {
    const int jx = ix + o.first;
    const int jy = iy + o.second;
    if (jx < 0 || jx >= cols || jy < 0 || jy >= rows) continue;
    if (counts[static_cast<size_t>(jy) * static_cast<size_t>(cols)
               + static_cast<size_t>(jx)] > 0)
      ++covered;
  }

  return static_cast<float>(covered)
       / static_cast<float>(disk_offsets.size());
}
