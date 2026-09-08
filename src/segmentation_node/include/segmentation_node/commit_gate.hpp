// file: commit_gate.hpp
//
// Kumpulkan dulu, baru umumkan.
//
// landing_circle dulu menerbitkan TF `map -> safety_point` sejak kandidat
// PERTAMA muncul — pada uji terbang itu 1,5 detik setelah node menyala, saat
// registry baru berisi segelintir entri. TF itu lalu ikut berpindah setiap
// kali ditemukan kandidat yang lebih baik, dan drone bereaksi terhadap
// keputusan yang belum matang: ia terbang ke satu titik, membatalkannya, lalu
// ke titik lain.
//
// Sebuah TF yang menyatakan "inilah titik pendaratan" seharusnya memang tidak
// terbit sebelum pernyataan itu benar. Gerbang ini menahannya sampai ada
// `collect_s` detik pengamatan sungguhan.
//
// Dipisah dari node supaya bisa diuji tanpa ROS, PCL, maupun jam sungguhan.

#pragma once

#include <algorithm>

class CommitGate
{
public:
  // `collect_s` <= 0 mematikan gerbang sepenuhnya: gerbang langsung terbuka
  // dan tidak pernah menutup lagi. Itu perilaku lama, dipakai di jalur debug.
  explicit CommitGate(float collect_s)
  : collect_s_(collect_s), committed_(collect_s <= 0.f) {}

  // Suapi satu frame. `have_candidate` harus berarti "ADA kandidat yang benar-
  // benar boleh dipilih frame ini", yaitu hasil seleksi setelah semua saringan
  // (terhalang, radius, penolakan) — bukan sekadar registry tidak kosong.
  // Registry berisi 50 entri yang semuanya terhalang tetap berarti tidak ada
  // titik tersimpan.
  //
  // Mengembalikan true bila keputusan sudah boleh diumumkan.
  bool update(bool have_candidate, float dt_s)
  {
    if (collect_s_ <= 0.f) return true;

    if (committed_) {
      // Selama masih ada kandidat layak, tetap terbuka: target yang terhalang
      // saat drone turun digantikan seketika oleh kandidat berikutnya dari
      // registry, dan drone naik lalu langsung menuju titik yang sudah ada.
      if (have_candidate) return true;

      // Kandidat habis — semua terhalang, terhapus, atau sudah ditolak. Tidak
      // ada lagi "titik yang tersimpan", jadi keputusan berikutnya harus
      // dikumpulkan dari nol.
      reset();
      return false;
    }

    // Jam hanya berjalan saat ada yang diamati. Kalau dihitung sejak node
    // menyala, sebagian jendela habis menunggu /plane_cpu mulai mengalir —
    // terukur 1,5-3 detik. N detik harus berarti N detik pengamatan.
    if (have_candidate)
      elapsed_s_ += std::max(0.f, dt_s);

    committed_ = (elapsed_s_ >= collect_s_);
    return committed_;
  }

  bool  committed() const { return committed_; }
  float elapsed_s() const { return elapsed_s_; }

  void reset()
  {
    elapsed_s_ = 0.f;
    committed_ = (collect_s_ <= 0.f);
  }

private:
  float collect_s_;
  float elapsed_s_ = 0.f;
  bool  committed_;
};
