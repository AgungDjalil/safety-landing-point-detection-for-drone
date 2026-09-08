// file: landing_registry.hpp
//
// Registry kandidat titik pendaratan yang persisten dalam frame `map`.
//
// Kelas ini sengaja dibuat bebas ROS dan PCL — hanya Eigen + STL — supaya
// bisa di-unit-test tanpa menyalakan node. Semua urusan geometri per-frame
// (proyeksi grid, transform TF) tetap menjadi tanggung jawab LandingCircle.
//
// Dua hal yang sebelumnya menyatu kini dipisah tegas:
//
//   POSISI   dibekukan. Begitu sebuah titik terdaftar, koordinatnya tidak
//            pernah dikoreksi lagi (center_ema default 0). Deteksi berikutnya
//            dalam radius merge_dist_m dianggap titik yang sama.
//
//   STATUS   terus diperbarui dan bisa pulih. Sebuah titik yang terhalang
//            tidak langsung dihapus — ia hanya berhenti boleh dipilih, dan
//            kembali boleh dipilih begitu terlihat aman lagi.
//
// Ambangnya berbasis WAKTU, bukan hitungan frame: laju /plane berbeda jauh
// antara backend GNG CPU (~2 Hz) dan GPU (~10 Hz), sehingga hitungan frame
// yang sama berarti durasi nyata yang sangat berbeda. "Sudah berapa lama
// terhalang" adalah besaran fisik yang benar untuk keputusan ini.

#pragma once

#include "segmentation_node/landing_score.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <vector>

// Satu kandidat titik pendaratan, koordinat selalu dalam frame `map`.
struct LandingCandidate {
  Eigen::Vector3f center{Eigen::Vector3f::Zero()};
  float clearance_m = 0.f;
  float fill_ratio  = 0.f;
  int   hits        = 1;     // berapa kali terobservasi
  long  last_seen   = 0;     // nomor urut observe() terakhir yang melihatnya

  // Akumulasi detik "seharusnya terlihat tapi tidak ada". Bertambah saat
  // terhalang, berkurang 3x lebih cepat saat terlihat lagi.
  float blocked_s   = 0.f;

  // Diperbarui tiap observe(): false berarti sedang terhalang dan tidak boleh
  // dipilih sebagai target, tapi entrinya tetap disimpan.
  bool  selectable  = true;

  // Rata-rata BERJALAN kekasaran permukaan cakram ini, dan berapa sampel sah
  // yang menyusunnya. Nol sampel berarti belum pernah terukur — dibedakan
  // tegas dari "terukur dan hasilnya nol", karena skor memperlakukan keduanya
  // secara berlawanan. Lihat landing_score.hpp.
  //
  // Dirata-ratakan lintas frame karena satu cakram hanya berisi ~7 titik pada
  // kerapatan /plane_cpu, sehingga satu pengukuran tunggal berisik. Sepuluh
  // detik pengumpulan pada ~0,8 Hz memberi hingga delapan sampel — inilah yang
  // membuat jendela pengumpulan benar-benar bekerja, bukan sekadar menunggu.
  float roughness_m = 0.f;
  int   rough_n     = 0;
};

struct RegistryConfig {
  // Dua deteksi dalam radius ini dianggap titik pendaratan yang sama.
  float  merge_dist_m   = 1.0f;

  // 0 = posisi dibekukan setelah pendaftaran pertama (perilaku default).
  // Nilai > 0 membuat posisi ditarik ke deteksi terbaru secara eksponensial.
  float  center_ema     = 0.0f;

  size_t max_candidates = 50;

  // Terhalang selama ini (detik) -> berhenti boleh dipilih.
  float  block_after_s  = 3.0f;
  // Lama terlihat (detik) yang dibutuhkan untuk pulih penuh dari terhalang.
  float  recover_s      = 1.0f;
  // Terhalang selama ini (detik) -> dihapus permanen.
  float  stale_after_s  = 30.0f;

  // Batas keras jarak MENDATAR dari titik acuan seleksi. 0 = tanpa batas.
  //
  // Mendatar, bukan 3D: acuannya berada di ketinggian terbang sedangkan
  // kandidat berada di permukaan, jadi radius 3D akan menolak semuanya.
  //
  // Registry mengumpulkan kandidat sepanjang penerbangan, jadi tanpa batas ini
  // "yang terdekat" bisa saja sebuah titik belasan meter jauhnya yang terlihat
  // di tengah perjalanan. Untuk sebuah misi, yang dicari adalah titik aman di
  // sekitar area yang diminta untuk dipindai, bukan yang kebetulan terdekat.
  float  select_radius_m = 0.0f;

  // true = target yang sedang dipakai TIDAK ditinggalkan hanya karena ada yang
  // lebih dekat; ia hanya ditinggalkan bila sudah terhalang, terhapus, atau
  // jatuh di luar select_radius_m.
  //
  // Ini bukan penyetelan kosmetik. Acuan seleksi bergerak (drone terbang, atau
  // waypoint berganti), sehingga "terdekat" berubah terus-menerus tanpa ada
  // apa pun yang terjadi pada targetnya sendiri. Konsumen yang membaca TF
  // `safety_point` tidak bisa membedakan kedua sebab itu; membuat seleksinya
  // sticky-lah yang membuat perpindahan TF punya satu arti saja: target lama
  // sudah tidak layak. Diuji terbang — tanpa ini, node misi membatalkan
  // pendaratan tiga kali berturut-turut karena TF bergeser 1,7-11,3 m
  // sementara semua kandidatnya masih aman.
  bool   sticky_target   = false;

  // Radius di sekitar titik yang DITOLAK konsumen; kandidat di dalamnya tidak
  // boleh dipilih. 0 = fitur mati.
  //
  // Registry tahu kandidat mana yang terhalang, tapi tidak tahu kandidat mana
  // yang baru saja GAGAL didarati — itu hanya diketahui node misi. Tanpa
  // jalur ini, seleksi sticky menempel pada titik yang baru saja ditinggalkan:
  // misi naik, memindai ulang, lalu mengunci titik yang sama persis, dan dari
  // luar terlihat seperti titik landing yang mandek di posisi lama.
  //
  // Diukur 3D, bukan mendatar seperti select_radius_m: acuannya adalah titik
  // pendaratan itu sendiri, jadi selisih ketinggian memang berarti.
  float  reject_radius_m = 1.5f;

  // Bobot dan penormalisasi skor seleksi. Lihat landing_score.hpp.
  ScoreWeights score;

  // Margin skor yang harus dilampaui sebelum target berpindah, HANYA di jalur
  // non-sticky. Tak berdimensi karena yang dibandingkan skor, bukan meter.
  //
  // Menggantikan `retarget_hysteresis_m` yang lama: mempertahankan ambang
  // bersatuan meter untuk membandingkan besaran tak berdimensi berarti
  // menyimpan parameter yang diam-diam berubah arti.
  float  score_hysteresis = 0.05f;
};

class LandingRegistry
{
public:
  explicit LandingRegistry(RegistryConfig cfg) : cfg_(cfg) {}

  // Perbarui registry dengan hasil satu frame.
  //
  //   seen            kandidat yang terdeteksi frame ini, dalam frame `map`.
  //   was_observable  predikat: true bila pusat tersebut SEHARUSNYA terlihat
  //                   pada frame ini. Pemanggil yang menyediakannya karena
  //                   dialah yang memegang transform dan batas grid.
  //   dt_s            detik sejak observe() sebelumnya.
  //
  // Entri di luar jangkauan pandang tidak disentuh sama sekali — tidak
  // terlihat karena kamera menghadap arah lain bukan bukti bahwa zona itu
  // sudah tidak aman.
  void observe(const std::vector<LandingCandidate>& seen,
               const std::function<bool(const Eigen::Vector3f&)>& was_observable,
               float dt_s);

  // Skor satu kandidat terhadap titik acuan `ref`. Lihat landing_score.hpp.
  // Dibuka ke publik supaya pemanggil bisa MELAPORKAN alasan sebuah titik
  // menang, bukan hanya menerima hasilnya.
  float scoreOf(const LandingCandidate& c, const Eigen::Vector3f& ref) const;

  // Kandidat dengan SKOR TERTINGGI terhadap `ref` — gabungan jarak mendatar
  // dan roughness. Hanya kandidat yang boleh dipilih, berada dalam
  // select_radius_m, dan tidak sedang ditolak yang dipertimbangkan; ketiganya
  // batas keras, bukan bagian dari skor.
  //
  // Mengembalikan nullptr bila tidak ada satu pun yang memenuhi syarat.
  //
  // Pointer yang dikembalikan menunjuk ke dalam penyimpanan internal dan
  // menjadi tidak valid setelah observe() berikutnya.
  const LandingCandidate* selectBest(const Eigen::Vector3f& ref) const;

  // Seleksi yang dipakai jalur produksi: selectBest, ditambah aturan kapan
  // target yang sudah dipegang boleh ditinggalkan.
  //
  //   current_center  pusat target yang sedang dipakai, dalam frame `map`.
  //   has_current     false pada frame pertama, saat belum ada target.
  //   sticky          true = pertahankan target apa pun skor pesaingnya.
  //
  // `sticky` sengaja argumen, bukan dibaca dari cfg_.sticky_target saja:
  // pemanggil menyalakannya hanya SETELAH gerbang commit terbuka. Sticky sejak
  // frame pertama adalah bug yang pernah terjadi — target terkunci pada
  // kandidat yang kebetulan terlihat lebih dulu, dan sepuluh detik pengumpulan
  // berikutnya mengisi daftar yang keputusannya sudah diambil.
  //
  // Saat sticky mati, target tetap tidak berpindah untuk keuntungan skor yang
  // remeh: pesaing harus unggul lebih dari cfg.score_hysteresis. Tanpa itu dua
  // kandidat yang skornya nyaris sama akan bertukar tiap frame.
  //
  // Target yang sedang dipakai tapi sudah terhalang, keluar radius, atau
  // ditolak konsumen TIDAK pernah dipertahankan — sticky sekalipun.
  const LandingCandidate* selectTarget(
      const Eigen::Vector3f& ref,
      const Eigen::Vector3f& current_center,
      bool has_current,
      bool sticky) const;

  // Tandai satu titik sebagai baru saja gagal didarati. Kandidat dalam
  // reject_radius_m darinya berhenti boleh dipilih sampai clearRejected().
  //
  // Ini BUKAN vonis permanen dan tidak menyentuh isi registry: kalau
  // penghalangnya pergi, penilaian registry sendiri yang lebih layak dipakai,
  // jadi pemanggil yang memutuskan kapan melepasnya.
  void setRejected(const Eigen::Vector3f& p);
  void clearRejected();

  // Lindungi satu entri dari pembuangan karena kapasitas: target yang sedang
  // dipakai konsumen.
  //
  // enforceCapacity() membuang berdasarkan last_seen. Saat drone bertranslasi,
  // kandidat baru membanjir dengan stempel segar sementara target yang dikunci
  // justru berhenti cocok (grid bergeser 1,7-2,4 m, lebih jauh dari
  // merge_dist_m) sehingga stempelnya menua — ia jadi korban LRU pertama, dan
  // konsumen kehilangan target yang sebenarnya masih baik-baik saja.
  //
  // Hanya SATU entri yang dilindungi, jadi registry tetap berbatas.
  void setPinned(const Eigen::Vector3f& p);
  void clearPinned();

  // Tambahkan waktu terhalang pada entri terdekat ke `p`, TANPA memandang
  // pembekuan jam observe(). Untuk konsumen yang punya bukti LANGSUNG tentang
  // satu titik tertentu — misalnya menguji ulang cakram titik yang sudah
  // dikunci terhadap peta obstacle frame ini, yang tidak butuh pencocokan
  // identitas karena koordinatnya sudah diketahui.
  //
  // Perlu ada karena observe() sengaja dibekukan (dt_s = 0) saat drone
  // bertranslasi: entri berhenti cocok akibat grid bergeser mengikuti kamera,
  // dan memberinya strike akan memvonis zona yang sebenarnya aman. Pembekuan
  // itu benar untuk pencocokan identitas, tapi ia ikut mematikan deteksi
  // penghalang atas target yang sudah dipilih. Terukur: 27 dari 27 frame
  // selama penurunan berjalan beku.
  //
  // Negatif untuk memulihkan; blocked_s selalu diklem di 0. `selectable`
  // dihitung ulang memakai block_after_s yang sama, jadi tidak ada jalur
  // "terhalang" kedua di sistem — hanya sumber buktinya yang bertambah.
  //
  // false bila tidak ada entri dalam merge_dist_m dari `p`.
  bool addBlockedTime(const Eigen::Vector3f& p, float seconds);

  // Buang SELURUH isi registry: entri, pin, dan penolakan.
  //
  // Dipakai saat target yang sudah dikunci terpaksa dilepas. Kandidat yang
  // tersisa saat itu dikumpulkan sebelum drone menukik, dari ketinggian yang
  // sama sekali berbeda, dan sebagian sudah berumur puluhan detik — memakai
  // salah satunya sebagai target berikutnya berarti mendarat di tempat yang
  // penilaiannya sudah usang. Membuang semuanya memaksa pengamatan baru.
  void clear();

  const std::vector<LandingCandidate>& all() const { return items_; }
  size_t size() const { return items_.size(); }
  bool   empty() const { return items_.empty(); }

  // Jumlah entri yang sedang terhalang (tersimpan tapi tidak boleh dipilih).
  size_t blockedCount() const;

private:
  // Indeks entri terdekat ke `c` dalam radius merge_dist_m, atau -1.
  int findMatch(const Eigen::Vector3f& c) const;

  // Jarak MENDATAR antara dua titik. Dipakai skor dan withinRadius, dua-duanya
  // karena acuan seleksi berada di ketinggian terbang sedangkan kandidat
  // berada di permukaan.
  static float horizontalDistance(const Eigen::Vector3f& a,
                                  const Eigen::Vector3f& b);

  // Apakah `c` cukup dekat ke acuan seleksi. Selalu true bila radiusnya 0.
  bool withinRadius(const Eigen::Vector3f& c,
                    const Eigen::Vector3f& ref) const;

  // Apakah `c` jatuh dalam radius titik yang ditolak konsumen.
  bool isRejected(const Eigen::Vector3f& c) const;

  // Buang entri yang paling lama tidak terlihat sampai muat kapasitas.
  //
  // Sengaja LRU, bukan "hits terendah": kandidat yang baru masuk selalu
  // punya hits == 1, jadi aturan hits-terendah akan selalu membuangnya lebih
  // dulu dan registry yang sudah penuh tidak akan pernah bisa menerima lokasi
  // baru. LRU tidak punya masalah itu karena entri baru justru yang paling
  // baru terlihat.
  void enforceCapacity();

  RegistryConfig                cfg_;
  std::vector<LandingCandidate> items_;
  long                          seq_ = 0;   // dinaikkan tiap observe()

  Eigen::Vector3f               rejected_{Eigen::Vector3f::Zero()};
  bool                          has_rejected_ = false;

  Eigen::Vector3f               pinned_{Eigen::Vector3f::Zero()};
  bool                          has_pinned_ = false;
};
