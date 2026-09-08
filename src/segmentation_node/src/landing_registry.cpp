// file: landing_registry.cpp
//
// Implementasi LandingRegistry — lihat landing_registry.hpp untuk kontraknya.

#include "segmentation_node/landing_registry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

// ─────────────────────────────────────────────────────────────────────────────
// findMatch — entri terdekat ke `c` dalam radius merge_dist_m, atau -1.
//
// Ambangnya inklusif: dua deteksi yang berjarak PERSIS merge_dist_m dianggap
// titik yang sama. Ini disengaja — permintaannya adalah (4,4,5) dan (5,4,5),
// yang berjarak tepat 1.0 m, harus tergabung pada merge_dist_m = 1.0.
// ─────────────────────────────────────────────────────────────────────────────
int LandingRegistry::findMatch(const Eigen::Vector3f& c) const
{
  const float thr2 = cfg_.merge_dist_m * cfg_.merge_dist_m;

  int   best    = -1;
  float best_d2 = std::numeric_limits<float>::max();

  for (size_t i = 0; i < items_.size(); ++i) {
    const float d2 = (items_[i].center - c).squaredNorm();
    if (d2 <= thr2 && d2 < best_d2) {
      best_d2 = d2;
      best    = static_cast<int>(i);
    }
  }
  return best;
}

// ─────────────────────────────────────────────────────────────────────────────
// observe — tiga fase: match & pulih, blokir, cap kapasitas.
// ─────────────────────────────────────────────────────────────────────────────
void LandingRegistry::observe(
    const std::vector<LandingCandidate>& seen,
    const std::function<bool(const Eigen::Vector3f&)>& was_observable,
    float dt_s)
{
  ++seq_;
  if (!(dt_s > 0.f)) dt_s = 0.f;   // menangkap negatif dan NaN sekaligus

  // Pulih dibuat proporsional: dari kondisi terhalang penuh (block_after_s),
  // recover_s detik terlihat kembali cukup untuk membuatnya boleh dipilih lagi.
  const float recover_rate = (cfg_.recover_s > 0.f)
                           ? (cfg_.block_after_s / cfg_.recover_s)
                           : std::numeric_limits<float>::max();

  // ── Fase 1: match & update ────────────────────────────────────────────────
  // `last_seen == seq_` adalah penanda "sudah terlihat di frame ini". Memakai
  // field yang memang sudah ada lebih aman daripada vektor flag terpisah,
  // karena items_ bisa bertambah di tengah loop ini.
  for (const auto& s : seen) {
    const int idx = findMatch(s.center);

    if (idx >= 0) {
      LandingCandidate& e = items_[static_cast<size_t>(idx)];

      // Posisi dibekukan saat center_ema == 0 (default). Titik yang sudah
      // terdaftar tidak dikoreksi lagi supaya bisa dirujuk dengan stabil.
      if (cfg_.center_ema > 0.f)
        e.center = (1.f - cfg_.center_ema) * e.center + cfg_.center_ema * s.center;

      e.clearance_m = s.clearance_m;   // metrik kualitas selalu pakai yang terbaru
      e.fill_ratio  = s.fill_ratio;

      // Dua deteksi yang jatuh ke entri yang sama dalam satu frame hanya boleh
      // dihitung sekali — kalau tidak, hits mencerminkan kepadatan grid,
      // bukan berapa kali zona itu benar-benar terlihat.
      if (e.last_seen != seq_) {
        ++e.hits;
        e.blocked_s = std::max(0.f, e.blocked_s - dt_s * recover_rate);

        // Rata-rata berjalan roughness. Ditaruh di dalam penjaga yang sama
        // dengan `hits` supaya satu frame menyumbang tepat satu sampel: dua
        // deteksi yang jatuh ke entri yang sama dalam satu frame mencerminkan
        // kepadatan grid, bukan dua pengamatan permukaan yang berbeda.
        //
        // Sampel yang tidak sah (cakram terlalu miskin titik, rough_n == 0)
        // dilewati sepenuhnya — nol bukan pengukuran, dan memasukkannya akan
        // menarik rata-rata ke bawah sehingga cakram yang datanya jarang
        // justru terlihat paling rata.
        if (s.rough_n > 0) {
          const float n_old = static_cast<float>(e.rough_n);
          e.roughness_m = (e.roughness_m * n_old + s.roughness_m)
                        / (n_old + 1.f);
          ++e.rough_n;
        }
      }
      e.last_seen = seq_;
    } else {
      LandingCandidate e = s;
      e.hits      = 1;
      e.blocked_s = 0.f;
      e.last_seen = seq_;
      items_.push_back(e);
    }
  }

  // ── Fase 2: blokir entri yang seharusnya terlihat tapi tidak muncul ───────
  // Entri di luar jangkauan pandang frame ini tidak disentuh sama sekali —
  // tidak terlihat karena kamera menghadap ke arah lain bukan bukti bahwa
  // zona itu sudah tidak aman.
  for (size_t i = 0; i < items_.size(); ) {
    LandingCandidate& e = items_[i];

    if (e.last_seen != seq_ && was_observable(e.center)) {
      e.blocked_s += dt_s;
      if (e.blocked_s >= cfg_.stale_after_s) {
        items_.erase(items_.begin() + static_cast<std::ptrdiff_t>(i));
        continue;
      }
    }

    e.selectable = (e.blocked_s < cfg_.block_after_s);
    ++i;
  }

  // ── Fase 3: kapasitas ─────────────────────────────────────────────────────
  enforceCapacity();
}

// ─────────────────────────────────────────────────────────────────────────────
// enforceCapacity — buang entri yang paling lama tidak terlihat (LRU).
// ─────────────────────────────────────────────────────────────────────────────
void LandingRegistry::enforceCapacity()
{
  // Entri yang dipin dikecualikan dari pemilihan korban. Tanpa ini, target
  // yang sedang dipakai konsumen justru yang pertama dibuang saat registry
  // membengkak oleh gerakan drone — stempel last_seen-nya menua persis karena
  // ia berhenti cocok, sementara kandidat baru terus datang dengan stempel
  // segar.
  //
  // Indeks pin dihitung ulang tiap putaran: erase menggeser indeks di
  // belakangnya.
  while (items_.size() > cfg_.max_candidates) {
    const std::ptrdiff_t pin_idx = has_pinned_ ? findMatch(pinned_) : -1;

    auto victim = items_.end();
    for (auto it = items_.begin(); it != items_.end(); ++it) {
      if (pin_idx >= 0 && std::distance(items_.begin(), it) == pin_idx) continue;
      if (victim == items_.end()
          || it->last_seen < victim->last_seen
          || (it->last_seen == victim->last_seen && it->hits < victim->hits))
        victim = it;
    }

    // Hanya entri yang dipin yang tersisa — berhenti, jangan membuangnya.
    if (victim == items_.end()) break;
    items_.erase(victim);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingRegistry::clear()
{
  items_.clear();
  has_pinned_   = false;
  has_rejected_ = false;
  // `seq_` sengaja TIDAK direset. Ia hanya penanda "sudah terlihat di frame
  // ini"; mengembalikannya ke nol tidak menambah apa pun, dan menaikkannya
  // terus membuat entri baru tidak mungkin bertabrakan dengan sisa nomor lama.
}

// ─────────────────────────────────────────────────────────────────────────────
size_t LandingRegistry::blockedCount() const
{
  size_t n = 0;
  for (const auto& c : items_) if (!c.selectable) ++n;
  return n;
}

// ─────────────────────────────────────────────────────────────────────────────
// scoreOf — nilai satu kandidat terhadap titik acuan.
// ─────────────────────────────────────────────────────────────────────────────
float LandingRegistry::scoreOf(const LandingCandidate& c,
                               const Eigen::Vector3f&  ref) const
{
  // Penormalisasi jarak mengikuti select_radius_m bila radius itu aktif:
  // di sanalah batas kerasnya memang berada, jadi kandidat di tepi area pindai
  // bernilai nol dan yang tepat di acuan bernilai satu. Tanpa radius, jatuh ke
  // nilai konfigurasi.
  ScoreWeights w = cfg_.score;
  if (cfg_.select_radius_m > 0.f) w.dist_ref_m = cfg_.select_radius_m;

  return landingScore(horizontalDistance(c.center, ref),
                      c.roughness_m, c.rough_n, w);
}

// ─────────────────────────────────────────────────────────────────────────────
// selectBest — argmax skor di antara kandidat yang lolos ketiga batas keras.
// ─────────────────────────────────────────────────────────────────────────────
const LandingCandidate* LandingRegistry::selectBest(
    const Eigen::Vector3f& ref) const
{
  const LandingCandidate* best = nullptr;
  float best_score = -1.f;   // skor selalu >= 0, jadi entri pertama pasti menang

  for (const auto& c : items_) {
    if (!c.selectable)            continue;
    if (!withinRadius(c.center, ref)) continue;
    if (isRejected(c.center))     continue;

    const float sc = scoreOf(c, ref);
    if (sc > best_score) {
      best_score = sc;
      best       = &c;
    }
  }
  return best;
}

// ─────────────────────────────────────────────────────────────────────────────
// selectTarget — selectBest + aturan kapan target lama boleh ditinggalkan.
// ─────────────────────────────────────────────────────────────────────────────
const LandingCandidate* LandingRegistry::selectTarget(
    const Eigen::Vector3f& ref,
    const Eigen::Vector3f& current_center,
    bool  has_current,
    bool  sticky) const
{
  const LandingCandidate* best = selectBest(ref);
  if (best == nullptr || !has_current) return best;

  // Lacak ulang target lama: entrinya bisa sudah dibuang, atau sedang
  // terhalang — dua-duanya berarti target itu harus ditinggalkan.
  const int idx = findMatch(current_center);
  if (idx < 0) return best;

  const LandingCandidate* current = &items_[static_cast<size_t>(idx)];
  if (!current->selectable) return best;

  // Radius adalah batas keras: sticky pun tidak boleh mempertahankan target
  // yang sudah terlalu jauh dari acuan.
  if (!withinRadius(current->center, ref)) return best;

  // Begitu pula penolakan konsumen. Inti perbaikannya: sticky tidak boleh
  // membuat TF menempel pada titik yang baru saja gagal didarati.
  if (isRejected(current->center)) return best;
  if (current == best) return best;

  // Sticky: target ditinggalkan hanya kalau sudah tidak layak (ditangani di
  // atas), bukan karena ada yang skornya lebih tinggi. Dipakai SETELAH gerbang
  // commit terbuka, saat TF sudah terbit dan konsumen sudah menuju ke sana —
  // pada titik itu perpindahan TF harus punya satu arti saja.
  if (sticky) return current;

  return (scoreOf(*best, ref) - scoreOf(*current, ref) > cfg_.score_hysteresis)
       ? best
       : current;
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingRegistry::setRejected(const Eigen::Vector3f& p)
{
  rejected_     = p;
  has_rejected_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingRegistry::clearRejected()
{
  has_rejected_ = false;
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingRegistry::setPinned(const Eigen::Vector3f& p)
{
  pinned_     = p;
  has_pinned_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingRegistry::clearPinned()
{
  has_pinned_ = false;
}

// ─────────────────────────────────────────────────────────────────────────────
bool LandingRegistry::addBlockedTime(const Eigen::Vector3f& p, float seconds)
{
  const int idx = findMatch(p);
  if (idx < 0) return false;

  LandingCandidate& e = items_[static_cast<size_t>(idx)];
  e.blocked_s  = std::max(0.f, e.blocked_s + seconds);
  e.selectable = (e.blocked_s < cfg_.block_after_s);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
bool LandingRegistry::isRejected(const Eigen::Vector3f& c) const
{
  if (!has_rejected_ || cfg_.reject_radius_m <= 0.f) return false;
  return (c - rejected_).squaredNorm()
         <= cfg_.reject_radius_m * cfg_.reject_radius_m;
}

// ─────────────────────────────────────────────────────────────────────────────
float LandingRegistry::horizontalDistance(
    const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
  const float dx = a.x() - b.x();
  const float dy = a.y() - b.y();
  return std::sqrt(dx * dx + dy * dy);
}

// ─────────────────────────────────────────────────────────────────────────────
bool LandingRegistry::withinRadius(
    const Eigen::Vector3f& c, const Eigen::Vector3f& ref) const
{
  if (cfg_.select_radius_m <= 0.f) return true;

  // MENDATAR saja. Acuan seleksi berada di ketinggian terbang sedangkan
  // kandidat berada di permukaan, sehingga jarak 3D antara keduanya tidak
  // pernah kurang dari ketinggian itu — radius 3D akan menolak semua kandidat
  // pada ketinggian pemindaian mana pun yang wajar. "Di dalam area yang
  // diminta untuk dipindai" memang pertanyaan mendatar.
  const float dx = c.x() - ref.x();
  const float dy = c.y() - ref.y();
  return dx * dx + dy * dy
         <= cfg_.select_radius_m * cfg_.select_radius_m;
}
