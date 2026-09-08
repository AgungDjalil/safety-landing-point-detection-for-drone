// Unit test untuk LandingRegistry.
//
// Registry sengaja dibuat bebas ROS/PCL, jadi seluruh berkas ini berjalan
// tanpa rclcpp, tanpa TF, dan tanpa point cloud.

#include <gtest/gtest.h>

#include "segmentation_node/landing_registry.hpp"
#include "segmentation_node/plane_roughness.hpp"

namespace {

// Predikat observabilitas yang selalu menjawab sama.
auto always(bool v) {
  return [v](const Eigen::Vector3f&) { return v; };
}

LandingCandidate cand(float x, float y, float z, float clearance = 1.f) {
  LandingCandidate c;
  c.center      = Eigen::Vector3f(x, y, z);
  c.clearance_m = clearance;
  c.fill_ratio  = 1.f;
  return c;
}

RegistryConfig defaultCfg() {
  RegistryConfig cfg;
  cfg.merge_dist_m   = 1.0f;
  cfg.center_ema     = 0.0f;   // posisi dibekukan
  cfg.max_candidates = 50;
  cfg.block_after_s  = 3.0f;
  cfg.recover_s      = 1.0f;
  cfg.stale_after_s  = 30.0f;
  // Sama persis dengan default node, supaya tes menguji angka yang benar-benar
  // berjalan di udara — bukan angka yang dipilih agar tesnya lulus.
  cfg.score.w_dist      = 0.5f;
  cfg.score.w_rough     = 0.5f;
  cfg.score.dist_ref_m  = 10.0f;
  cfg.score.rough_max_m = 0.10f;
  cfg.score_hysteresis  = 0.05f;
  return cfg;
}

// Jalankan observe() berulang selama `seconds` detik dengan langkah `dt`.
void advance(LandingRegistry& reg,
             const std::vector<LandingCandidate>& seen,
             bool observable, float seconds, float dt = 0.5f) {
  for (float t = 0.f; t < seconds - 1e-6f; t += dt)
    reg.observe(seen, always(observable), dt);
}

}  // namespace

// ── 1. Penambahan & merge ───────────────────────────────────────────────────

TEST(LandingRegistry, AddsUnmatchedCandidatesAsNewEntries)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(0, 0, 0), cand(5, 0, 0)}, always(false), 0.5f);
  EXPECT_EQ(reg.size(), 2u);
}

// Contoh konkret dari permintaan: (4,4,5) lalu (5,4,5) berjarak 1.0 m harus
// dianggap satu titik pendaratan yang sama.
TEST(LandingRegistry, MergesCandidateOneMetreAway)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(4, 4, 5)}, always(false), 0.5f);
  reg.observe({cand(5, 4, 5)}, always(false), 0.5f);

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_EQ(reg.all()[0].hits, 2);
}

TEST(LandingRegistry, DoesNotMergeCandidateBeyondMergeDistance)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(1.5f, 0, 0)}, always(false), 0.5f);
  EXPECT_EQ(reg.size(), 2u);
}

// ── 2. Posisi dibekukan ─────────────────────────────────────────────────────

TEST(LandingRegistry, PositionIsFrozenAfterFirstRegistration)
{
  LandingRegistry reg(defaultCfg());   // center_ema = 0
  reg.observe({cand(4, 4, 5)}, always(false), 0.5f);

  for (int i = 0; i < 10; ++i)
    reg.observe({cand(4.8f, 4.2f, 5.1f)}, always(false), 0.5f);

  ASSERT_EQ(reg.size(), 1u);
  const auto& c = reg.all()[0];
  EXPECT_FLOAT_EQ(c.center.x(), 4.f) << "posisi dikoreksi padahal harus beku";
  EXPECT_FLOAT_EQ(c.center.y(), 4.f);
  EXPECT_FLOAT_EQ(c.center.z(), 5.f);
}

TEST(LandingRegistry, NonZeroEmaStillDragsPosition)
{
  RegistryConfig cfg = defaultCfg();
  cfg.center_ema = 0.3f;
  LandingRegistry reg(cfg);

  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(1.0f, 0, 0)}, always(false), 0.5f);

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_NEAR(reg.all()[0].center.x(), 0.3f, 1e-5f);
}

TEST(LandingRegistry, MergeRefreshesQualityMetrics)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(0, 0, 0, /*clearance=*/1.f)}, always(false), 0.5f);
  reg.observe({cand(0, 0, 0, /*clearance=*/2.5f)}, always(false), 0.5f);

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_FLOAT_EQ(reg.all()[0].clearance_m, 2.5f);
}

// ── 3. Objek bergerak: terhalang sesaat harus dimaafkan ─────────────────────

TEST(LandingRegistry, TransientBlockageKeepsCandidateSelectable)
{
  LandingRegistry reg(defaultCfg());   // block_after_s = 3.0
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);

  // Objek melintas 2 detik — di bawah ambang 3 detik.
  advance(reg, {}, /*observable=*/true, 2.0f);

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_TRUE(reg.all()[0].selectable)
      << "objek yang cuma melintas tidak boleh mencoret titik pendaratan";
}

TEST(LandingRegistry, SustainedBlockageMakesCandidateUnselectableButKept)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);

  advance(reg, {}, /*observable=*/true, 3.5f);

  ASSERT_EQ(reg.size(), 1u) << "titik terhalang tidak boleh langsung dihapus";
  EXPECT_FALSE(reg.all()[0].selectable);
  EXPECT_EQ(reg.blockedCount(), 1u);
}

TEST(LandingRegistry, BlockedCandidateRecoversWhenSeenAgain)
{
  LandingRegistry reg(defaultCfg());   // recover_s = 1.0
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);

  advance(reg, {}, /*observable=*/true, 3.5f);
  ASSERT_FALSE(reg.all()[0].selectable);

  // Pulih penuh butuh recover_s = 1 detik terlihat kembali.
  advance(reg, {cand(0, 0, 0)}, /*observable=*/true, 1.5f);

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_TRUE(reg.all()[0].selectable) << "titik tidak pulih setelah bebas lagi";
  EXPECT_FLOAT_EQ(reg.all()[0].blocked_s, 0.f);
}

TEST(LandingRegistry, PermanentBlockageEventuallyDeletesCandidate)
{
  LandingRegistry reg(defaultCfg());   // stale_after_s = 30.0
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);

  advance(reg, {}, /*observable=*/true, 20.0f);
  EXPECT_EQ(reg.size(), 1u) << "dihapus terlalu cepat, sebelum stale_after_s";

  advance(reg, {}, /*observable=*/true, 11.0f);
  EXPECT_EQ(reg.size(), 0u) << "titik yang terhalang permanen harus dibuang";
}

TEST(LandingRegistry, OutOfViewCandidateIsNeverBlocked)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);

  // Satu menit penuh di luar jangkauan pandang.
  advance(reg, {}, /*observable=*/false, 60.0f);

  ASSERT_EQ(reg.size(), 1u) << "kandidat di luar FOV tidak boleh dibuang";
  EXPECT_TRUE(reg.all()[0].selectable);
  EXPECT_FLOAT_EQ(reg.all()[0].blocked_s, 0.f);
}

TEST(LandingRegistry, BlocksOnlyEntriesInsideObservableRegion)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(0, 0, 0), cand(10, 0, 0)}, always(false), 0.5f);
  ASSERT_EQ(reg.size(), 2u);

  auto near_only = [](const Eigen::Vector3f& c) { return c.x() < 5.f; };
  for (float t = 0.f; t < 3.5f; t += 0.5f)
    reg.observe({}, near_only, 0.5f);

  ASSERT_EQ(reg.size(), 2u);
  for (const auto& c : reg.all()) {
    if (c.center.x() < 5.f) EXPECT_FALSE(c.selectable);
    else                    EXPECT_TRUE(c.selectable);
  }
}

// ── 4. Seleksi terdekat Euclidean 3D ────────────────────────────────────────

TEST(LandingRegistry, SelectNearestReturnsNullptrWhenEmpty)
{
  LandingRegistry reg(defaultCfg());
  EXPECT_EQ(reg.selectBest(Eigen::Vector3f(0, 0, 0)), nullptr);
}

TEST(LandingRegistry, SelectNearestPicksMinimumEuclideanDistance)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(10, 0, 0), cand(3, 0, 0), cand(7, 0, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f(0, 0, 0));
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 3.f);
}

// Seleksi memakai jarak MENDATAR, bukan Euclidean 3D.
//
// Dulu 3D, dan itu salah untuk perkara ini: acuan seleksi (`scan_center`)
// berada di ketinggian terbang sedangkan kandidat berada di permukaan, jadi
// jarak 3D antara keduanya selalu memuat selisih ketinggian yang kurang lebih
// sama untuk SEMUA kandidat. Untuk sekadar mengurutkan itu masih monoton, tapi
// begitu jaraknya dinormalisasi menjadi suku skor, semua nilainya berdesakan
// di dekat satu angka dan berhenti membedakan apa pun.
//
// "Seberapa dekat ke tempat yang diminta dipindai" memang pertanyaan mendatar.
TEST(LandingRegistry, SelectBestUsesHorizontalDistanceNot3d)
{
  LandingRegistry reg(defaultCfg());
  // A: mendatar 1.0 tapi 3D ~= 10.05 ; B: mendatar 4.0 dan 3D 4.0.
  // Aturan 3D memenangkan B; aturan mendatar memenangkan A.
  reg.observe({cand(1, 0, 10), cand(4, 0, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f(0, 0, 0));
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 1.f)
      << "seleksi harus memakai jarak mendatar, bukan Euclidean 3D";
}

TEST(LandingRegistry, SelectNearestSkipsBlockedCandidates)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(3, 0, 0), cand(9, 0, 0)}, always(false), 0.5f);

  // Halangi hanya yang dekat: hanya x < 5 yang teramati.
  auto near_only = [](const Eigen::Vector3f& c) { return c.x() < 5.f; };
  for (float t = 0.f; t < 3.5f; t += 0.5f)
    reg.observe({}, near_only, 0.5f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f(0, 0, 0));
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 9.f)
      << "titik yang sedang terhalang tidak boleh dipilih jadi target";
}

TEST(LandingRegistry, SelectNearestReturnsNullptrWhenAllBlocked)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(3, 0, 0)}, always(false), 0.5f);
  advance(reg, {}, /*observable=*/true, 3.5f);

  EXPECT_EQ(reg.selectBest(Eigen::Vector3f(0, 0, 0)), nullptr);
}

// ── 5. Kapasitas ────────────────────────────────────────────────────────────

TEST(LandingRegistry, CapacityIsEnforced)
{
  RegistryConfig cfg = defaultCfg();
  cfg.max_candidates = 2;
  LandingRegistry reg(cfg);

  reg.observe({cand(0, 0, 0), cand(5, 0, 0), cand(10, 0, 0)}, always(false), 0.5f);
  EXPECT_EQ(reg.size(), 2u);
}

// Regresi: dengan aturan "buang hits terendah", kandidat baru (hits == 1)
// selalu jadi korban dan registry yang penuh tidak akan pernah menerima lokasi
// baru. LRU harus membuang entri yang paling lama tidak terlihat.
TEST(LandingRegistry, FullRegistryStillAcceptsNewCandidates)
{
  RegistryConfig cfg = defaultCfg();
  cfg.max_candidates = 2;
  LandingRegistry reg(cfg);

  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(5, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(5, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(20, 0, 0)}, always(false), 0.5f);

  ASSERT_EQ(reg.size(), 2u);
  bool has_new = false;
  for (const auto& c : reg.all())
    if (c.center.x() == 20.f) has_new = true;
  EXPECT_TRUE(has_new)
      << "registry penuh menolak kandidat baru — eviksi harus LRU, bukan hits";
}

TEST(LandingRegistry, CapacityEvictsLeastRecentlySeen)
{
  RegistryConfig cfg = defaultCfg();
  cfg.max_candidates = 2;
  LandingRegistry reg(cfg);

  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(5, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(5, 0, 0)}, always(false), 0.5f);
  reg.observe({cand(20, 0, 0)}, always(false), 0.5f);

  ASSERT_EQ(reg.size(), 2u);
  for (const auto& c : reg.all())
    EXPECT_NE(c.center.x(), 0.f) << "A adalah yang paling lama tidak terlihat";
}

// ── 6. Histeresis pemilihan target ──────────────────────────────────────────

TEST(LandingRegistry, HysteresisFallsBackToNearestWhenNoCurrentTarget)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(3, 0, 0), cand(9, 0, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f(0, 0, 0), Eigen::Vector3f::Zero(), false, false);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 3.f);
}

TEST(LandingRegistry, HysteresisKeepsCurrentWhenGainIsSmall)
{
  LandingRegistry reg(defaultCfg());
  // A di (5,0,0) berjarak 5.00 m; B di (0,4.9,0) berjarak 4.90 m.
  // Selisih 0.10 m, di bawah ambang 0.15 m. Keduanya terpisah jauh > merge.
  reg.observe({cand(5.0f, 0, 0), cand(0, 4.9f, 0)}, always(false), 0.5f);
  ASSERT_EQ(reg.size(), 2u);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(5.0f, 0, 0), true, false);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 5.0f)
      << "target berpindah padahal keuntungannya di bawah histeresis";
}

TEST(LandingRegistry, HysteresisSwitchesWhenGainExceedsThreshold)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(5.0f, 0, 0), cand(0, 3.0f, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(5.0f, 0, 0), true, false);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.y(), 3.0f);
}

TEST(LandingRegistry, HysteresisAdoptsNearestWhenCurrentTargetIsGone)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(5.0f, 0, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(50.f, 0, 0), true, false);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 5.0f);
}

// Target yang sedang dipakai lalu terhalang tidak boleh dipertahankan.
TEST(LandingRegistry, HysteresisAbandonsCurrentTargetOnceBlocked)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(3, 0, 0), cand(9, 0, 0)}, always(false), 0.5f);

  auto near_only = [](const Eigen::Vector3f& c) { return c.x() < 5.f; };
  for (float t = 0.f; t < 3.5f; t += 0.5f)
    reg.observe({}, near_only, 0.5f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(3, 0, 0), true, false);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 9.f)
      << "target yang terhalang harus ditinggalkan, bukan dipertahankan";
}

TEST(LandingRegistry, HysteresisReturnsNullptrWhenEmpty)
{
  LandingRegistry reg(defaultCfg());
  EXPECT_EQ(reg.selectTarget(Eigen::Vector3f(0, 0, 0),
                                     Eigen::Vector3f(1, 0, 0), true, false),
            nullptr);
}

// ─────────────────────────────────────────────────────────────────────────────
// select_radius_m — batas keras jarak dari titik acuan.
//
// Node misi menyiarkan TF `map -> scan_center` di waypoint dan menjalankan
// landing_circle dengan base_frame:=scan_center, sehingga acuan seleksi adalah
// waypoint, bukan drone. Radius inilah yang mencegah drone dikirim ke kandidat
// belasan meter jauhnya yang kebetulan ikut terkumpul sepanjang perjalanan.
// ─────────────────────────────────────────────────────────────────────────────

TEST(LandingRegistrySelectRadius, IgnoresCandidatesBeyondTheRadius)
{
  RegistryConfig cfg = defaultCfg();
  cfg.select_radius_m = 5.0f;
  LandingRegistry reg(cfg);

  reg.observe({cand(12.f, 0.f, 0.f), cand(4.f, 0.f, 0.f)}, always(true), 0.1f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 4.f);
}

TEST(LandingRegistrySelectRadius, ZeroMeansUnlimited)
{
  RegistryConfig cfg = defaultCfg();
  cfg.select_radius_m = 0.0f;          // perilaku lama, tidak dibatasi
  LandingRegistry reg(cfg);

  reg.observe({cand(40.f, 0.f, 0.f)}, always(true), 0.1f);

  EXPECT_NE(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistrySelectRadius, BoundaryDistanceIsInside)
{
  RegistryConfig cfg = defaultCfg();
  cfg.select_radius_m = 5.0f;
  LandingRegistry reg(cfg);

  reg.observe({cand(5.f, 0.f, 0.f)}, always(true), 0.1f);

  EXPECT_NE(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistrySelectRadius, NullptrWhenEverythingIsTooFar)
{
  RegistryConfig cfg = defaultCfg();
  cfg.select_radius_m = 3.0f;
  LandingRegistry reg(cfg);

  reg.observe({cand(9.f, 0.f, 0.f), cand(0.f, 12.f, 0.f)}, always(true), 0.1f);

  EXPECT_EQ(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistrySelectRadius, IgnoresAltitudeDifference)
{
  RegistryConfig cfg = defaultCfg();
  cfg.select_radius_m = 5.0f;
  LandingRegistry reg(cfg);

  // Tepat di bawah acuan secara horizontal, tapi 9 m di bawahnya.
  //
  // Radiusnya HARUS horizontal. Acuan seleksi node misi adalah `scan_center`,
  // yang berada di ketinggian waypoint (mis. 11 m), sedangkan kandidat berada
  // di tanah. Jarak 3D antara keduanya tidak pernah kurang dari ketinggian
  // terbang, jadi radius berbasis 3D akan menolak setiap kandidat yang ada.
  reg.observe({cand(0.f, 0.f, -9.f)}, always(true), 0.1f);

  EXPECT_NE(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistrySelectRadius, StillRejectsOnHorizontalDistanceWhenHigh)
{
  RegistryConfig cfg = defaultCfg();
  cfg.select_radius_m = 5.0f;
  LandingRegistry reg(cfg);

  // 9 m mendatar, 11 m di bawah: di luar area yang diminta untuk dipindai.
  reg.observe({cand(9.f, 0.f, -11.f)}, always(true), 0.1f);

  EXPECT_EQ(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

// ─────────────────────────────────────────────────────────────────────────────
// sticky_target — TF hanya berpindah kalau targetnya memang sudah tidak layak.
//
// Diuji terbang: tanpa ini, seleksi "terdekat" berpindah terus-menerus karena
// DRONENYA yang bergerak, bukan karena targetnya jadi tidak aman. Node misi
// membaca perpindahan itu sebagai "target terhalang" dan membatalkan
// pendaratan tiga kali berturut-turut sebelum menyerah.
// ─────────────────────────────────────────────────────────────────────────────

TEST(LandingRegistrySticky, KeepsTheCurrentTargetEvenWhenAnotherIsMuchNearer)
{
  RegistryConfig cfg = defaultCfg();
  cfg.sticky_target = true;
  LandingRegistry reg(cfg);

  reg.observe({cand(8.f, 0.f, 0.f), cand(0.f, 1.f, 0.f)}, always(true), 0.1f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(8.f, 0.f, 0.f), true, true);
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 8.f);
}

TEST(LandingRegistrySticky, MovesOnWhenTheCurrentTargetBecomesBlocked)
{
  RegistryConfig cfg = defaultCfg();
  cfg.sticky_target = true;
  LandingRegistry reg(cfg);

  reg.observe({cand(8.f, 0.f, 0.f), cand(0.f, 1.f, 0.f)}, always(true), 0.1f);

  // Hanya (0,1,0) yang terlihat lagi; (8,0,0) seharusnya terlihat tapi hilang.
  for (int i = 0; i < 40; ++i)
    reg.observe({cand(0.f, 1.f, 0.f)}, always(true), 0.1f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(8.f, 0.f, 0.f), true, true);
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.y(), 1.f);
}

TEST(LandingRegistrySticky, MovesOnWhenTheCurrentTargetIsGoneEntirely)
{
  RegistryConfig cfg = defaultCfg();
  cfg.sticky_target = true;
  LandingRegistry reg(cfg);

  reg.observe({cand(0.f, 1.f, 0.f)}, always(true), 0.1f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(30.f, 30.f, 0.f), true, true);
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.y(), 1.f);
}

TEST(LandingRegistrySticky, PicksTheNearestWhenThereIsNoCurrentTarget)
{
  RegistryConfig cfg = defaultCfg();
  cfg.sticky_target = true;
  LandingRegistry reg(cfg);

  reg.observe({cand(8.f, 0.f, 0.f), cand(0.f, 2.f, 0.f)}, always(true), 0.1f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), false, true);
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.y(), 2.f);
}

TEST(LandingRegistrySticky, AbandonsACurrentTargetThatFellOutsideTheRadius)
{
  RegistryConfig cfg = defaultCfg();
  cfg.sticky_target   = true;
  cfg.select_radius_m = 5.0f;
  LandingRegistry reg(cfg);

  reg.observe({cand(9.f, 0.f, 0.f), cand(0.f, 2.f, 0.f)}, always(true), 0.1f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(9.f, 0.f, 0.f), true, true);
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.y(), 2.f);
}

TEST(LandingRegistrySticky, NullptrWhenNothingIsLeftInsideTheRadius)
{
  RegistryConfig cfg = defaultCfg();
  cfg.sticky_target   = true;
  cfg.select_radius_m = 2.0f;
  LandingRegistry reg(cfg);

  reg.observe({cand(9.f, 0.f, 0.f)}, always(true), 0.1f);

  EXPECT_EQ(reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(9.f, 0.f, 0.f), true, true),
      nullptr);
}

TEST(LandingRegistrySticky, DisabledByDefaultSoHysteresisStillGoverns)
{
  RegistryConfig cfg = defaultCfg();          // sticky_target tidak diset
  LandingRegistry reg(cfg);

  reg.observe({cand(8.f, 0.f, 0.f), cand(0.f, 1.f, 0.f)}, always(true), 0.1f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(8.f, 0.f, 0.f), true, false);
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.y(), 1.f);      // pindah ke yang jauh lebih dekat
}

// ─────────────────────────────────────────────────────────────────────────────
// diskCoverage — berapa banyak cakram yang benar-benar TERSENTUH data frame ini.
//
// Dipakai untuk memutuskan apakah sebuah kandidat lama boleh kena strike.
// Uji terbang menunjukkan kenapa ini perlu: /plane_cpu bervariasi 243-3378
// titik per frame, dan cakram terpilih kosong sama sekali pada 26% frame.
// Tanpa uji cakupan, frame yang kebetulan tidak menyentuh sebuah zona dibaca
// sebagai "zona itu terhalang", dan tiga frame sial berturut-turut sudah cukup
// memvonis zona yang sebenarnya aman.
// ─────────────────────────────────────────────────────────────────────────────

#include "segmentation_node/grid_coverage.hpp"

namespace {
// Cakram 3x3 penuh (radius 1 sel, 8-neigh + pusat).
std::vector<std::pair<int, int>> disk3x3() {
  std::vector<std::pair<int, int>> d;
  for (int dy = -1; dy <= 1; ++dy)
    for (int dx = -1; dx <= 1; ++dx)
      if (dx * dx + dy * dy <= 1) d.emplace_back(dx, dy);
  return d;   // 5 sel: pusat + 4 tetangga
}
}  // namespace

TEST(DiskCoverage, FullyCoveredDiskIsOne)
{
  std::vector<int> counts(25, 7);          // 5x5, semua sel berisi
  EXPECT_FLOAT_EQ(diskCoverage(counts, 5, 5, 2, 2, disk3x3()), 1.0f);
}

TEST(DiskCoverage, EmptyFrameIsZero)
{
  std::vector<int> counts(25, 0);
  EXPECT_FLOAT_EQ(diskCoverage(counts, 5, 5, 2, 2, disk3x3()), 0.0f);
}

TEST(DiskCoverage, ASinglePointMakesACellCovered)
{
  // Cakupan menanyakan "apakah sensor melihat ke sini", bukan "apakah aman".
  // Satu titik saja sudah bukti bahwa area itu terobservasi.
  std::vector<int> counts(25, 0);
  counts[2 * 5 + 2] = 1;                   // hanya sel pusat
  EXPECT_FLOAT_EQ(diskCoverage(counts, 5, 5, 2, 2, disk3x3()), 1.0f / 5.0f);
}

TEST(DiskCoverage, PartiallyCoveredDiskIsAFraction)
{
  std::vector<int> counts(25, 0);
  counts[2 * 5 + 2] = 3;                   // pusat
  counts[1 * 5 + 2] = 3;                   // atas
  counts[3 * 5 + 2] = 3;                   // bawah
  EXPECT_FLOAT_EQ(diskCoverage(counts, 5, 5, 2, 2, disk3x3()), 3.0f / 5.0f);
}

TEST(DiskCoverage, OutOfBoundsCellsCountAsUncovered)
{
  // Cakram di pojok: dua selnya jatuh di luar grid. Menghitungnya sebagai
  // "tercakup" akan membuat tepi grid tampak terobservasi padahal tidak.
  std::vector<int> counts(25, 9);
  EXPECT_FLOAT_EQ(diskCoverage(counts, 5, 5, 0, 0, disk3x3()), 3.0f / 5.0f);
}

TEST(DiskCoverage, EmptyOffsetListIsZeroNotADivisionByZero)
{
  std::vector<int> counts(25, 9);
  EXPECT_FLOAT_EQ(diskCoverage(counts, 5, 5, 2, 2, {}), 0.0f);
}

TEST(DiskCoverage, CentreOutsideTheGridIsZero)
{
  std::vector<int> counts(25, 9);
  EXPECT_FLOAT_EQ(diskCoverage(counts, 5, 5, 99, 99, disk3x3()), 0.0f);
}

TEST(DiskCoverage, NegativeCountsAreTreatedAsNoData)
{
  std::vector<int> counts(25, 0);
  counts[2 * 5 + 2] = -1;                  // tidak seharusnya terjadi, tapi
  EXPECT_FLOAT_EQ(diskCoverage(counts, 5, 5, 2, 2, disk3x3()), 0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Titik yang DITOLAK konsumen.
//
// Registry tahu kandidat mana yang terhalang, tapi tidak tahu kandidat mana
// yang baru saja GAGAL didarati. Yang tahu itu node misi. Ia menyampaikannya
// lewat TF `map -> reject_point`, dan landing_circle meneruskannya ke sini.
//
// Tanpa ini, seleksi sticky menempel pada titik yang baru saja ditinggalkan:
// misi naik lagi, memindai ulang, lalu mengunci titik yang sama persis. Dari
// luar itu terlihat sebagai "titik landing-nya mandek di posisi lama".
// ─────────────────────────────────────────────────────────────────────────────

TEST(LandingRegistryReject, ARejectedCandidateIsNotSelected)
{
  RegistryConfig cfg = defaultCfg();
  cfg.reject_radius_m = 1.5f;
  LandingRegistry reg(cfg);

  reg.observe({cand(2.f, 0.f, 0.f), cand(6.f, 0.f, 0.f)}, always(true), 0.1f);
  reg.setRejected(Eigen::Vector3f(2.f, 0.f, 0.f));

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 6.f);
}

TEST(LandingRegistryReject, OnlyCandidatesInsideTheRejectRadiusAreExcluded)
{
  RegistryConfig cfg = defaultCfg();
  cfg.reject_radius_m = 1.5f;
  LandingRegistry reg(cfg);

  reg.observe({cand(2.f, 0.f, 0.f), cand(4.f, 0.f, 0.f)}, always(true), 0.1f);
  reg.setRejected(Eigen::Vector3f(4.f, 0.f, 0.f));

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 2.f);
}

TEST(LandingRegistryReject, ClearRejectedBringsTheCandidateBack)
{
  // Penolakan itu "aku baru saja gagal di sana", bukan vonis permanen. Kalau
  // penghalangnya pergi, penilaian registry sendiri yang lebih layak dipakai.
  RegistryConfig cfg = defaultCfg();
  cfg.reject_radius_m = 1.5f;
  LandingRegistry reg(cfg);

  reg.observe({cand(2.f, 0.f, 0.f), cand(6.f, 0.f, 0.f)}, always(true), 0.1f);
  reg.setRejected(Eigen::Vector3f(2.f, 0.f, 0.f));
  reg.clearRejected();

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 2.f);
}

TEST(LandingRegistryReject, NullptrWhenEverythingIsRejected)
{
  RegistryConfig cfg = defaultCfg();
  cfg.reject_radius_m = 3.0f;
  LandingRegistry reg(cfg);

  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);
  reg.setRejected(Eigen::Vector3f(2.f, 0.f, 0.f));

  EXPECT_EQ(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistryReject, StickySelectionAbandonsATargetThatBecomesRejected)
{
  // Inti perbaikannya: sticky tidak boleh membuat TF menempel pada titik yang
  // baru saja ditinggalkan node misi.
  RegistryConfig cfg = defaultCfg();
  cfg.sticky_target   = true;
  cfg.reject_radius_m = 1.5f;
  LandingRegistry reg(cfg);

  reg.observe({cand(2.f, 0.f, 0.f), cand(6.f, 0.f, 0.f)}, always(true), 0.1f);
  reg.setRejected(Eigen::Vector3f(2.f, 0.f, 0.f));

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(2.f, 0.f, 0.f), true, true);
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 6.f);
}

TEST(LandingRegistryReject, NoRejectionSetChangesNothing)
{
  RegistryConfig cfg = defaultCfg();
  cfg.reject_radius_m = 1.5f;
  LandingRegistry reg(cfg);

  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);

  EXPECT_NE(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistryReject, ZeroRadiusDisablesTheExclusion)
{
  RegistryConfig cfg = defaultCfg();
  cfg.reject_radius_m = 0.0f;
  LandingRegistry reg(cfg);

  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);
  reg.setRejected(Eigen::Vector3f(2.f, 0.f, 0.f));

  EXPECT_NE(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistryReject, RejectionIsMeasuredIn3d)
{
  // Berbeda dari select_radius_m yang mendatar: acuannya di sini adalah titik
  // pendaratan itu sendiri, bukan posisi terbang, jadi ketinggian ikut berarti.
  RegistryConfig cfg = defaultCfg();
  cfg.reject_radius_m = 1.5f;
  LandingRegistry reg(cfg);

  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);
  reg.setRejected(Eigen::Vector3f(2.f, 0.f, 5.f));   // 5 m di atasnya

  EXPECT_NE(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

// ─────────────────────────────────────────────────────────────────────────────
// CommitGate — kumpulkan dulu, baru umumkan.
//
// landing_circle dulu menerbitkan TF `safety_point` sejak kandidat PERTAMA
// muncul: 1,5 detik setelah node menyala, saat registry baru berisi segelintir
// entri. TF itu lalu ikut berubah tiap kali ditemukan yang lebih baik, dan
// drone bereaksi terhadap keputusan yang belum matang.
//
// Gerbang ini menahan pengumuman sampai ada N detik pengamatan sungguhan.
// ─────────────────────────────────────────────────────────────────────────────

#include "segmentation_node/commit_gate.hpp"

TEST(CommitGate, ClockDoesNotRunBeforeTheFirstCandidate)
{
  // Jendela harus berarti N detik PENGAMATAN, bukan N detik menunggu.
  // Persepsi butuh 1,5-3 detik untuk mulai mengalir setelah dinyalakan.
  CommitGate gate(1.0f);

  for (int i = 0; i < 20; ++i)
    EXPECT_FALSE(gate.update(false, 0.5f));

  EXPECT_FLOAT_EQ(gate.elapsed_s(), 0.0f);
}

TEST(CommitGate, CommitsExactlyWhenTheWindowElapses)
{
  CommitGate gate(1.0f);

  EXPECT_FALSE(gate.update(true, 0.25f));   // 0.25
  EXPECT_FALSE(gate.update(true, 0.25f));   // 0.50
  EXPECT_FALSE(gate.update(true, 0.25f));   // 0.75
  EXPECT_TRUE (gate.update(true, 0.25f));   // 1.00
}

TEST(CommitGate, AGapInTheMiddleOfTheWindowPausesButDoesNotReset)
{
  // Keluaran GNG berayun keras antar-frame (243-3378 titik terukur), jadi satu
  // frame tanpa kandidat bukan alasan mengulang seluruh jendela.
  CommitGate gate(1.0f);

  gate.update(true, 0.5f);
  EXPECT_FALSE(gate.update(false, 0.5f));   // jeda, bukan reset
  EXPECT_FLOAT_EQ(gate.elapsed_s(), 0.5f);
  EXPECT_TRUE(gate.update(true, 0.5f));     // 1.00
}

TEST(CommitGate, ZeroWindowCommitsImmediately)
{
  // Perilaku lama, untuk jalur debug.
  CommitGate gate(0.0f);
  EXPECT_TRUE(gate.update(true, 0.0f));
}

TEST(CommitGate, NegativeWindowCommitsImmediately)
{
  CommitGate gate(-5.0f);
  EXPECT_TRUE(gate.update(true, 0.0f));
}

TEST(CommitGate, StaysCommittedWhileACandidateRemains)
{
  // "Kalau drone landing lalu terhalang maka drone akan naik lagi dan menuju
  // titik landing yang sudah ada" — selama registry masih menyodorkan kandidat
  // layak, penggantinya diumumkan seketika tanpa menunggu lagi.
  CommitGate gate(1.0f);
  gate.update(true, 1.0f);
  ASSERT_TRUE(gate.committed());

  for (int i = 0; i < 50; ++i)
    EXPECT_TRUE(gate.update(true, 0.1f));
}

TEST(CommitGate, ReturnsToCollectingWhenCandidatesRunOut)
{
  // "...kalau titik landingnya belum ada atau tersimpan maka akan scan lagi."
  CommitGate gate(1.0f);
  gate.update(true, 1.0f);
  ASSERT_TRUE(gate.committed());

  EXPECT_FALSE(gate.update(false, 0.1f));
  EXPECT_FALSE(gate.committed());
}

TEST(CommitGate, TheNextCommitDemandsAFullWindowAgain)
{
  CommitGate gate(1.0f);
  gate.update(true, 1.0f);
  gate.update(false, 0.1f);                 // kandidat habis -> mengumpulkan

  EXPECT_FALSE(gate.update(true, 0.25f));   // 0.25, bukan melanjutkan 1.00
  EXPECT_FALSE(gate.update(true, 0.25f));
  EXPECT_FALSE(gate.update(true, 0.25f));
  EXPECT_TRUE (gate.update(true, 0.25f));
}

TEST(CommitGate, ElapsedIsZeroedWhenCandidatesRunOut)
{
  CommitGate gate(1.0f);
  gate.update(true, 1.0f);
  gate.update(false, 0.1f);

  EXPECT_FLOAT_EQ(gate.elapsed_s(), 0.0f);
}

TEST(CommitGate, ResetGoesBackToCollecting)
{
  CommitGate gate(1.0f);
  gate.update(true, 1.0f);
  ASSERT_TRUE(gate.committed());

  gate.reset();
  EXPECT_FALSE(gate.committed());
  EXPECT_FLOAT_EQ(gate.elapsed_s(), 0.0f);
  EXPECT_FALSE(gate.update(true, 0.5f));
}

TEST(CommitGate, NegativeDtDoesNotRewindTheClock)
{
  CommitGate gate(1.0f);
  gate.update(true, 0.5f);
  gate.update(true, -5.0f);
  EXPECT_FLOAT_EQ(gate.elapsed_s(), 0.5f);
}

TEST(CommitGate, AZeroWindowNeverFallsBackToCollecting)
{
  // Jalur debug harus tetap sederhana: tidak ada jendela berarti tidak ada
  // keadaan mengumpulkan untuk dikembalikan.
  CommitGate gate(0.0f);
  gate.update(true, 0.1f);
  EXPECT_TRUE(gate.update(false, 0.1f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Pin — target yang sudah dikunci tidak boleh dibuang karena kapasitas.
//
// Diukur saat uji terbang: begitu drone mulai bergerak, registry meledak dari
// 14 ke 50 entri (mentok cap) dalam enam detik, karena grid terpaku pada frame
// kamera sehingga satu titik fisik yang sama jatuh ke sel berbeda dan
// centroid-nya bergeser 1,7-2,4 m — lebih jauh dari merge_dist_m.
//
// enforceCapacity() membuang berdasarkan last_seen. Kandidat baru membanjir
// dengan stempel segar, sementara target yang dikunci justru berhenti cocok
// dan stempelnya menua — jadi ia korban LRU pertama. Membekukan strike saja
// tidak menutup lubang ini.
// ─────────────────────────────────────────────────────────────────────────────

namespace {
// Banjiri registry dengan kandidat baru yang berjarak aman satu sama lain.
void flood(LandingRegistry& reg, int n, float from_x) {
  for (int i = 0; i < n; ++i)
    reg.observe({cand(from_x + 3.f * static_cast<float>(i), 0.f, 0.f)},
                always(false), 0.1f);
}
}  // namespace

TEST(LandingRegistryPin, PinnedEntrySurvivesCapacityPressure)
{
  RegistryConfig cfg = defaultCfg();
  cfg.max_candidates = 5;
  LandingRegistry reg(cfg);

  reg.observe({cand(0.f, 0.f, 0.f)}, always(false), 0.1f);
  reg.setPinned(Eigen::Vector3f(0.f, 0.f, 0.f));

  flood(reg, 20, 100.f);          // jauh lebih banyak dari kapasitas

  EXPECT_EQ(reg.size(), 5u);
  bool found = false;
  for (const auto& c : reg.all())
    if ((c.center - Eigen::Vector3f::Zero()).norm() < 0.5f) found = true;
  EXPECT_TRUE(found) << "target yang dipin ikut terbuang oleh LRU";
}

TEST(LandingRegistryPin, WithoutPinTheOldestEntryIsStillEvicted)
{
  // Regresi: LRU harus tetap bekerja seperti biasa untuk entri biasa.
  RegistryConfig cfg = defaultCfg();
  cfg.max_candidates = 5;
  LandingRegistry reg(cfg);

  reg.observe({cand(0.f, 0.f, 0.f)}, always(false), 0.1f);
  flood(reg, 20, 100.f);

  EXPECT_EQ(reg.size(), 5u);
  for (const auto& c : reg.all())
    EXPECT_GT(c.center.x(), 50.f) << "entri tertua seharusnya sudah dibuang";
}

TEST(LandingRegistryPin, ClearPinRestoresNormalEviction)
{
  RegistryConfig cfg = defaultCfg();
  cfg.max_candidates = 5;
  LandingRegistry reg(cfg);

  reg.observe({cand(0.f, 0.f, 0.f)}, always(false), 0.1f);
  reg.setPinned(Eigen::Vector3f(0.f, 0.f, 0.f));
  reg.clearPinned();

  flood(reg, 20, 100.f);

  for (const auto& c : reg.all())
    EXPECT_GT(c.center.x(), 50.f);
}

TEST(LandingRegistryPin, PinningAPointWithNoEntryIsHarmless)
{
  RegistryConfig cfg = defaultCfg();
  cfg.max_candidates = 5;
  LandingRegistry reg(cfg);

  reg.setPinned(Eigen::Vector3f(999.f, 999.f, 999.f));
  flood(reg, 20, 0.f);

  EXPECT_EQ(reg.size(), 5u);
}

TEST(LandingRegistryPin, OnlyOneEntryIsProtected)
{
  // Pin bukan pengecualian kapasitas umum: hanya SATU entri yang dilindungi,
  // sisanya tetap tunduk pada LRU sehingga registry tetap berbatas.
  RegistryConfig cfg = defaultCfg();
  cfg.max_candidates = 3;
  LandingRegistry reg(cfg);

  reg.observe({cand(0.f, 0.f, 0.f)}, always(false), 0.1f);
  reg.setPinned(Eigen::Vector3f(0.f, 0.f, 0.f));
  flood(reg, 20, 100.f);

  EXPECT_EQ(reg.size(), 3u);
}

// ─────────────────────────────────────────────────────────────────────────────
// dt_s == 0 membekukan seluruh jam registry.
//
// Inilah cara node menahan strike selama drone bertranslasi: kandidat yang
// berhenti cocok karena grid bergeser tidak boleh divonis terhalang.
// ─────────────────────────────────────────────────────────────────────────────

TEST(LandingRegistryFreeze, ZeroDtNeverBlocksAnEntry)
{
  RegistryConfig cfg = defaultCfg();          // block_after_s = 3.0
  LandingRegistry reg(cfg);

  reg.observe({cand(0.f, 0.f, 0.f)}, always(true), 0.1f);

  // Seratus frame "seharusnya terlihat tapi tidak muncul", jam beku.
  for (int i = 0; i < 100; ++i)
    reg.observe({}, always(true), 0.0f);

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_TRUE(reg.all()[0].selectable);
  EXPECT_NE(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistryFreeze, ZeroDtNeverErasesAnEntry)
{
  RegistryConfig cfg = defaultCfg();          // stale_after_s = 30.0
  LandingRegistry reg(cfg);

  reg.observe({cand(0.f, 0.f, 0.f)}, always(true), 0.1f);
  for (int i = 0; i < 1000; ++i)
    reg.observe({}, always(true), 0.0f);

  EXPECT_EQ(reg.size(), 1u);
}

TEST(LandingRegistryFreeze, NewDetectionsStillRegisterWhileFrozen)
{
  // Beku hanya menghentikan JAM, bukan pengamatan: kandidat baru tetap
  // terdaftar supaya peta terus bertambah selama drone bergerak.
  RegistryConfig cfg = defaultCfg();
  LandingRegistry reg(cfg);

  reg.observe({cand(0.f, 0.f, 0.f)}, always(true), 0.0f);
  reg.observe({cand(5.f, 0.f, 0.f)}, always(true), 0.0f);

  EXPECT_EQ(reg.size(), 2u);
}

// ─────────────────────────────────────────────────────────────────────────────
// addBlockedTime — bukti langsung dari konsumen, kebal terhadap pembekuan.
//
// observe() membekukan jamnya (dt_s = 0) selama drone bertranslasi, karena
// entri berhenti cocok akibat grid bergeser 1,7-2,4 m mengikuti kamera. Itu
// benar untuk pencocokan identitas — tapi ia ikut mematikan deteksi penghalang
// atas titik yang SUDAH DIKUNCI, yang koordinatnya justru sudah diketahui dan
// tidak butuh pencocokan sama sekali.
//
// Terukur: 27 dari 27 frame selama DESCEND berjalan beku. Sepanjang penurunan
// itu tidak ada objek yang bisa terdeteksi masuk.
// ─────────────────────────────────────────────────────────────────────────────

TEST(LandingRegistryDirectBlock, BlocksTheMatchingEntry)
{
  RegistryConfig cfg = defaultCfg();          // block_after_s = 3.0
  LandingRegistry reg(cfg);
  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);

  EXPECT_TRUE(reg.addBlockedTime(Eigen::Vector3f(2.f, 0.f, 0.f), 3.0f));

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_FALSE(reg.all()[0].selectable);
  EXPECT_EQ(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistryDirectBlock, WorksWhileTheObserveClockIsFrozen)
{
  // Inti perbaikannya: bukti langsung tidak boleh ikut beku.
  RegistryConfig cfg = defaultCfg();
  LandingRegistry reg(cfg);
  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);

  for (int i = 0; i < 50; ++i)
    reg.observe({}, always(true), 0.0f);      // beku: tidak ada strike
  ASSERT_TRUE(reg.all()[0].selectable);

  reg.addBlockedTime(Eigen::Vector3f(2.f, 0.f, 0.f), 3.0f);
  EXPECT_FALSE(reg.all()[0].selectable);
}

TEST(LandingRegistryDirectBlock, BelowTheThresholdItStaysSelectable)
{
  RegistryConfig cfg = defaultCfg();
  LandingRegistry reg(cfg);
  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);

  reg.addBlockedTime(Eigen::Vector3f(2.f, 0.f, 0.f), 2.9f);

  EXPECT_TRUE(reg.all()[0].selectable);
}

TEST(LandingRegistryDirectBlock, NegativeSecondsRecoverAndClampAtZero)
{
  RegistryConfig cfg = defaultCfg();
  LandingRegistry reg(cfg);
  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);

  reg.addBlockedTime(Eigen::Vector3f(2.f, 0.f, 0.f), 3.0f);
  ASSERT_FALSE(reg.all()[0].selectable);

  reg.addBlockedTime(Eigen::Vector3f(2.f, 0.f, 0.f), -100.f);
  EXPECT_TRUE(reg.all()[0].selectable);
  EXPECT_FLOAT_EQ(reg.all()[0].blocked_s, 0.f);
}

TEST(LandingRegistryDirectBlock, ReturnsFalseWhenNothingMatches)
{
  RegistryConfig cfg = defaultCfg();          // merge_dist_m = 1.0
  LandingRegistry reg(cfg);
  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);

  EXPECT_FALSE(reg.addBlockedTime(Eigen::Vector3f(40.f, 40.f, 0.f), 3.0f));
  EXPECT_TRUE(reg.all()[0].selectable);
}

TEST(LandingRegistryDirectBlock, LeavesOtherEntriesAlone)
{
  RegistryConfig cfg = defaultCfg();
  LandingRegistry reg(cfg);
  reg.observe({cand(2.f, 0.f, 0.f), cand(8.f, 0.f, 0.f)}, always(true), 0.1f);

  reg.addBlockedTime(Eigen::Vector3f(2.f, 0.f, 0.f), 3.0f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 8.f)
      << "seleksi harus berpindah ke kandidat tersimpan berikutnya";
}

TEST(LandingRegistryDirectBlock, StickySelectionLetsGoOfADirectlyBlockedTarget)
{
  // Rantai lengkapnya: penjaga cakram memblokir -> sticky melepas -> TF
  // berubah -> node misi membatalkan dan naik.
  RegistryConfig cfg = defaultCfg();
  cfg.sticky_target = true;
  LandingRegistry reg(cfg);
  reg.observe({cand(2.f, 0.f, 0.f), cand(8.f, 0.f, 0.f)}, always(true), 0.1f);

  reg.addBlockedTime(Eigen::Vector3f(2.f, 0.f, 0.f), 3.0f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(2.f, 0.f, 0.f), true, true);
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 8.f);
}

TEST(LandingRegistryDirectBlock, AccumulatesAcrossCalls)
{
  RegistryConfig cfg = defaultCfg();
  LandingRegistry reg(cfg);
  reg.observe({cand(2.f, 0.f, 0.f)}, always(true), 0.1f);

  for (int i = 0; i < 5; ++i)
    reg.addBlockedTime(Eigen::Vector3f(2.f, 0.f, 0.f), 0.5f);   // 2.5 s
  EXPECT_TRUE(reg.all()[0].selectable);

  reg.addBlockedTime(Eigen::Vector3f(2.f, 0.f, 0.f), 0.5f);     // 3.0 s
  EXPECT_FALSE(reg.all()[0].selectable);
}

// ─────────────────────────────────────────────────────────────────────────────
// countPointsAbove — bukti POSITIF adanya penghalang.
//
// Percobaan sebelumnya mencari LUBANG di /plane_cpu dan gagal: lubang bisa
// berarti "ada yang menghalangi" atau "frame ini kebetulan tidak menyentuhnya",
// dan keduanya tidak bisa dipisahkan. Terukur Safe: 0/2/1/5 di cakram yang
// sebenarnya bersih — enam salah alarm, tiga pendaratan baik dibatalkan.
//
// Penghalang yang berdiri di tanah bukan permukaan datar, jadi titik-titiknya
// masuk /outlier_cpu. Menghitung KEHADIRAN titik di atas cakram tidak bisa
// dipalsukan oleh data yang jarang: data jarang membuat titik hilang, bukan
// membuat titik muncul di tempat kosong.
// ─────────────────────────────────────────────────────────────────────────────

#include "segmentation_node/obstacle_probe.hpp"

namespace {
const Eigen::Vector3f kCenter(2.f, 3.f, -1.f);   // pusat permukaan pendaratan
}  // namespace

TEST(ObstacleProbe, CountsAPointStandingOnTheDisk)
{
  const std::vector<Eigen::Vector3f> pts{
    Eigen::Vector3f(2.f, 3.f, -0.2f)};           // 0,8 m di atas permukaan
  EXPECT_EQ(countPointsAbove(pts, kCenter, 0.65f, 0.2f, 3.0f), 1);
}

TEST(ObstacleProbe, IgnoresPointsOutsideTheHorizontalRadius)
{
  // Tingginya pas, tapi berdiri di sebelah cakram — bukan urusan kita.
  const std::vector<Eigen::Vector3f> pts{
    Eigen::Vector3f(2.f + 1.5f, 3.f, -0.2f)};
  EXPECT_EQ(countPointsAbove(pts, kCenter, 0.65f, 0.2f, 3.0f), 0);
}

TEST(ObstacleProbe, IgnoresPointsTooLowToBeAnObstacle)
{
  // Rumput dan derau permukaan. Menghitungnya akan membuat setiap zona
  // berumput terbaca sebagai terhalang.
  const std::vector<Eigen::Vector3f> pts{
    Eigen::Vector3f(2.f, 3.f, -0.95f)};          // hanya 5 cm di atas
  EXPECT_EQ(countPointsAbove(pts, kCenter, 0.65f, 0.2f, 3.0f), 0);
}

TEST(ObstacleProbe, IgnoresPointsFarAbove)
{
  // Dahan, atau drone lain yang lewat: ada di atas cakram, tapi bukan sesuatu
  // yang berdiri di zona pendaratan.
  const std::vector<Eigen::Vector3f> pts{
    Eigen::Vector3f(2.f, 3.f, 9.f)};             // 10 m di atas
  EXPECT_EQ(countPointsAbove(pts, kCenter, 0.65f, 0.2f, 3.0f), 0);
}

TEST(ObstacleProbe, IgnoresPointsBelowTheSurface)
{
  const std::vector<Eigen::Vector3f> pts{
    Eigen::Vector3f(2.f, 3.f, -4.f)};
  EXPECT_EQ(countPointsAbove(pts, kCenter, 0.65f, 0.2f, 3.0f), 0);
}

TEST(ObstacleProbe, BoundariesCountAsInside)
{
  // Nilainya sengaja pangkat dua supaya selisihnya EKSAK di float. Dengan
  // 0,65 m, `2.0f + 0.65f` tidak menghasilkan selisih tepat 0.65f, dan yang
  // diuji berubah jadi pembulatan float — bukan perilaku yang kita maksud.
  const Eigen::Vector3f c(2.f, 3.f, -1.f);
  const std::vector<Eigen::Vector3f> pts{
    Eigen::Vector3f(2.5f, 3.f, -0.75f),   // tepat di batas radius (0,5 m)
    Eigen::Vector3f(2.f, 3.f, 1.f)};      // tepat di batas atas    (2,0 m)
  EXPECT_EQ(countPointsAbove(pts, c, 0.5f, 0.25f, 2.0f), 2);
}

TEST(ObstacleProbe, EmptyCloudIsZero)
{
  EXPECT_EQ(countPointsAbove({}, kCenter, 0.65f, 0.2f, 3.0f), 0);
}

TEST(ObstacleProbe, CountsOnlyTheQualifyingPoints)
{
  // Jumlahnya harus tepat, bukan sekadar bukan-nol: ambang
  // obstacle_min_points bergantung padanya.
  const std::vector<Eigen::Vector3f> pts{
    Eigen::Vector3f(2.00f, 3.00f, -0.5f),   // ya
    Eigen::Vector3f(2.30f, 3.20f, -0.3f),   // ya  (jarak mendatar 0,36 m)
    Eigen::Vector3f(2.00f, 3.00f, -0.9f),   // tidak (cuma 0,1 m di atas)
    Eigen::Vector3f(5.00f, 3.00f, -0.5f),   // tidak (jauh mendatar)
    Eigen::Vector3f(2.10f, 2.95f, -0.4f)};  // ya
  EXPECT_EQ(countPointsAbove(pts, kCenter, 0.65f, 0.2f, 3.0f), 3);
}

TEST(ObstacleProbe, RadiusIsHorizontalOnlyNot3d)
{
  // Titik tinggi tepat di atas pusat harus dihitung. Kalau radiusnya diukur
  // 3D, penghalang yang tinggi justru lolos — persis kebalikan yang dimau.
  const std::vector<Eigen::Vector3f> pts{
    Eigen::Vector3f(2.f, 3.f, -1.f + 2.5f)};
  EXPECT_EQ(countPointsAbove(pts, kCenter, 0.65f, 0.2f, 3.0f), 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// 12. planeRoughness — kekasaran permukaan cakram.
//
// Yang diuji bukan sekadar "mengembalikan angka", melainkan tiga sifat yang
// membuat angka itu layak dipakai sebagai kriteria pendaratan: nol untuk
// bidang sempurna, TIDAK terpengaruh kemiringan, dan tidak sah bila titiknya
// terlalu sedikit untuk berarti apa-apa.
// ─────────────────────────────────────────────────────────────────────────────

namespace {

// Kisi n x n pada bidang z = a*x + b*y + c, ditambah simpangan dari `bump`.
std::vector<Eigen::Vector3f> gridPlane(
    int n, float a, float b, float c,
    const std::function<float(int, int)>& bump = {})
{
  std::vector<Eigen::Vector3f> pts;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const float x = static_cast<float>(i) * 0.25f;
      const float y = static_cast<float>(j) * 0.25f;
      float z = a * x + b * y + c;
      if (bump) z += bump(i, j);
      pts.emplace_back(x, y, z);
    }
  return pts;
}

}  // namespace

TEST(PlaneRoughness, PerfectlyFlatSurfaceIsZero)
{
  const auto r = planeRoughness(gridPlane(5, 0.f, 0.f, 2.f));
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.roughness_m, 0.f, 1e-5f);
}

// Kemiringan bukan kekasaran. Permukaan miring tapi mulus harus terbaca mulus;
// kalau tidak, setiap lereng landai akan tersingkir hanya karena ia miring.
TEST(PlaneRoughness, IsInvariantToSlope)
{
  const auto r = planeRoughness(gridPlane(5, 0.4f, -0.3f, 1.f));
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.roughness_m, 0.f, 1e-5f);
}

// Simpangan +/- A yang berselang-seling punya RMS tepat A terhadap bidang
// terbaiknya, jadi nilainya bisa dicocokkan dengan hitungan tangan.
TEST(PlaneRoughness, MatchesRmsOfAKnownDeviation)
{
  const float A = 0.05f;
  auto checker = [A](int i, int j) {
    return ((i + j) % 2 == 0) ? A : -A;
  };
  const auto r = planeRoughness(gridPlane(6, 0.f, 0.f, 0.f, checker));
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.roughness_m, A, 1e-4f);
}

TEST(PlaneRoughness, RougherSurfaceScoresHigher)
{
  auto amp = [](float a) {
    return [a](int i, int j) { return ((i + j) % 2 == 0) ? a : -a; };
  };
  const auto smooth = planeRoughness(gridPlane(6, 0.f, 0.f, 0.f, amp(0.01f)));
  const auto rough  = planeRoughness(gridPlane(6, 0.f, 0.f, 0.f, amp(0.08f)));

  ASSERT_TRUE(smooth.valid);
  ASSERT_TRUE(rough.valid);
  EXPECT_LT(smooth.roughness_m, rough.roughness_m);
}

// Tiga titik menentukan bidang secara persis: residualnya selalu nol, dan nol
// yang tidak berarti apa-apa lebih berbahaya daripada tidak ada jawaban.
TEST(PlaneRoughness, ThreePointsAreNotEnough)
{
  const std::vector<Eigen::Vector3f> pts = {
    {0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  EXPECT_FALSE(planeRoughness(pts).valid);
}

TEST(PlaneRoughness, EmptyInputIsInvalid)
{
  EXPECT_FALSE(planeRoughness({}).valid);
}

// Lantai empat titik tidak bisa ditembus dari luar.
TEST(PlaneRoughness, MinPointsIsClampedToFour)
{
  const std::vector<Eigen::Vector3f> pts = {
    {0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  EXPECT_FALSE(planeRoughness(pts, 1).valid);
}

TEST(PlaneRoughness, CallerCanDemandMoreThanFourPoints)
{
  const auto pts = gridPlane(3, 0.f, 0.f, 0.f);   // 9 titik
  EXPECT_TRUE (planeRoughness(pts, 9).valid);
  EXPECT_FALSE(planeRoughness(pts, 10).valid);
}

// ─────────────────────────────────────────────────────────────────────────────
// 13. landingScore — jarak + roughness jadi satu angka.
// ─────────────────────────────────────────────────────────────────────────────

TEST(LandingScore, PerfectCandidateScoresOne)
{
  ScoreWeights w;                       // 0.5 / 0.5, ref 10 m, rough_max 0.10
  EXPECT_FLOAT_EQ(landingScore(0.f, 0.f, 1, w), 1.f);
}

TEST(LandingScore, WorstCandidateScoresZero)
{
  ScoreWeights w;
  EXPECT_FLOAT_EQ(landingScore(w.dist_ref_m, w.rough_max_m, 1, w), 0.f);
}

TEST(LandingScore, TermsAreClampedBeyondTheirLimits)
{
  ScoreWeights w;
  EXPECT_FLOAT_EQ(landingScore(100.f, 5.f, 1, w), 0.f)
      << "skor tidak boleh negatif";
}

// Roughness yang BELUM PERNAH TERUKUR bernilai nol, bukan sempurna. Kalau
// tidak, cakram yang datanya terlalu miskin untuk diukur justru menang atas
// cakram yang sudah terbukti rata.
TEST(LandingScore, UnmeasuredRoughnessContributesNothing)
{
  ScoreWeights w;
  const float unmeasured = landingScore(0.f, 0.f, 0, w);
  const float measured   = landingScore(0.f, 0.f, 1, w);

  EXPECT_FLOAT_EQ(unmeasured, w.w_dist);
  EXPECT_GT(measured, unmeasured);
}

TEST(LandingScore, WeightsAreRespected)
{
  ScoreWeights w;
  w.w_dist  = 0.8f;
  w.w_rough = 0.2f;

  // Dekat tapi kasar vs jauh tapi rata; bobot condong ke jarak.
  const float near_rough = landingScore(0.f, w.rough_max_m, 1, w);
  const float far_smooth = landingScore(w.dist_ref_m, 0.f, 1, w);

  EXPECT_FLOAT_EQ(near_rough, 0.8f);
  EXPECT_FLOAT_EQ(far_smooth, 0.2f);
  EXPECT_GT(near_rough, far_smooth);
}

TEST(LandingScore, RoughnessCanOutweighDistanceWhenWeighted)
{
  ScoreWeights w;
  w.w_dist  = 0.2f;
  w.w_rough = 0.8f;

  EXPECT_LT(landingScore(0.f, w.rough_max_m, 1, w),
            landingScore(w.dist_ref_m, 0.f, 1, w));
}

TEST(LandingScore, DistanceTermIsLinear)
{
  EXPECT_FLOAT_EQ(distanceTerm(5.f, 10.f), 0.5f);
  EXPECT_FLOAT_EQ(distanceTerm(2.5f, 10.f), 0.75f);
}

TEST(LandingScore, ZeroReferenceDisablesTheDistanceTerm)
{
  EXPECT_FLOAT_EQ(distanceTerm(1.f, 0.f), 0.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// 14. Roughness di dalam registry — rata-rata berjalan dan pengaruhnya pada
//     seleksi.
// ─────────────────────────────────────────────────────────────────────────────

namespace {

LandingCandidate roughCand(float x, float y, float z, float roughness)
{
  LandingCandidate c = cand(x, y, z);
  c.roughness_m = roughness;
  c.rough_n     = 1;
  return c;
}

}  // namespace

TEST(LandingRegistryRoughness, AveragesAcrossObservations)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({roughCand(0, 0, 0, 0.02f)}, always(false), 0.5f);
  reg.observe({roughCand(0, 0, 0, 0.04f)}, always(false), 0.5f);
  reg.observe({roughCand(0, 0, 0, 0.06f)}, always(false), 0.5f);

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_EQ(reg.all()[0].rough_n, 3);
  EXPECT_NEAR(reg.all()[0].roughness_m, 0.04f, 1e-5f);
}

// Dua deteksi yang jatuh ke entri yang sama dalam SATU frame mencerminkan
// kepadatan grid, bukan dua pengamatan permukaan yang berbeda — aturan yang
// sama dengan `hits`.
TEST(LandingRegistryRoughness, TwoDetectionsInOneFrameCountAsOneSample)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({roughCand(0, 0, 0, 0.02f), roughCand(0.5f, 0, 0, 0.10f)},
              always(false), 0.5f);

  ASSERT_EQ(reg.size(), 1u) << "keduanya dalam merge_dist_m";
  EXPECT_EQ(reg.all()[0].rough_n, 1);
  EXPECT_NEAR(reg.all()[0].roughness_m, 0.02f, 1e-5f);
}

// Sampel tidak sah dilewati sepenuhnya. Memasukkannya sebagai nol akan menarik
// rata-rata ke bawah, sehingga cakram yang datanya jarang justru terlihat
// paling rata — persis kebalikan dari yang benar.
TEST(LandingRegistryRoughness, InvalidSamplesAreSkippedNotCountedAsZero)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({roughCand(0, 0, 0, 0.06f)}, always(false), 0.5f);
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);   // rough_n = 0

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_EQ(reg.all()[0].rough_n, 1);
  EXPECT_NEAR(reg.all()[0].roughness_m, 0.06f, 1e-5f);
}

TEST(LandingRegistryRoughness, NeverMeasuredStaysAtZeroSamples)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(0, 0, 0)}, always(false), 0.5f);

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_EQ(reg.all()[0].rough_n, 0);
}

// Inti permintaannya: yang menang bukan sekadar yang terdekat.
TEST(LandingRegistryRoughness, SmootherFartherCandidateBeatsNearerRoughOne)
{
  RegistryConfig cfg = defaultCfg();
  cfg.score.w_dist  = 0.3f;
  cfg.score.w_rough = 0.7f;

  LandingRegistry reg(cfg);
  // A: 1 m tapi kasar 0.10 m -> 0.3*0.9 + 0.7*0.0  = 0.27
  // B: 4 m tapi rata  0.00 m -> 0.3*0.6 + 0.7*1.0  = 0.88
  reg.observe({roughCand(1, 0, 0, 0.10f), roughCand(4, 0, 0, 0.0f)},
              always(false), 0.5f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 4.f)
      << "kandidat yang jauh lebih rata harus menang meski lebih jauh";
}

// Bobot condong ke jarak membalikkan hasilnya — buktinya bobot benar-benar
// dipakai, bukan kebetulan.
TEST(LandingRegistryRoughness, DistanceWinsWhenWeightedThatWay)
{
  RegistryConfig cfg = defaultCfg();
  cfg.score.w_dist  = 0.9f;
  cfg.score.w_rough = 0.1f;

  LandingRegistry reg(cfg);
  reg.observe({roughCand(1, 0, 0, 0.10f), roughCand(4, 0, 0, 0.0f)},
              always(false), 0.5f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 1.f);
}

// Skor hanya MENGURUTKAN yang sudah lolos. Kandidat sempurna yang terhalang
// tidak boleh menang atas kandidat biasa yang masih layak.
TEST(LandingRegistryRoughness, ScoreNeverOverridesTheHardFilters)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({roughCand(1, 0, 0, 0.0f), roughCand(8, 0, 0, 0.05f)},
              always(false), 0.5f);

  // Halangi hanya yang dekat sampai lewat block_after_s.
  auto near_only = [](const Eigen::Vector3f& c) { return c.x() < 5.f; };
  for (float t = 0.f; t < 3.5f; t += 0.5f)
    reg.observe({}, near_only, 0.5f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 8.f);
}

TEST(LandingRegistryRoughness, SelectRadiusOverridesDistanceReference)
{
  RegistryConfig cfg = defaultCfg();
  cfg.select_radius_m  = 4.0f;
  cfg.score.dist_ref_m = 100.0f;   // harus DIABAIKAN saat radius aktif
  cfg.score.w_dist     = 1.0f;
  cfg.score.w_rough    = 0.0f;

  LandingRegistry reg(cfg);
  reg.observe({cand(2, 0, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  // Dengan ref = 4 m: 1 - 2/4 = 0.5. Dengan ref = 100 m nilainya akan 0.98.
  EXPECT_NEAR(reg.scoreOf(*sel, Eigen::Vector3f::Zero()), 0.5f, 1e-5f);
}

// ─────────────────────────────────────────────────────────────────────────────
// 15. Sticky bersyarat — bug yang membuat target terkunci sejak frame pertama.
//
// Sticky sekarang argumen, bukan sifat registry: pemanggil menyalakannya hanya
// SETELAH gerbang commit terbuka. Tanpa itu, target terkunci pada kandidat yang
// kebetulan terlihat lebih dulu, dan sepuluh detik pengumpulan berikutnya
// mengisi daftar yang keputusannya sudah diambil.
// ─────────────────────────────────────────────────────────────────────────────

TEST(LandingRegistrySticky, TargetFollowsBestWhileStickyIsOff)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(8, 0, 0)}, always(false), 0.5f);

  // Kandidat jauh lebih baik muncul BELAKANGAN — persis keadaan detik-detik
  // awal jendela pengumpulan.
  reg.observe({cand(8, 0, 0), cand(1, 0, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(8, 0, 0), true, false);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 1.f)
      << "selama mengumpulkan, seleksi harus mengikuti kandidat terbaik";
}

TEST(LandingRegistrySticky, TargetIsHeldOnceStickyIsOn)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(8, 0, 0), cand(1, 0, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(8, 0, 0), true, true);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 8.f)
      << "setelah commit, target tidak boleh berpindah hanya karena ada yang "
         "skornya lebih tinggi";
}

// Peralihan off -> on adalah momen commit: yang terkunci harus argmax atas
// registry yang sudah penuh, bukan kandidat pertama yang dulu terlihat.
TEST(LandingRegistrySticky, SwitchingStickyOnLocksTheCurrentBest)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(8, 0, 0)}, always(false), 0.5f);

  Eigen::Vector3f target(8, 0, 0);

  // Jendela pengumpulan: kandidat lebih baik masuk, sticky masih mati.
  reg.observe({cand(8, 0, 0), cand(1, 0, 0)}, always(false), 0.5f);
  const LandingCandidate* a = reg.selectTarget(
      Eigen::Vector3f::Zero(), target, true, false);
  ASSERT_NE(a, nullptr);
  target = a->center;
  EXPECT_FLOAT_EQ(target.x(), 1.f);

  // Gerbang terbuka: sticky menyala, dan yang terkunci adalah hasil di atas.
  reg.observe({cand(8, 0, 0), cand(1, 0, 0), cand(0.2f, 0, 0)},
              always(false), 0.5f);
  const LandingCandidate* b = reg.selectTarget(
      Eigen::Vector3f::Zero(), target, true, true);
  ASSERT_NE(b, nullptr);
  EXPECT_FLOAT_EQ(b->center.x(), 1.f);
}

// Sticky bukan izin untuk bertahan di titik yang sudah tidak layak.
TEST(LandingRegistrySticky, StickyStillReleasesABlockedTarget)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(3, 0, 0), cand(9, 0, 0)}, always(false), 0.5f);

  auto near_only = [](const Eigen::Vector3f& c) { return c.x() < 5.f; };
  for (float t = 0.f; t < 3.5f; t += 0.5f)
    reg.observe({}, near_only, 0.5f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(3, 0, 0), true, true);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 9.f);
}

TEST(LandingRegistrySticky, ScoreHysteresisKeepsTargetWhenGainIsSmall)
{
  RegistryConfig cfg = defaultCfg();
  cfg.score.w_rough = 0.f;
  cfg.score.w_dist  = 1.f;      // ref 10 m -> selisih 0.1 m = skor 0.01

  LandingRegistry reg(cfg);
  reg.observe({cand(5.0f, 0, 0), cand(0, 4.9f, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(5.0f, 0, 0), true, false);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.x(), 5.0f)
      << "keuntungan 0.01 di bawah score_hysteresis 0.05";
}

TEST(LandingRegistrySticky, ScoreHysteresisYieldsWhenGainIsLarge)
{
  RegistryConfig cfg = defaultCfg();
  cfg.score.w_rough = 0.f;
  cfg.score.w_dist  = 1.f;      // selisih 2 m -> skor 0.2

  LandingRegistry reg(cfg);
  reg.observe({cand(5.0f, 0, 0), cand(0, 3.0f, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectTarget(
      Eigen::Vector3f::Zero(), Eigen::Vector3f(5.0f, 0, 0), true, false);

  ASSERT_NE(sel, nullptr);
  EXPECT_FLOAT_EQ(sel->center.y(), 3.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
// 16. clear() — membuang seluruh isi registry.
//
// Dipakai saat target yang sudah dikunci terpaksa dilepas. Sejak pencarian
// dibekukan setelah commit, kandidat yang tersisa saat itu dikumpulkan SEBELUM
// drone menukik — dari ketinggian yang sama sekali berbeda, sebagian berumur
// puluhan detik. Melompat ke salah satunya berarti mendarat di tempat yang
// penilaiannya sudah usang, jadi semuanya dibuang dan pengamatan dimulai lagi.
// ─────────────────────────────────────────────────────────────────────────────

TEST(LandingRegistryClear, EmptiesEveryEntry)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(1, 0, 0), cand(5, 0, 0), cand(9, 0, 0)},
              always(false), 0.5f);
  ASSERT_EQ(reg.size(), 3u);

  reg.clear();

  EXPECT_TRUE(reg.empty());
  EXPECT_EQ(reg.size(), 0u);
  EXPECT_EQ(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);
}

TEST(LandingRegistryClear, IsSafeOnAnAlreadyEmptyRegistry)
{
  LandingRegistry reg(defaultCfg());
  reg.clear();
  EXPECT_TRUE(reg.empty());
}

// Pin menunjuk entri yang sudah tidak ada; membiarkannya menyala akan membuat
// enforceCapacity melindungi hantu.
TEST(LandingRegistryClear, DropsThePin)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(1, 0, 0)}, always(false), 0.5f);
  reg.setPinned(Eigen::Vector3f(1, 0, 0));

  reg.clear();

  RegistryConfig cfg = defaultCfg();
  cfg.max_candidates = 2;
  LandingRegistry small(cfg);
  small.observe({cand(1, 0, 0)}, always(false), 0.5f);
  small.setPinned(Eigen::Vector3f(1, 0, 0));
  small.clear();
  small.observe({cand(2, 0, 0), cand(4, 0, 0), cand(6, 0, 0)},
                always(false), 0.5f);
  EXPECT_EQ(small.size(), 2u) << "kapasitas ditegakkan tanpa pin hantu";
}

// Penolakan konsumen ikut dibuang: ia menunjuk titik yang sudah tidak ada di
// registry, dan node misi tetap menyiarkan frame reject_point selama
// penolakannya masih berlaku.
TEST(LandingRegistryClear, DropsTheRejection)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(3, 0, 0)}, always(false), 0.5f);
  reg.setRejected(Eigen::Vector3f(3, 0, 0));
  ASSERT_EQ(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);

  reg.clear();
  reg.observe({cand(3, 0, 0)}, always(false), 0.5f);

  EXPECT_NE(reg.selectBest(Eigen::Vector3f::Zero()), nullptr)
      << "penolakan lama tidak boleh ikut bertahan melewati clear()";
}

// Yang dikumpulkan sesudahnya adalah pengamatan BARU, bukan lanjutan yang lama.
TEST(LandingRegistryClear, ReregistersFromScratchNotFromHistory)
{
  LandingRegistry reg(defaultCfg());
  for (int i = 0; i < 5; ++i)
    reg.observe({cand(3, 0, 0)}, always(false), 0.5f);
  ASSERT_EQ(reg.all()[0].hits, 5);

  reg.clear();
  reg.observe({cand(3, 0, 0)}, always(false), 0.5f);

  ASSERT_EQ(reg.size(), 1u);
  EXPECT_EQ(reg.all()[0].hits, 1) << "riwayat lama tidak boleh diwarisi";
  EXPECT_FLOAT_EQ(reg.all()[0].blocked_s, 0.f);
}

// Kandidat yang terhalang saat dibuang tidak boleh kembali sebagai entri yang
// masih terhalang — itu akan membuat titik yang sebenarnya bersih tetap
// tercoret sepanjang scan berikutnya.
TEST(LandingRegistryClear, ABlockedCandidateComesBackSelectable)
{
  LandingRegistry reg(defaultCfg());
  reg.observe({cand(3, 0, 0)}, always(false), 0.5f);
  for (float t = 0.f; t < 3.5f; t += 0.5f)
    reg.observe({}, always(true), 0.5f);
  ASSERT_EQ(reg.selectBest(Eigen::Vector3f::Zero()), nullptr);

  reg.clear();
  reg.observe({cand(3, 0, 0)}, always(false), 0.5f);

  const LandingCandidate* sel = reg.selectBest(Eigen::Vector3f::Zero());
  ASSERT_NE(sel, nullptr);
  EXPECT_TRUE(sel->selectable);
}
