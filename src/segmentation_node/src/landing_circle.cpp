// file: landing_circle.cpp
//
// Deteksi multi-kandidat titik pendaratan + seleksi terdekat ke drone.
// Lihat landing_circle.hpp untuk pembagian tanggung jawabnya.

#include <cstdio>

#include "segmentation_node/landing_circle.hpp"

#include "segmentation_node/grid_coverage.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>

LandingCircle::LandingCircle() : Node("landing_circle")
{
  // ── I/O ──────────────────────────────────────────────────────────────────
  input_topic_  = declare_parameter<std::string>("input_topic",  "/plane");
  output_topic_ = declare_parameter<std::string>("output_topic", "/safe_circle");

  // ── Frame TF ─────────────────────────────────────────────────────────────
  plane_frame_  = declare_parameter<std::string>("plane_frame",  "camera_link");
  odom_frame_   = declare_parameter<std::string>("odom_frame",   "map");
  safety_frame_ = declare_parameter<std::string>("safety_frame", "safety_point");
  base_frame_   = declare_parameter<std::string>("base_frame",   "base_link");

  // Frame opsional yang disiarkan konsumen di titik yang baru saja GAGAL
  // didarati. Registry tahu kandidat mana yang terhalang, tapi hanya konsumen
  // yang tahu kandidat mana yang sudah dicoba dan ditinggalkan.
  reject_frame_ = declare_parameter<std::string>("reject_frame", "reject_point");

  // Umur maksimum TF penolakan. Konsumen menyiarkannya terus-menerus selama
  // penolakannya masih berlaku, jadi frame yang basi berarti penolakan itu
  // sudah dicabut -- bukan berarti masih berlaku dan kebetulan telat.
  reject_timeout_s_ = declare_parameter<double>("reject_timeout_s", 1.0);

  // Selama drone BERTRANSLASI, jam registry dibekukan: tidak ada strike, tidak
  // ada penghapusan, tidak ada pemulihan.
  //
  // Diukur saat uji terbang: begitu drone mulai bergerak, registry meledak 14
  // -> 50 entri dalam enam detik dan `blocked` menumpuk sampai 29, lalu pulih
  // sendiri ke 3 begitu drone diam lagi. Grid terpaku pada frame kamera, jadi
  // satu titik fisik yang sama jatuh ke sel berbeda dan centroid-nya bergeser
  // 1,7-2,4 m — lebih jauh dari merge_dist_m — sehingga entri lama berhenti
  // cocok dan kena strike. Itu artefak gerakan, bukan penghalang, dan
  // memvonisnya membuat target yang baik dilepas di tengah pendekatan.
  motion_frame_    = declare_parameter<std::string>("motion_frame", "base_link");
  still_speed_mps_ = declare_parameter<double>("still_speed_mps", 0.15);

  // Penjaga cakram yang sudah dikunci: berapa lama ia harus terlihat terhalang
  // sebelum targetnya dilepas.
  //
  // Untuk titik yang SUDAH dipilih kita tidak butuh pencocokan identitas sama
  // sekali — koordinatnya sudah diketahui, jadi tiap frame ia bisa
  // diproyeksikan ulang ke grid dan diuji langsung terhadap peta obstacle. Itu
  // kebal terhadap pergeseran grid, yang justru alasan jam registry dibekukan
  // saat drone bergerak. Tanpa penjaga ini, deteksi penghalang mati sepanjang
  // approach dan descend: terukur 27 dari 27 frame penurunan berjalan beku.
  //
  // 1,5 s ~= 1-2 frame pada 0,79 Hz; pada laju turun 0,4 m/s berarti drone
  // turun 0,6 m lagi sebelum bereaksi. Cukup untuk menyaring satu frame buruk
  // (keluaran GNG berayun 243-3378 titik), tapi tetap tanggap.
  target_block_s_ = declare_parameter<double>("target_block_s", 1.5);

  // Penghalang dicari di /outlier_cpu -- keluaran GNG yang berisi titik BUKAN
  // bidang. Sesuatu yang berdiri di tanah tersaring keluar dari /plane_cpu dan
  // muncul di sana, jadi kehadirannya adalah bukti positif yang tidak bisa
  // dipalsukan oleh data yang jarang.
  obstacle_min_height_m_ =
    declare_parameter<double>("obstacle_min_height_m", 0.20);   // rumput/derau
  obstacle_max_height_m_ =
    declare_parameter<double>("obstacle_max_height_m", 3.00);   // dahan/wahana
  obstacle_min_points_ =
    declare_parameter<int>("obstacle_min_points", 3);
  outlier_max_age_s_ =
    declare_parameter<double>("outlier_max_age_s", 1.0);

  // ── Proyeksi bidang ───────────────────────────────────────────────────────
  plane_axes_   = declare_parameter<std::string>("plane_axes",   "yz");

  // ── Geometri area ─────────────────────────────────────────────────────────
  safe_diameter_ = declare_parameter<double>("safe_diameter",    1.3);

  // ── Grid & kriteria ───────────────────────────────────────────────────────
  // Default disetel untuk cloud KELUARAN GNG, bukan cloud depth mentah.
  // dbl_gng_cpu_node melakukan voxel-downsample 0.1 m lalu hanya menerbitkan
  // titik yang terpetakan ke simpul GNG datar, sehingga /plane_cpu hanya
  // sekitar 1.8k titik untuk area ~380 m² — kerapatannya ~5 titik/m².
  //
  // Pada grid_cell 0.23 m, satu sel hanya berisi ~0.25 titik (terukur: median
  // 1, maksimum 2), jadi nilai lama min_pts_per_cell=7 secara aritmetika
  // mustahil dipenuhi: nol sel lolos, dan tidak ada kandidat yang pernah
  // terbentuk. Sel yang kosong juga ditandai obstacle lalu didilatasi, yang
  // menghabiskan sisa kandidat sekalipun ambang titiknya dilonggarkan.
  //
  // grid_cell 0.6 m memberi ~1.7 titik per sel; dengan min_pts_per_cell=2
  // pipeline menghasilkan puluhan kandidat pada data yang sama.
  //
  // Konsekuensinya: pada grid_cell 0.6 m dan safe_diameter 1.3 m,
  // R_safe_cells jatuh ke 1 sehingga cakramnya hanya 5 sel — kandidatnya sahih
  // tapi geometrinya kasar. Untuk cloud yang lebih padat (mis. keluaran
  // plane_segmentation_ransac dari depth mentah), kecilkan lagi grid_cell dan
  // naikkan min_pts_per_cell.
  grid_cell_      = declare_parameter<double>("grid_cell",       0.6);
  min_pts_cell_   = declare_parameter<int>("min_pts_per_cell",   2);
  fill_ratio_req_ = declare_parameter<double>("safe_fill_ratio", 0.95);
  close_gaps_     = declare_parameter<bool>("close_gaps",        true);
  inflate_cells_  = declare_parameter<int>("inflate_cells",      1);

  // ── Multi-kandidat ────────────────────────────────────────────────────────
  // Dua kandidat dalam satu frame harus terpisah minimal sejauh ini, kalau
  // tidak setiap sel di tengah zona besar akan dilaporkan sebagai kandidat
  // tersendiri. Default = safe_diameter agar cakramnya tidak saling tumpang tindih.
  candidate_min_sep_m_ =
    declare_parameter<double>("candidate_min_sep_m", safe_diameter_);
  max_candidates_per_frame_ =
    declare_parameter<int>("max_candidates_per_frame", 10);

  // ── Registry lintas-frame ─────────────────────────────────────────────────
  RegistryConfig rc;
  // Dua deteksi dalam radius ini adalah titik pendaratan yang SAMA.
  // Diukur dari data terbang: perpindahan kandidat antar-frame p90 = 0.62 m
  // dengan ekor sampai 4.66 m. Nilai lama 0.5 m hanya menyerap 83% guncangan,
  // sehingga 18% deteksi tiap frame menjadi entri baru sementara entri lama
  // kena strike lalu mati — registry tidak pernah konvergen. 1.0 m menyerap
  // 97% dan tetap di bawah lantai pemisahan NMS (candidate_min_sep_m = 1.3 m),
  // jadi dua zona yang benar-benar berbeda tidak akan tergabung.
  rc.merge_dist_m = static_cast<float>(
    declare_parameter<double>("merge_dist_m", 1.0));

  // 0 = posisi dibekukan setelah pendaftaran pertama. Titik yang sudah
  // ditemukan tidak dikoreksi lagi supaya bisa dirujuk dengan stabil.
  rc.center_ema = static_cast<float>(
    declare_parameter<double>("center_ema", 0.0));

  const int reg_cap = declare_parameter<int>("max_registry_size", 50);
  rc.max_candidates = static_cast<size_t>(reg_cap > 1 ? reg_cap : 1);

  // Ambang berbasis WAKTU, bukan hitungan frame: /plane_cpu berjalan ~2 Hz
  // sedangkan backend GPU ~10 Hz, sehingga hitungan frame yang sama berarti
  // durasi nyata yang sangat berbeda. Setelan lama (3 frame) menghapus zona
  // pendaratan setelah hanya 1.3 detik — satu orang lewat sudah cukup.
  rc.block_after_s = static_cast<float>(
    declare_parameter<double>("block_after_s", 3.0));
  rc.recover_s = static_cast<float>(
    declare_parameter<double>("recover_s", 1.0));
  rc.stale_after_s = static_cast<float>(
    declare_parameter<double>("stale_after_s", 30.0));

  // Batas keras jarak dari acuan seleksi (base_frame). 0 = tanpa batas, yaitu
  // perilaku lama. Node misi menjalankan node ini dengan base_frame:=scan_center
  // (TF yang disiarkannya di waypoint), sehingga radius ini berarti "cari titik
  // aman di sekitar tempat yang diminta untuk dipindai", bukan di mana pun
  // kandidat kebetulan terkumpul sepanjang penerbangan.
  rc.select_radius_m = static_cast<float>(
    declare_parameter<double>("select_radius_m", 0.0));

  // Diuji terbang: acuan seleksi bergerak (drone terbang), sehingga "yang
  // terdekat" berubah terus tanpa ada apa pun yang terjadi pada target itu
  // sendiri. Konsumen TF `safety_point` tidak bisa membedakan "target saya
  // terhalang" dari "drone saya pindah", dan node misi membatalkan pendaratan
  // tiga kali berturut-turut karenanya. Dengan sticky, perpindahan TF punya
  // satu arti saja.
  // Disimpan di node, BUKAN di registry: sticky hanya berlaku setelah gerbang
  // commit terbuka, jadi keputusannya diambil per frame di langkah 15.
  // rc.sticky_target sengaja dibiarkan false supaya tidak ada dua sumber
  // kebenaran untuk satu perilaku.
  sticky_target_ = declare_parameter<bool>("sticky_target", false);

  rc.reject_radius_m = static_cast<float>(
    declare_parameter<double>("reject_radius_m", 1.5));

  // ── Skor seleksi ──────────────────────────────────────────────────────────
  // Kandidat tidak lagi diperingkat hanya dari jaraknya. Petak tanah terdekat
  // belum tentu petak yang paling layak didarati, dan sepuluh detik yang
  // dihabiskan drone untuk mengumpulkan kandidat tidak ada gunanya kalau yang
  // dibandingkan cuma jarak. Lihat landing_score.hpp.
  //
  // Bobot 50/50 adalah titik awal yang netral, bukan hasil pengukuran: belum
  // ada data terbang tentang seberapa jauh roughness membedakan kandidat di
  // dunia ini. `roughness_m` per kandidat ikut diterbitkan ke
  // /landing_circle_stats justru supaya itu bisa diukur, bukan ditebak.
  rc.score.w_dist = static_cast<float>(
    declare_parameter<double>("score_w_dist", 0.5));
  rc.score.w_rough = static_cast<float>(
    declare_parameter<double>("score_w_rough", 0.5));

  // Roughness yang membuat suku permukaan bernilai nol. 0,10 m = simpangan RMS
  // 10 cm terhadap bidang terbaik.
  rc.score.rough_max_m = static_cast<float>(
    declare_parameter<double>("rough_max_m", 0.10));

  // Hanya dipakai bila select_radius_m mati; kalau radius itu aktif, dialah
  // penormalisasi jaraknya, karena di sanalah batas kerasnya memang berada.
  rc.score.dist_ref_m = static_cast<float>(
    declare_parameter<double>("score_dist_ref_m", 10.0));

  // Margin skor sebelum target berpindah, hanya jalur non-sticky.
  // Menggantikan retarget_hysteresis_m yang bersatuan meter — yang
  // dibandingkan sekarang skor tak berdimensi.
  rc.score_hysteresis = static_cast<float>(
    declare_parameter<double>("score_hysteresis", 0.05));

  // Titik minimum agar residual bidang berarti; tiga titik menentukan bidang
  // secara persis sehingga residualnya selalu nol. Lihat plane_roughness.hpp.
  roughness_min_points_ = declare_parameter<int>("roughness_min_points", 4);

  // Berhenti mencari begitu keputusannya diumumkan. Seleksi sticky tidak akan
  // berpindah ke kandidat yang ditemukan sesudah itu, jadi mencarinya hanya
  // membakar CPU -- dan mengumpulkan kandidat dari ketinggian yang terus
  // berubah lalu menyimpannya seolah setara dengan yang dikumpulkan saat
  // hover. Lihat cabang beku di cbCloud.
  freeze_after_commit_ = declare_parameter<bool>("freeze_after_commit", true);

  block_after_s_   = rc.block_after_s;
  reject_radius_m_ = rc.reject_radius_m;

  registry_ = std::make_unique<LandingRegistry>(rc);

  // Kumpulkan dulu, baru umumkan. Node ini dulu menerbitkan TF `safety_point`
  // sejak kandidat pertama muncul -- 1,5 detik setelah menyala pada uji
  // terbang -- lalu memindahkannya tiap kali menemukan yang lebih baik,
  // sehingga drone mengejar keputusan yang belum matang. 0 = perilaku lama.
  const double commit_after_s = declare_parameter<double>("commit_after_s", 10.0);
  commit_gate_ = std::make_unique<CommitGate>(
    static_cast<float>(commit_after_s));


  // Bagian cakram yang harus benar-benar tersentuh data frame ini sebelum
  // kandidat lama yang tidak terdeteksi boleh dianggap terhalang.
  //
  // 0.6 diambil dari data terbang: /plane_cpu berayun 243-3378 titik per
  // frame dan cakram terpilih kosong sama sekali pada 26% frame. Nilai 0
  // memulihkan perilaku lama (strike tanpa memandang cakupan).
  observe_coverage_min_ =
    declare_parameter<double>("observe_coverage_min", 0.6);

  // ── TF ────────────────────────────────────────────────────────────────────
  tf_buffer_      = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // ── Subs & pubs ───────────────────────────────────────────────────────────
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LandingCircle::cbCloud, this, std::placeholders::_1));

  // Cloud outlier hanya disimpan; penilaiannya menumpang siklus /plane_cpu
  // supaya keduanya berasal dari frame masukan yang sama.
  sub_outlier_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    declare_parameter<std::string>("outlier_topic", "/outlier_cpu"),
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr m) {
      last_outlier_ = m;
    });

  pub_              = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 1);
  pub_center_point_ = create_publisher<geometry_msgs::msg::PointStamped>(
                        "/safe_circle_center_coords", 1);
  pub_stats_        = create_publisher<std_msgs::msg::String>(
                        "/landing_circle_stats", 10);
  pub_markers_      = create_publisher<visualization_msgs::msg::MarkerArray>(
                        "/landing_candidates", 1);

  RCLCPP_INFO(get_logger(),
    "LandingCircle: plane_frame=%s base=%s axes=%s safe_d=%.2fm grid=%.2fm "
    "fill>=%.2f pts/cell>=%d inflate=%d | candidates(sep=%.2fm max=%d) "
    "registry(merge=%.2fm cap=%zu ema=%.2f block=%.1fs recover=%.1fs "
    "stale=%.1fs cover>=%.2f) select(radius=%.2fm sticky-after-commit=%s "
    "score[w_dist=%.2f w_rough=%.2f rough_max=%.2fm dist_ref=%.2fm "
    "hyst=%.2f min_pts=%d] reject=%.2fm via '%s' commit=%.1fs) "
    "-> dynamic TF '%s' in '%s'",
    plane_frame_.c_str(), base_frame_.c_str(), plane_axes_.c_str(),
    safe_diameter_, grid_cell_, fill_ratio_req_, min_pts_cell_, inflate_cells_,
    candidate_min_sep_m_, max_candidates_per_frame_,
    rc.merge_dist_m, rc.max_candidates, rc.center_ema,
    rc.block_after_s, rc.recover_s, rc.stale_after_s, observe_coverage_min_,
    rc.select_radius_m, sticky_target_ ? "yes" : "no",
    rc.score.w_dist, rc.score.w_rough, rc.score.rough_max_m,
    rc.score.dist_ref_m, rc.score_hysteresis, roughness_min_points_,
    rc.reject_radius_m, reject_frame_.c_str(),
    commit_after_s,
    safety_frame_.c_str(), odom_frame_.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// releaseTarget — lepaskan target terkunci dan mulai lagi dari nol.
// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::releaseTarget(const char* reason)
{
  RCLCPP_WARN(get_logger(),
    "Melepas titik pendaratan (%.2f, %.2f, %.2f): %s. Registry dikosongkan; "
    "pengumpulan dimulai ulang dari nol.",
    selected_center_.x(), selected_center_.y(), selected_center_.z(), reason);

  // Seluruh registry dibuang, bukan hanya entri ini. Kandidat yang tersisa
  // dikumpulkan sebelum drone menukik, dari ketinggian yang sama sekali
  // berbeda; melompat ke salah satunya berarti mendarat di tempat yang
  // penilaiannya sudah usang. Frame `reject_point` dari node misi tetap
  // mencegah titik yang baru gagal ini didaftarkan ulang.
  registry_->clear();
  commit_gate_->reset();
  has_selection_    = false;
  target_blocked_s_ = 0.f;
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const auto t_start = std::chrono::high_resolution_clock::now();

  // ── 0) Konversi cloud ─────────────────────────────────────────────────────
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input cloud empty");
    publishSafeCloud(*cloud, {}, false, 0.f, 0.f, 0.f, msg->header);
    FrameStats fs;
    fs.stamp_sec     = msg->header.stamp.sec;
    fs.stamp_nanosec = msg->header.stamp.nanosec;
    publishStats(t_start, fs);
    return;
  }

  const size_t total_pts = cloud->size();

  // ── 1) Ambil TF plane <- cloud ────────────────────────────────────────────
  const std::string cloud_frame = msg->header.frame_id;
  geometry_msgs::msg::TransformStamped T_plane_from_cloud;
  try {
    T_plane_from_cloud = tf_buffer_->lookupTransform(
      plane_frame_, cloud_frame, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No TF %s <- %s: %s", plane_frame_.c_str(), cloud_frame.c_str(), e.what());
    return;
  }

  const Eigen::Isometry3d Xpc = tf2::transformToEigen(T_plane_from_cloud);

  // ── 2) Transform semua titik → plane_frame ────────────────────────────────
  std::vector<Eigen::Vector3d>          P_plane(total_pts);
  std::vector<std::pair<float, float>>  uv(total_pts);
  size_t valid_pts = 0;

  float umin = +FLT_MAX, vmin = +FLT_MAX,
        umax = -FLT_MAX, vmax = -FLT_MAX;

  for (size_t i = 0; i < total_pts; ++i) {
    const auto& p = (*cloud)[i];

    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      ++valid_pts;

    const Eigen::Vector3d pp = Xpc * Eigen::Vector3d(p.x, p.y, p.z);
    P_plane[i] = pp;

    float u, v;
    pickUV(pp, u, v);
    uv[i] = {u, v};

    umin = std::min(umin, u);  umax = std::max(umax, u);
    vmin = std::min(vmin, v);  vmax = std::max(vmax, v);
  }

  // ── 12) TF map <-> plane ──────────────────────────────────────────────────
  // Kandidat harus dipindah ke frame map supaya bisa diakumulasi lintas frame.
  Eigen::Isometry3d Xop;   // map <- plane
  try {
    Xop = tf2::transformToEigen(tf_buffer_->lookupTransform(
      odom_frame_, plane_frame_, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1)));
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No TF %s <- %s: %s", odom_frame_.c_str(), plane_frame_.c_str(), e.what());
    publishSafeCloud(*cloud, uv, false, 0.f, 0.f, 0.f, msg->header);
    FrameStats fs; fs.valid_pts = valid_pts; fs.total_pts = total_pts;
    fs.stamp_sec = msg->header.stamp.sec;
    fs.stamp_nanosec = msg->header.stamp.nanosec;
    publishStats(t_start, fs);
    return;
  }
  const Eigen::Isometry3d Xpo = Xop.inverse();   // plane <- map

  // ── Mode beku ─────────────────────────────────────────────────────────────
  // Begitu keputusan diumumkan, seleksi sticky tidak akan berpindah ke
  // kandidat mana pun yang ditemukan sesudahnya. Mencarinya hanya membakar
  // CPU pada mesin yang waktu komputasinya sedang diukur, dan mengumpulkan
  // kandidat dari ketinggian yang terus berubah lalu menyimpannya seolah
  // setara dengan yang dikumpulkan saat hover.
  //
  // Yang TETAP jalan saat beku: proyeksi titik (agar cakram hijau tetap
  // tergambar), penjaga cakram terkunci, cek penolakan, seleksi, dan
  // penerbitan TF. TF-nya wajib: node misi membatalkan pendaratan bila
  // `safety_point` basi lebih dari target_timeout_s.
  const bool frozen =
    freeze_after_commit_ && commit_gate_->committed() && has_selection_;

  // Jari-jari cakram dipakai juga saat beku, untuk memproyeksikan ulang titik
  // terpilih di langkah 16 -- jadi ia tidak boleh ikut masuk ke dalam blok
  // pencarian.
  const float r_safe = static_cast<float>(0.5 * safe_diameter_);

  // Kandidat frame ini, dan predikat "seharusnya terlihat" yang menyertainya.
  //
  // Saat beku keduanya sengaja dibiarkan kosong: tidak ada kandidat baru,
  // dan tidak ada satu pun entri lama yang boleh kena strike. Registry tidak
  // sedang diperbarui, jadi ia juga tidak boleh dihukum.
  std::vector<LandingCandidate> seen;
  std::function<bool(const Eigen::Vector3f&)> was_observable =
      [](const Eigen::Vector3f&) { return false; };

  if (!frozen) {
      // ── 3) Margin grid ────────────────────────────────────────────────────────
      constexpr float margin = 0.25f;
      umin -= margin;  umax += margin;
      vmin -= margin;  vmax += margin;

      // ── 4) Param grid ─────────────────────────────────────────────────────────
        const float cell   = static_cast<float>(grid_cell_);
      const int   cols   = std::max(1, static_cast<int>(std::ceil((umax - umin) / cell)));
      const int   rows   = std::max(1, static_cast<int>(std::ceil((vmax - vmin) / cell)));

      if (static_cast<long long>(rows) * cols > 10'000'000LL) {
        RCLCPP_WARN(get_logger(), "Grid too large (%dx%d). Increase grid_cell.", cols, rows);
        publishSafeCloud(*cloud, uv, false, 0.f, 0.f, 0.f, msg->header);
        FrameStats fs; fs.valid_pts = valid_pts; fs.total_pts = total_pts;
        fs.stamp_sec = msg->header.stamp.sec;
        fs.stamp_nanosec = msg->header.stamp.nanosec;
        publishStats(t_start, fs);
        return;
      }

      auto IDX       = [cols](int ix, int iy) { return iy * cols + ix; };
      auto in_bounds = [cols, rows](int x, int y) {
        return (x >= 0 && x < cols && y >= 0 && y < rows);
      };

      // ── 5) Histogram per cell ─────────────────────────────────────────────────
      std::vector<int> counts_raw(rows * cols, 0);
      for (const auto& q : uv) {
        const int ix = static_cast<int>(std::floor((q.first  - umin) / cell));
        const int iy = static_cast<int>(std::floor((q.second - vmin) / cell));
        if (in_bounds(ix, iy)) counts_raw[IDX(ix, iy)]++;
      }

      // ── 6) Close gaps (opsional) ──────────────────────────────────────────────
      std::vector<int> counts = counts_raw;
      if (close_gaps_) {
        std::vector<int> counts2 = counts;
        for (int y = 1; y < rows - 1; ++y) {
          for (int x = 1; x < cols - 1; ++x) {
            if (counts[IDX(x, y)]) continue;
            int sum = 0;
            for (int dy = -1; dy <= 1; ++dy)
              for (int dx = -1; dx <= 1; ++dx)
                sum += (counts[IDX(x + dx, y + dy)] > 0);
            if (sum >= 5) counts2[IDX(x, y)] = 1;
          }
        }
        counts.swap(counts2);
      }

      // ── 7) Disk offsets — cached, rebuild hanya jika param berubah ───────────
      const int R_safe_cells =
        std::max(1, static_cast<int>(std::round(r_safe / cell)));

      if (cached_R_safe_cells_ != R_safe_cells) {
        disk_offsets_safe_.clear();
        disk_offsets_safe_.reserve(
          static_cast<size_t>((2 * R_safe_cells + 1) * (2 * R_safe_cells + 1)));
        for (int dy = -R_safe_cells; dy <= R_safe_cells; ++dy)
          for (int dx = -R_safe_cells; dx <= R_safe_cells; ++dx)
            if (dx * dx + dy * dy <= R_safe_cells * R_safe_cells)
              disk_offsets_safe_.emplace_back(dx, dy);
        cached_R_safe_cells_ = R_safe_cells;
      }
      const int disk_area_safe = static_cast<int>(disk_offsets_safe_.size());

      // ── 8) Kandidat pusat berdasarkan fill ratio ──────────────────────────────
      std::vector<uint8_t> safe_center(rows * cols, 0);
      for (int iy = 0; iy < rows; ++iy) {
        for (int ix = 0; ix < cols; ++ix) {
          if (ix < R_safe_cells || ix >= cols - R_safe_cells ||
              iy < R_safe_cells || iy >= rows - R_safe_cells) continue;

          int filled_cells = 0;
          for (const auto& o : disk_offsets_safe_) {
            const int jx = ix + o.first, jy = iy + o.second;
            if (!in_bounds(jx, jy)) continue;
            if (counts[IDX(jx, jy)] >= min_pts_cell_) filled_cells++;
          }
          const float ratio =
            static_cast<float>(filled_cells) / static_cast<float>(disk_area_safe);
          if (ratio >= static_cast<float>(fill_ratio_req_))
            safe_center[IDX(ix, iy)] = 1;
        }
      }

      // ── 9) Peta obstacle + inflasi ────────────────────────────────────────────
      std::vector<uint8_t> obstacle(rows * cols, 0);
      for (int iy = 0; iy < rows; ++iy)
        for (int ix = 0; ix < cols; ++ix)
          obstacle[IDX(ix, iy)] =
            (counts_raw[IDX(ix, iy)] < min_pts_cell_) ? 1 : 0;

      auto dilate_once = [&](std::vector<uint8_t>& src) {
        std::vector<uint8_t> dst = src;
        for (int y = 0; y < rows; ++y) {
          for (int x = 0; x < cols; ++x) {
            if (src[IDX(x, y)]) continue;
            bool hit = false;
            for (int dy = -1; dy <= 1 && !hit; ++dy)
              for (int dx = -1; dx <= 1 && !hit; ++dx) {
                const int nx = x + dx, ny = y + dy;
                if (nx >= 0 && nx < cols && ny >= 0 && ny < rows &&
                    src[IDX(nx, ny)])
                  hit = true;
              }
            if (hit) dst[IDX(x, y)] = 1;
          }
        }
        src.swap(dst);
      };
      for (int it = 0; it < inflate_cells_; ++it) dilate_once(obstacle);

      // ── 10) Distance transform 8-neigh (BFS) ─────────────────────────────────
      const int INF = 1'000'000'000;
      std::vector<int> dist(rows * cols, INF);
      std::deque<std::pair<int, int>> dq;

      for (int iy = 0; iy < rows; ++iy)
        for (int ix = 0; ix < cols; ++ix)
          if (obstacle[IDX(ix, iy)]) {
            dist[IDX(ix, iy)] = 0;
            dq.emplace_back(ix, iy);
          }

      if (dq.empty())
        std::fill(dist.begin(), dist.end(), std::max(rows, cols) * 2);

      constexpr int dx8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
      constexpr int dy8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
      while (!dq.empty()) {
        auto [x, y] = dq.front();
        dq.pop_front();
        const int dcur = dist[IDX(x, y)];
        for (int k = 0; k < 8; ++k) {
          const int nx = x + dx8[k], ny = y + dy8[k];
          if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
          if (dist[IDX(nx, ny)] > dcur + 1) {
            dist[IDX(nx, ny)] = dcur + 1;
            dq.emplace_back(nx, ny);
          }
        }
      }

      // ── 11) Ekstraksi SEMUA kandidat (menggantikan pemilihan satu pemenang) ──
      const std::vector<GridCandidate> grid_cands = extractCandidates(
        safe_center, obstacle, dist, counts,
        rows, cols, cell, r_safe, umin, vmin);


      // ── 13) Centroid 3D tiap kandidat, lalu ke frame map ─────────────────────
      seen.reserve(grid_cands.size());
      for (const auto& gc : grid_cands) {
        const SafeResult r =
          computeDiskCentroid(*cloud, uv, P_plane, gc.cu, gc.cv, r_safe);
        if (!r.valid) continue;

        const Eigen::Vector3d c_map =
          Xop * Eigen::Vector3d(r.centroid[0], r.centroid[1], r.centroid[2]);

        LandingCandidate lc;
        lc.center      = c_map.cast<float>();
        lc.clearance_m = gc.clearance_m;
        lc.fill_ratio  = gc.fill_ratio;

        // rough_n = 1 berarti "frame ini menyumbang satu sampel sah"; registry
        // yang merata-ratakannya lintas frame. Sampel tidak sah dibiarkan
        // rough_n = 0 dan dilewati di sana — bukan dimasukkan sebagai nol.
        if (r.roughness_valid) {
          lc.roughness_m = r.roughness_m;
          lc.rough_n     = 1;
        }

        seen.push_back(lc);
      }

      // ── 14) Perbarui registry ─────────────────────────────────────────────────
      // Sebuah kandidat lama hanya boleh kena strike bila frame ini benar-benar
      // punya bukti tentangnya. Dua syarat, keduanya perlu:
      //
      //   GEOMETRI  pusatnya, diproyeksikan balik ke grid, jatuh di wilayah tempat
      //             sebuah cakram masih muat dievaluasi.
      //
      //   CAKUPAN   data frame ini benar-benar menyentuh cakram itu. Syarat kedua
      //             ini ditambahkan setelah uji terbang: tanpanya, frame yang
      //             kebetulan tidak menyentuh sebuah zona terbaca sebagai "zona
      //             itu terhalang", dan pada 0,79 Hz tiga frame sial berturut-turut
      //             sudah cukup memvonis zona yang sebenarnya aman. Lihat
      //             grid_coverage.hpp untuk angka ukurannya.
      //
      // Keselamatannya tidak berkurang: penghalang sungguhan MENGHASILKAN titik,
      // jadi cakupannya tetap tinggi dan strike-nya tetap terjadi. Yang berhenti
      // hanyalah menghukum zona yang memang tidak terlihat frame ini.
    was_observable = [&](const Eigen::Vector3f& c_map) {
        const Eigen::Vector3d p = Xpo * c_map.cast<double>();
        float u, v;
        pickUV(p, u, v);
        if (!(u >= umin + r_safe && u <= umax - r_safe &&
              v >= vmin + r_safe && v <= vmax - r_safe))
          return false;

        if (observe_coverage_min_ <= 0.0) return true;

        // Hitungan MENTAH, bukan hasil close-gaps: sel yang diisi close-gaps
        // adalah tebakan, dan tebakan bukan bukti bahwa sensor melihat ke sana.
        const int ix = static_cast<int>((u - umin) / cell);
        const int iy = static_cast<int>((v - vmin) / cell);
        return diskCoverage(counts_raw, rows, cols, ix, iy, disk_offsets_safe_)
               >= static_cast<float>(observe_coverage_min_);
      };
  }

  // Selisih waktu antar frame. Lompatan negatif atau sangat besar (simulasi
  // di-reset, atau clock sim melompat) diabaikan supaya tidak ada zona yang
  // langsung dianggap basi.
  const rclcpp::Time stamp(msg->header.stamp);
  float dt_s = 0.f;
  if (have_last_stamp_) {
    const double d = (stamp - last_stamp_).seconds();
    if (d > 0.0 && d < 5.0) dt_s = static_cast<float>(d);
  }
  last_stamp_      = stamp;
  have_last_stamp_ = true;

  // ── 13b) Apakah drone sedang bertranslasi? ───────────────────────────────
  // Diukur dari TF, bukan dari topik kecepatan PX4: node ini sudah memegang
  // buffer TF, dan `map -> base_link` sudah diterbitkan drone_kinematic.
  bool  moving    = false;
  float speed_mps = 0.f;
  Eigen::Vector3f drone_map_pos = Eigen::Vector3f::Zero();
  bool  have_drone_map = false;
  try {
    const auto M = tf_buffer_->lookupTransform(
      odom_frame_, motion_frame_, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
    const Eigen::Vector3f pos(
      static_cast<float>(M.transform.translation.x),
      static_cast<float>(M.transform.translation.y),
      static_cast<float>(M.transform.translation.z));
    const rclcpp::Time m_stamp(M.header.stamp);

    if (have_prev_motion_) {
      const double d_t = (m_stamp - prev_motion_stamp_).seconds();
      if (d_t > 1e-3) {
        speed_mps = static_cast<float>(
          (pos - prev_motion_pos_).norm() / d_t);
        moving = speed_mps > static_cast<float>(still_speed_mps_);
      }
    }
    drone_map_pos      = pos;
    have_drone_map     = true;
    prev_motion_pos_   = pos;
    prev_motion_stamp_ = m_stamp;
    have_prev_motion_  = true;
  } catch (const std::exception&) {
    // Tidak bisa menilai gerakan: perlakukan sebagai diam, yaitu perilaku
    // lama. Membekukan jam karena ketiadaan informasi akan membuat registry
    // berhenti menua tanpa alasan.
  }

  // Registry hanya diperbarui saat mencari. Saat beku ia dibiarkan apa adanya:
  // tidak ada entri baru, tidak ada strike, tidak ada penghapusan.
  if (!frozen)
    registry_->observe(seen, was_observable, moving ? 0.f : dt_s);

  // ── 14c) Penjaga cakram yang sudah dikunci ───────────────────────────────
  // Dijalankan SETELAH observe() supaya tidak dianulir olehnya, dan memakai
  // dt_s yang sebenarnya — bukan yang dibekukan. Inilah satu-satunya jalur
  // deteksi penghalang yang tetap hidup saat drone bergerak.
  int obstacle_points = 0;
  if (has_selection_ && commit_gate_->committed() && last_outlier_) {
    const double out_age =
      (now() - rclcpp::Time(last_outlier_->header.stamp)).seconds();

    if (out_age <= outlier_max_age_s_) {
      // Ke frame `map`: hanya di sana sumbu z benar-benar ke atas, dan
      // "berdiri di atas cakram" adalah pertanyaan tentang tinggi.
      pcl::PointCloud<pcl::PointXYZ> out_cloud;
      pcl::fromROSMsg(*last_outlier_, out_cloud);

      std::vector<Eigen::Vector3f> out_map;
      out_map.reserve(out_cloud.size());
      for (const auto& p : out_cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          continue;
        out_map.emplace_back(
          (Xop * Eigen::Vector3d(p.x, p.y, p.z)).cast<float>());
      }

      obstacle_points = countPointsAbove(
        out_map, selected_center_,
        static_cast<float>(safe_diameter_ * 0.5),
        static_cast<float>(obstacle_min_height_m_),
        static_cast<float>(obstacle_max_height_m_));

      target_blocked_s_ = (obstacle_points >= obstacle_min_points_)
        ? target_blocked_s_ + dt_s
        : std::max(0.f, target_blocked_s_ - dt_s);

      if (target_blocked_s_ >= static_cast<float>(target_block_s_)) {
        char why[128];
        std::snprintf(why, sizeof(why),
                      "%d titik outlier selama %.1fs",
                      obstacle_points, static_cast<double>(target_blocked_s_));

        if (freeze_after_commit_) {
          // Registry dibuang seluruhnya, bukan hanya entri ini. Saat beku,
          // kandidat yang tersisa dikumpulkan sebelum drone menukik -- dari
          // ketinggian yang sama sekali berbeda, sebagian berumur puluhan
          // detik. Melompat ke salah satunya berarti mendarat di tempat yang
          // penilaiannya sudah usang.
          releaseTarget(why);
        } else {
          // Perilaku lama: tandai entri ini saja dan biarkan seleksi pindah
          // ke kandidat tersimpan berikutnya.
          RCLCPP_WARN(get_logger(),
            "Penghalang di atas cakram terkunci (%.2f, %.2f, %.2f): %s -- "
            "melepas target.",
            selected_center_.x(), selected_center_.y(),
            selected_center_.z(), why);
          registry_->addBlockedTime(selected_center_, block_after_s_);
          target_blocked_s_ = 0.f;
        }
      }
    }
    // Cloud outlier basi: TIDAK ADA BUKTI. Jangan menambah maupun mengurangi --
    // prinsip yang sama dengan was_observable.
  } else {
    target_blocked_s_ = 0.f;
  }

  FrameStats fs;
  fs.stamp_sec      = msg->header.stamp.sec;
  fs.stamp_nanosec  = msg->header.stamp.nanosec;
  fs.valid_pts      = valid_pts;
  fs.total_pts      = total_pts;
  fs.num_candidates = registry_->size();
  fs.blocked_count  = registry_->blockedCount();

  // ── 14b) Titik yang ditolak konsumen ─────────────────────────────────────
  // Dibaca dari TF, bukan topik: konsumen sudah menyiarkan `scan_center` lewat
  // jalur yang sama, dan sebuah titik memang paling wajar dinyatakan sebagai
  // frame. Diambil pada waktu TERBARU, bukan pada stempel cloud: penolakan
  // adalah pernyataan tentang saat ini, dan cloud selalu tertinggal.
  try {
    const auto R = tf_buffer_->lookupTransform(
      odom_frame_, reject_frame_, tf2::TimePointZero);
    const double age = (now() - rclcpp::Time(R.header.stamp)).seconds();
    if (age <= reject_timeout_s_) {
      const Eigen::Vector3f rp(
        static_cast<float>(R.transform.translation.x),
        static_cast<float>(R.transform.translation.y),
        static_cast<float>(R.transform.translation.z));
      registry_->setRejected(rp);

      // Bila yang ditolak adalah target yang SEDANG dipakai, perlakukan sama
      // dengan penghalang: buang semuanya dan kumpulkan lagi. Tanpa ini,
      // seleksi hanya melompat ke kandidat beku berikutnya -- padahal itu
      // justru yang ingin dihindari.
      if (freeze_after_commit_ && has_selection_ &&
          (rp - selected_center_).norm() <= reject_radius_m_) {
        releaseTarget("ditolak node misi");
      }
    } else {
      registry_->clearRejected();
    }
  } catch (const std::exception&) {
    // Tidak ada frame penolakan sama sekali: keadaan normal, bukan kesalahan.
    registry_->clearRejected();
  }

  // ── 15) Posisi drone → pilih kandidat terdekat ───────────────────────────
  Eigen::Vector3f drone_pos = Eigen::Vector3f::Zero();
  bool have_drone = false;
  try {
    const auto T = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
    drone_pos = Eigen::Vector3f(
      static_cast<float>(T.transform.translation.x),
      static_cast<float>(T.transform.translation.y),
      static_cast<float>(T.transform.translation.z));
    have_drone = true;
    score_ref_ = drone_pos;
  } catch (const std::exception& e) {
    // Registry tetap diperbarui di atas; hanya seleksinya yang dilewati,
    // sehingga target terakhir dipertahankan alih-alih hilang.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No TF %s <- %s, keeping previous target: %s",
      odom_frame_.c_str(), base_frame_.c_str(), e.what());
  }

  if (have_drone) {
    // Sticky hanya SETELAH gerbang commit terbuka.
    //
    // Sebelumnya sticky menyala sejak frame pertama, dan itu bug: pada frame
    // pertama belum ada target sehingga seleksi mengambil kandidat terbaik
    // dari satu sapuan saja — sering cuma satu atau dua yang kebetulan
    // terlihat — lalu menahannya. Registry terus terisi selama sepuluh detik
    // berikutnya, tapi tidak satu pun kandidat baru itu pernah dipertimbangkan.
    // Jendela pengumpulan mengisi daftar yang keputusannya sudah diambil.
    //
    // committed() dibaca SEBELUM update() frame ini, jadi nilainya keadaan
    // frame sebelumnya. Itu justru yang benar: pada frame saat gerbang
    // terbuka, seleksi masih bebas, sehingga yang terkunci adalah skor
    // tertinggi atas registry yang sudah penuh. Urutannya tidak bisa dibalik
    // karena gerbang butuh has_selection_ yang justru hasil seleksi ini.
    const bool sticky_now = sticky_target_ && commit_gate_->committed();

    const LandingCandidate* sel = registry_->selectTarget(
      drone_pos, selected_center_, has_selection_, sticky_now);

    if (sel != nullptr) {
      selected_center_ = sel->center;
      has_selection_   = true;
    } else {
      has_selection_ = false;   // registry kosong — tidak ada target
    }
  }

  // Gerbang disuapi hasil SELEKSI, bukan registry_->size(): yang menentukan
  // adalah apakah ada kandidat yang benar-benar boleh dipilih setelah semua
  // saringan (terhalang, radius, penolakan). Registry berisi 50 entri yang
  // semuanya terhalang tetap berarti "tidak ada titik tersimpan", dan itu yang
  // memicu pengumpulan ulang.
  // Gerbang juga tidak boleh maju saat beku: jendela pengumpulan harus berisi
  // pengamatan dari pose yang sama, bukan detik-detik saat drone melintas.
  const bool committed = commit_gate_->update(has_selection_,
                                              moving ? 0.f : dt_s);
  fs.committed         = committed;
  fs.collect_elapsed_s = commit_gate_->elapsed_s();
  fs.frozen            = moving;
  fs.speed_mps         = speed_mps;
  fs.target_blocked_s  = target_blocked_s_;
  fs.search_frozen     = frozen;
  fs.obstacle_points   = obstacle_points;
  if (have_drone_map && has_selection_)
    fs.target_alt_m = drone_map_pos.z() - selected_center_.z();

  // Skor target terpilih, untuk log. Dicari lewat kesamaan posisi karena
  // selectTarget mengembalikan pointer yang sudah tidak dipegang di sini.
  if (has_selection_ && have_drone) {
    for (const auto& c : registry_->all()) {
      if ((c.center - selected_center_).squaredNorm() < 1e-6f) {
        fs.selected_score = registry_->scoreOf(c, drone_pos);
        break;
      }
    }
  }

  // Lindungi target yang sedang dipakai dari pembuangan LRU. Saat registry
  // membengkak oleh gerakan, entri inilah yang stempelnya paling menua.
  if (has_selection_) registry_->setPinned(selected_center_);
  else                registry_->clearPinned();

  // ── 16) Terbitkan hasil ───────────────────────────────────────────────────
  // Marker tetap terbit selama pengumpulan -- justru di jendela itulah operator
  // ingin melihat apa yang sedang dikumpulkan. Yang ditahan hanya pengumuman
  // keputusannya: TF, koordinat pusat, dan cakram terpilih.
  publishCandidateMarkers(msg->header.stamp);

  if (has_selection_ && committed) {
    fs.has_selection = true;
    if (have_drone)
      fs.selected_distance_m = (selected_center_ - drone_pos).norm();

    publishSafetyTF(selected_center_, msg->header.stamp);
    publishCenterCoordinates(selected_center_, msg->header.stamp);

    // Cakram yang diterbitkan adalah cakram terpilih — proyeksikan pusatnya
    // kembali ke (u,v) frame ini agar titik-titiknya bisa dikumpulkan.
    const Eigen::Vector3d c_plane = Xpo * selected_center_.cast<double>();
    float su, sv;
    pickUV(c_plane, su, sv);

    const SafeResult sel_disk =
      computeDiskCentroid(*cloud, uv, P_plane, su, sv, r_safe);
    fs.safe_size = sel_disk.size;

    publishSafeCloud(*cloud, uv, true, su, sv, r_safe, msg->header);
  } else {
    publishSafeCloud(*cloud, uv, false, 0.f, 0.f, 0.f, msg->header);
  }

  publishStats(t_start, fs);
}

// ─────────────────────────────────────────────────────────────────────────────
// extractCandidates — semua pusat cakram yang lolos, disaring dengan greedy NMS.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<GridCandidate> LandingCircle::extractCandidates(
    const std::vector<uint8_t>& safe_center,
    const std::vector<uint8_t>& obstacle,
    const std::vector<int>&     dist,
    const std::vector<int>&     counts,
    int rows, int cols,
    float cell, float r_safe,
    float umin, float vmin) const
{
  auto IDX = [cols](int ix, int iy) { return iy * cols + ix; };

  // Sel yang lolos fill-ratio DAN cakramnya sepenuhnya bebas obstacle.
  struct Scored { int ix, iy; float clearance_m; };
  std::vector<Scored> scored;

  for (int iy = 0; iy < rows; ++iy) {
    for (int ix = 0; ix < cols; ++ix) {
      if (!safe_center[IDX(ix, iy)]) continue;

      bool obstacle_free = true;
      for (const auto& o : disk_offsets_safe_) {
        const int jx = ix + o.first, jy = iy + o.second;
        if (jx < 0 || jx >= cols || jy < 0 || jy >= rows ||
            obstacle[IDX(jx, jy)]) {
          obstacle_free = false;
          break;
        }
      }
      if (!obstacle_free) continue;

      scored.push_back({ix, iy, dist[IDX(ix, iy)] * cell - r_safe});
    }
  }

  // Clearance tertinggi lebih dulu, supaya NMS mempertahankan yang terbaik
  // dari tiap gerombolan sel yang berdekatan.
  std::sort(scored.begin(), scored.end(),
            [](const Scored& a, const Scored& b) {
              return a.clearance_m > b.clearance_m;
            });

  const float sep    = static_cast<float>(candidate_min_sep_m_);
  const float sep2   = sep * sep;
  const size_t limit = static_cast<size_t>(std::max(1, max_candidates_per_frame_));
  const int disk_area = static_cast<int>(disk_offsets_safe_.size());

  std::vector<GridCandidate> out;
  out.reserve(limit);

  for (const auto& s : scored) {
    if (out.size() >= limit) break;

    const float cu = umin + (static_cast<float>(s.ix) + 0.5f) * cell;
    const float cv = vmin + (static_cast<float>(s.iy) + 0.5f) * cell;

    bool too_close = false;
    for (const auto& kept : out) {
      const float du = kept.cu - cu, dv = kept.cv - cv;
      if (du * du + dv * dv < sep2) { too_close = true; break; }
    }
    if (too_close) continue;

    // fill_ratio dihitung ulang hanya untuk kandidat yang diterima (≤ limit),
    // jauh lebih murah daripada menyimpan rasio setiap sel grid.
    int filled = 0;
    for (const auto& o : disk_offsets_safe_) {
      const int jx = s.ix + o.first, jy = s.iy + o.second;
      if (jx < 0 || jx >= cols || jy < 0 || jy >= rows) continue;
      if (counts[IDX(jx, jy)] >= min_pts_cell_) ++filled;
    }

    GridCandidate gc;
    gc.cu          = cu;
    gc.cv          = cv;
    gc.clearance_m = s.clearance_m;
    gc.fill_ratio  = (disk_area > 0)
                   ? static_cast<float>(filled) / static_cast<float>(disk_area)
                   : 0.f;
    out.push_back(gc);
  }

  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// computeDiskCentroid — centroid 3D titik-titik di dalam satu cakram (plane_frame).
// ─────────────────────────────────────────────────────────────────────────────
SafeResult LandingCircle::computeDiskCentroid(
    const pcl::PointCloud<pcl::PointXYZ>&       cloud,
    const std::vector<std::pair<float, float>>& uv,
    const std::vector<Eigen::Vector3d>&         P_plane,
    float cu, float cv, float r_safe) const
{
  SafeResult result;
  double sx = 0.0, sy = 0.0, sz = 0.0;
  const float r2 = r_safe * r_safe;

  // Titik cakram disalin sekali supaya roughness tidak perlu melintasi cloud
  // untuk kedua kalinya. Cakram berisi ~7 titik pada kerapatan /plane_cpu,
  // jadi salinannya tidak berarti apa-apa dibanding lintasan penuhnya.
  std::vector<Eigen::Vector3f> disk_pts;

  for (size_t i = 0; i < cloud.size(); ++i) {
    const float du = uv[i].first  - cu;
    const float dv = uv[i].second - cv;
    if (du * du + dv * dv > r2) continue;

    ++result.size;
    sx += P_plane[i].x();
    sy += P_plane[i].y();
    sz += P_plane[i].z();
    disk_pts.emplace_back(P_plane[i].cast<float>());
  }

  if (result.size > 0) {
    const double inv = 1.0 / static_cast<double>(result.size);
    result.centroid[0] = static_cast<float>(sx * inv);
    result.centroid[1] = static_cast<float>(sy * inv);
    result.centroid[2] = static_cast<float>(sz * inv);
    result.valid = true;
  }

  // Diukur di plane_frame, tidak dipindah ke `map` dulu: roughness tidak
  // berubah oleh rotasi maupun translasi.
  const RoughnessResult rr = planeRoughness(disk_pts, roughness_min_points_);
  result.roughness_m     = rr.roughness_m;
  result.roughness_valid = rr.valid;

  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// publishSafeCloud — titik-titik cakram terpilih, diwarnai hijau.
// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::publishSafeCloud(
    const pcl::PointCloud<pcl::PointXYZ>&       cloud,
    const std::vector<std::pair<float, float>>& uv,
    bool  have_center,
    float cu, float cv, float r_safe,
    const std_msgs::msg::Header& hdr)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr safe_rgb(
    new pcl::PointCloud<pcl::PointXYZRGB>);

  if (have_center) {
    const size_t est_cap = disk_offsets_safe_.size() *
                           static_cast<size_t>(std::max(1, min_pts_cell_)) * 2;
    safe_rgb->reserve(std::min(cloud.size(), est_cap));

    const float r2 = r_safe * r_safe;
    for (size_t i = 0; i < cloud.size(); ++i) {
      const float du = uv[i].first  - cu;
      const float dv = uv[i].second - cv;
      if (du * du + dv * dv > r2) continue;

      const auto& p = cloud[i];
      pcl::PointXYZRGB q;
      q.x = p.x;  q.y = p.y;  q.z = p.z;
      q.r = 30;   q.g = 255;  q.b = 30;
      safe_rgb->push_back(q);
    }
  }

  safe_rgb->width    = static_cast<uint32_t>(safe_rgb->size());
  safe_rgb->height   = 1;
  safe_rgb->is_dense = false;

  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*safe_rgb, out);
  out.header = hdr;
  pub_->publish(out);
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::publishCenterCoordinates(
    const Eigen::Vector3f& center_odom, const rclcpp::Time& stamp)
{
  auto pt = std::make_unique<geometry_msgs::msg::PointStamped>();
  pt->header.frame_id = odom_frame_;
  pt->header.stamp    = stamp;
  pt->point.x = center_odom.x();
  pt->point.y = center_odom.y();
  pt->point.z = center_odom.z();
  pub_center_point_->publish(std::move(pt));
}

// ─────────────────────────────────────────────────────────────────────────────
// publishSafetyTF — TF DINAMIS: safety_point bisa berpindah saat drone terbang.
// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::publishSafetyTF(
    const Eigen::Vector3f& p_odom, const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = stamp;
  t.header.frame_id = odom_frame_;
  t.child_frame_id  = safety_frame_;
  t.transform.translation.x = p_odom.x();
  t.transform.translation.y = p_odom.y();
  t.transform.translation.z = p_odom.z();
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(t);
}

// ─────────────────────────────────────────────────────────────────────────────
// publishCandidateMarkers — semua kandidat registry sebagai silinder hijau.
// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::publishCandidateMarkers(const rclcpp::Time& stamp)
{
  visualization_msgs::msg::MarkerArray arr;

  // DELETEALL lebih dulu supaya kandidat yang sudah dibuang registry ikut
  // hilang dari RViz, bukan tertinggal sebagai marker yatim.
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = odom_frame_;
  clear.header.stamp    = stamp;
  clear.ns              = "landing_candidates";
  clear.action          = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);

  int id = 0;
  for (const auto& c : registry_->all()) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = odom_frame_;
    m.header.stamp    = stamp;
    m.ns              = "landing_candidates";
    m.id              = id++;
    m.type            = visualization_msgs::msg::Marker::CYLINDER;
    m.action          = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x    = c.center.x();
    m.pose.position.y    = c.center.y();
    m.pose.position.z    = c.center.z();
    m.pose.orientation.w = 1.0;

    m.scale.x = safe_diameter_;
    m.scale.y = safe_diameter_;
    m.scale.z = 0.05;          // cakram tipis, bukan tabung

    // Hijau = boleh dipilih. Oranye = sedang terhalang: entrinya tetap
    // disimpan dan digambar, tapi tidak boleh jadi target. Membedakannya
    // penting untuk keselamatan — menggambar titik yang sudah dicoret dengan
    // warna yang sama persis dengan titik aman akan menyesatkan.
    // Yang TERPILIH tidak dibedakan warnanya; posisinya sudah ditunjukkan
    // oleh TF safety_point.
    if (c.selectable) {
      m.color.r = 0.1f;
      m.color.g = 1.0f;
      m.color.b = 0.1f;
      // Kandidat yang lebih sering terlihat digambar lebih pekat.
      m.color.a = std::min(0.9f, 0.35f + 0.05f * static_cast<float>(c.hits));
    } else {
      m.color.r = 1.0f;
      m.color.g = 0.55f;
      m.color.b = 0.0f;
      m.color.a = 0.55f;
    }

    arr.markers.push_back(m);

    // Skor di atas tiap cakram. Tanpa ini RViz menunjukkan bahwa sebuah titik
    // dipilih, tapi tidak pernah mengapa — dan dua kandidat yang terlihat sama
    // persis bisa berselisih jauh nilainya.
    visualization_msgs::msg::Marker t;
    t.header.frame_id = odom_frame_;
    t.header.stamp    = stamp;
    t.ns              = "landing_candidate_scores";
    t.id              = id;
    t.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    t.action          = visualization_msgs::msg::Marker::ADD;

    t.pose.position.x    = c.center.x();
    t.pose.position.y    = c.center.y();
    t.pose.position.z    = c.center.z() + 0.6;
    t.pose.orientation.w = 1.0;

    t.scale.z = 0.28;
    t.color.r = 1.0f;
    t.color.g = 1.0f;
    t.color.b = 1.0f;
    t.color.a = 0.9f;

    char txt[64];
    if (c.rough_n > 0)
      std::snprintf(txt, sizeof(txt), "%.2f  r=%.3fm",
                    static_cast<double>(registry_->scoreOf(c, score_ref_)),
                    static_cast<double>(c.roughness_m));
    else
      std::snprintf(txt, sizeof(txt), "%.2f  r=?",
                    static_cast<double>(registry_->scoreOf(c, score_ref_)));
    t.text = txt;

    arr.markers.push_back(t);
  }

  pub_markers_->publish(arr);
}

// ─────────────────────────────────────────────────────────────────────────────
// Usia awan saat hasilnya terbit: sekarang − stempel masukan.
//
// "null" bila use_sim_time mati -- stempel awan berasal dari jam simulasi
// Gazebo, jadi menguranginya dengan jam dinding menghasilkan angka besar yang
// konsisten: salah, tapi tidak tampak salah sepintas.
std::string LandingCircle::latencyJson(long long stamp_ns)
{
  if (stamp_ns <= 0) return "null";
  if (!get_parameter("use_sim_time").as_bool()) return "null";
  return std::to_string(
    static_cast<double>(now().nanoseconds() - stamp_ns) / 1e6);
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::publishStats(
    const std::chrono::high_resolution_clock::time_point& t_start,
    const FrameStats& fs)
{
  const double comp_ms = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::high_resolution_clock::now() - t_start).count() / 1000.0;

  const double valid_pct = (fs.total_pts > 0)
    ? (static_cast<double>(fs.valid_pts) / static_cast<double>(fs.total_pts) * 100.0)
    : 0.0;

  const auto& items = registry_->all();

  // Muatannya JSON, bukan teks berformat.
  //
  // Formatnya pernah diurai logger CSV dengan regex, dan regex itu patah
  // senyap dua kali: sekali saat `hits=` dan `blocked_s=` disisipkan, sekali
  // lagi saat roughness dan skor ditambahkan. Karena baris CSV hanya ditulis
  // bila ada koordinat baru, dan koordinatnya tidak pernah terparse, logger
  // berhenti menulis sama sekali -- tanpa satu pun pesan galat. Menambah field
  // ke JSON tidak akan pernah menghasilkan kegagalan seperti itu.
  //
  // Kandidat menjadi larik objek. Roughness dan skor dilaporkan PER KANDIDAT,
  // bukan hanya untuk pemenangnya: tanpa itu tidak ada cara menjawab mengapa
  // sebuah titik menang, dan tidak ada cara mengukur apakah suku roughness
  // benar-benar membedakan kandidat di dunia ini atau hanya menambah derau.
  std::string cands = "[";
  for (size_t i = 0; i < items.size(); ++i) {
    const auto& c = items[i];
    if (i) cands += ", ";
    cands +=
      "{\"x\": "           + std::to_string(c.center.x()) +
      ", \"y\": "          + std::to_string(c.center.y()) +
      ", \"z\": "          + std::to_string(c.center.z()) +
      ", \"hits\": "       + std::to_string(c.hits) +
      ", \"blocked_s\": "  + std::to_string(c.blocked_s) +
      ", \"clearance_m\": " + std::to_string(c.clearance_m) +
      ", \"fill_ratio\": " + std::to_string(c.fill_ratio) +
      ", \"roughness_m\": " + std::to_string(c.roughness_m) +
      ", \"rough_n\": "    + std::to_string(c.rough_n) +
      ", \"score\": "      + std::to_string(registry_->scoreOf(c, score_ref_)) +
      ", \"selectable\": " + (c.selectable ? "true" : "false") +
      "}";
  }
  cands += "]";

  const long long stamp_ns =
      static_cast<long long>(fs.stamp_sec) * 1000000000LL
      + static_cast<long long>(fs.stamp_nanosec);

  std_msgs::msg::String msg;
  msg.data =
    std::string("{\"source\": \"landing_circle\"") +
    ", \"stamp_ns\": "             + std::to_string(stamp_ns) +
    ", \"computation_time_ms\": "  + std::to_string(comp_ms) +
    ", \"latency_ms\": "           + latencyJson(stamp_ns) +
    ", \"input_points\": "         + std::to_string(fs.total_pts) +
    ", \"valid_points\": "         + std::to_string(fs.valid_pts) +
    ", \"valid_percentage\": "     + std::to_string(valid_pct) +
    ", \"safe_size\": "            + std::to_string(fs.safe_size) +
    ", \"registry_size\": "        + std::to_string(items.size()) +
    ", \"num_candidates\": "       + std::to_string(fs.num_candidates) +
    ", \"blocked_candidates\": "   + std::to_string(fs.blocked_count) +
    ", \"committed\": "            + (fs.committed ? "true" : "false") +
    ", \"collect_elapsed_s\": "    + std::to_string(fs.collect_elapsed_s) +
    ", \"frozen_by_motion\": "     + (fs.frozen ? "true" : "false") +
    ", \"search_frozen\": "        + (fs.search_frozen ? "true" : "false") +
    ", \"speed_mps\": "            + std::to_string(fs.speed_mps) +
    ", \"target_blocked_s\": "     + std::to_string(fs.target_blocked_s) +
    ", \"obstacle_points\": "      + std::to_string(fs.obstacle_points) +
    ", \"target_alt_m\": "         + std::to_string(fs.target_alt_m) +
    ", \"has_selection\": "        + (fs.has_selection ? "true" : "false") +
    ", \"selected_distance_m\": "  + std::to_string(fs.selected_distance_m) +
    ", \"selected_score\": "       + std::to_string(fs.selected_score) +
    ", \"selected_x\": "           + std::to_string(selected_center_.x()) +
    ", \"selected_y\": "           + std::to_string(selected_center_.y()) +
    ", \"selected_z\": "           + std::to_string(selected_center_.z()) +
    ", \"grid_cell_m\": "          + std::to_string(grid_cell_) +
    ", \"min_pts_per_cell\": "     + std::to_string(min_pts_cell_) +
    ", \"candidates\": "           + cands +
    "}";
  pub_stats_->publish(msg);

  char gate_txt[48];
  if (fs.committed)
    std::snprintf(gate_txt, sizeof(gate_txt), "COMMITTED%s%s",
                  fs.search_frozen ? "/NO-SEARCH" : "",
                  fs.frozen ? "/FROZEN" : "");
  else
    std::snprintf(gate_txt, sizeof(gate_txt), "collecting %.1fs%s",
                  static_cast<double>(fs.collect_elapsed_s),
                  fs.frozen ? "/FROZEN" : "");

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "Comp: %.2f ms | Valid: %zu / %zu (%.1f%%) | Safe: %zu | "
    "Candidates: %zu (%zu blocked) | %s | Selected: %s (%.2f m) | "
    "score=%.2f obs=%d tblk=%.1fs alt=%.1fm",
    comp_ms, fs.valid_pts, fs.total_pts, valid_pct, fs.safe_size,
    fs.num_candidates, fs.blocked_count, gate_txt,
    fs.has_selection ? "yes" : "no", fs.selected_distance_m,
    static_cast<double>(fs.selected_score),
    fs.obstacle_points, static_cast<double>(fs.target_blocked_s),
    static_cast<double>(fs.target_alt_m));
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingCircle>());
  rclcpp::shutdown();
  return 0;
}
