// file: landing_circle.hpp
//
// Deteksi titik pendaratan aman dari cloud bidang datar.
//
// Node ini menangani geometri per-frame: proyeksi cloud ke grid 2D,
// pencocokan cakram seukuran drone, dan ekstraksi SEMUA kandidat yang muat
// (bukan hanya yang terbaik). State lintas-frame — mengingat kandidat dalam
// frame `map` dan memilih yang terdekat ke drone — dipegang LandingRegistry,
// yang sengaja bebas ROS/PCL agar bisa diuji terpisah.

#pragma once

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Geometry>

// Proyek
#include "segmentation_node/commit_gate.hpp"
#include "segmentation_node/plane_roughness.hpp"
#include "segmentation_node/obstacle_probe.hpp"
#include "segmentation_node/landing_registry.hpp"

// STL
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Hasil penghitungan centroid satu cakram.
struct SafeResult {
  std::array<float, 3> centroid{};
  bool   valid = false;
  size_t size  = 0;

  // Kekasaran permukaan cakram ini. `roughness_valid` false berarti cakramnya
  // terlalu miskin titik untuk diukur — dibedakan tegas dari roughness nol.
  // Lihat plane_roughness.hpp.
  float  roughness_m     = 0.f;
  bool   roughness_valid = false;
};

// Satu kandidat hasil pemindaian grid, masih dalam koordinat (u,v) plane_frame.
struct GridCandidate {
  float cu = 0.f, cv = 0.f;
  float clearance_m = 0.f;
  float fill_ratio  = 0.f;
};

// Angka-angka yang dilaporkan ke /landing_circle_stats untuk satu frame.
struct FrameStats {
  size_t valid_pts           = 0;
  size_t total_pts           = 0;
  size_t safe_size           = 0;
  size_t num_candidates      = 0;
  size_t blocked_count       = 0;
  bool   has_selection       = false;
  float  selected_distance_m = 0.f;

  // Gerbang commit: apakah keputusan sudah boleh diumumkan, dan sudah berapa
  // detik pengamatan terkumpul kalau belum.
  bool   committed           = false;
  float  collect_elapsed_s   = 0.f;

  // Jam registry dibekukan frame ini karena drone bertranslasi.
  bool   frozen              = false;
  float  speed_mps           = 0.f;

  // Berapa titik outlier terhitung berdiri di atas cakram terkunci, dan
  // ketinggian drone di atas cakram itu saat pengukuran diambil. Keduanya
  // dicatat berpasangan supaya sebaran "titik palsu vs ketinggian" bisa
  // dibaca dari satu penerbangan, bukan ditebak dari satu kejadian.
  int    obstacle_points     = 0;
  float  target_alt_m        = 0.f;

  // Sudah berapa detik cakram yang dikunci terlihat terhalang.
  float  target_blocked_s    = 0.f;

  // Frame ini melewati seluruh pencarian dan hanya menjaga satu titik.
  bool   search_frozen       = false;

  // Skor kandidat terpilih, supaya log bisa menjawab MENGAPA titik itu menang
  // dan bukan sekadar bahwa ia menang.
  float  selected_score      = 0.f;

  // Stempel awan MASUKAN, dipecah seperti di header ROS. Diteruskan ke stats
  // supaya baris dari topik berbeda bisa digabungkan pada frame yang sama:
  // waktu terima berbeda di tiap node dan bergeser oleh beban CPU, sedangkan
  // stempel ini diwarisi dari pesan kamera yang sama.
  int32_t  stamp_sec         = 0;
  uint32_t stamp_nanosec     = 0;
};

class LandingCircle : public rclcpp::Node
{
public:
  LandingCircle();

private:
  // ── Callback utama ────────────────────────────────────────────────────────
  void cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // ── Ekstraksi multi-kandidat ──────────────────────────────────────────────
  // Kumpulkan SEMUA pusat cakram yang lolos uji, lalu saring dengan greedy
  // non-maximum suppression supaya kandidat yang berdekatan tidak dilaporkan
  // berkali-kali. Diurutkan menurun berdasarkan clearance.
  std::vector<GridCandidate> extractCandidates(
      const std::vector<uint8_t>& safe_center,
      const std::vector<uint8_t>& obstacle,
      const std::vector<int>&     dist,
      const std::vector<int>&     counts,
      int rows, int cols,
      float cell, float r_safe,
      float umin, float vmin) const;

  // ── Helpers geometri ──────────────────────────────────────────────────────
  // Centroid 3D (dalam plane_frame) dari titik-titik di dalam satu cakram.
  // Dipisah dari publishSafeCloud karena dipanggil untuk SETIAP kandidat,
  // sedangkan yang diterbitkan hanya cakram yang terpilih.
  SafeResult computeDiskCentroid(
      const pcl::PointCloud<pcl::PointXYZ>&       cloud,
      const std::vector<std::pair<float, float>>& uv,
      const std::vector<Eigen::Vector3d>&         P_plane,
      float cu, float cv, float r_safe) const;

  // ── Helpers publish ───────────────────────────────────────────────────────
  void publishSafeCloud(
      const pcl::PointCloud<pcl::PointXYZ>&       cloud,
      const std::vector<std::pair<float, float>>& uv,
      bool  have_center,
      float cu, float cv, float r_safe,
      const std_msgs::msg::Header& hdr);

  void publishCenterCoordinates(
      const Eigen::Vector3f& center_odom,
      const rclcpp::Time&    stamp);

  void publishStats(
      const std::chrono::high_resolution_clock::time_point& t_start,
      const FrameStats& fs);

  // Usia awan saat hasilnya terbit, dalam milidetik, sebagai literal JSON.
  // "null" bila stempelnya belum ada atau use_sim_time mati.
  std::string latencyJson(long long stamp_ns);

  // Lepaskan target yang sudah dikunci dan BUANG seluruh registry, sehingga
  // frame berikutnya kembali mengumpulkan dari nol. Lihat landing_registry.hpp
  // untuk alasan kandidat lama tidak boleh diwarisi.
  void releaseTarget(const char* reason);

  // TF dinamis: safety_point bisa berpindah saat drone terbang, jadi ia
  // diterbitkan ke /tf tiap frame — bukan ke /tf_static.
  void publishSafetyTF(const Eigen::Vector3f& p_odom, const rclcpp::Time& stamp);

  // Semua kandidat di registry sebagai silinder hijau, termasuk yang sudah
  // keluar dari jangkauan pandang kamera.
  void publishCandidateMarkers(const rclcpp::Time& stamp);

  // ── Proyeksi UV (implementasi tergantung plane_axes_) ─────────────────────
  inline void pickUV(const Eigen::Vector3d& p, float& u, float& v) const
  {
    if      (plane_axes_ == "xy") { u = p.x(); v = p.y(); }
    else if (plane_axes_ == "xz") { u = p.x(); v = p.z(); }
    else                           { u = p.y(); v = p.z(); } // default "yz"
  }

  // ── Parameter ROS ─────────────────────────────────────────────────────────
  std::string input_topic_, output_topic_;
  std::string plane_frame_, odom_frame_, safety_frame_, base_frame_;

  // Frame yang disiarkan konsumen (node misi) di titik yang baru saja
  // gagal didarati, supaya seleksi berhenti menempel di sana.
  std::string reject_frame_;
  double      reject_timeout_s_;

  // Frame yang dipantau untuk mengetahui apakah drone sedang bertranslasi,
  // dan ambang lajunya. Saat bergerak, jam registry dibekukan.
  std::string motion_frame_;
  double      still_speed_mps_;

  // Berapa lama cakram yang sudah dikunci harus terhalang sebelum targetnya
  // dilepas, dan akumulatornya. Jam ini TIDAK ikut beku saat drone bergerak —
  // itulah gunanya.
  double      target_block_s_;
  float       target_blocked_s_ = 0.f;

  // Penghalang dicari sebagai KEHADIRAN titik bukan-bidang di atas cakram,
  // bukan sebagai ketiadaan titik bidang di dalamnya. Lihat obstacle_probe.hpp.
  double      obstacle_min_height_m_;
  double      obstacle_max_height_m_;
  int         obstacle_min_points_;
  double      outlier_max_age_s_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_outlier_;
  sensor_msgs::msg::PointCloud2::SharedPtr                       last_outlier_;

  // Salinan RegistryConfig::block_after_s, supaya penjaga cakram bisa mendorong
  // target melewati ambang yang sama tanpa jalur "terhalang" kedua.
  float       block_after_s_ = 3.f;

  // Salinan RegistryConfig::reject_radius_m, untuk mengenali apakah penolakan
  // konsumen mengenai target yang sedang dipakai.
  float       reject_radius_m_ = 1.5f;

  Eigen::Vector3f prev_motion_pos_{Eigen::Vector3f::Zero()};
  rclcpp::Time    prev_motion_stamp_;
  bool            have_prev_motion_ = false;
  std::string plane_axes_;

  double safe_diameter_;
  double grid_cell_;
  int    min_pts_cell_;
  double fill_ratio_req_;
  bool   close_gaps_;
  int    inflate_cells_;

  // Multi-kandidat
  double candidate_min_sep_m_;
  int    max_candidates_per_frame_;

  // Seleksi.
  // Sticky hanya berlaku SETELAH gerbang commit terbuka; nilai ini adalah
  // niat, bukan keadaan. Lihat langkah 15 di landing_circle.cpp.
  bool   sticky_target_;

  // Titik minimum agar residual bidang berarti. Lihat plane_roughness.hpp.
  int    roughness_min_points_;

  // Hentikan pencarian begitu gerbang commit terbuka dan ada target.
  //
  // Sesudah commit, seleksi sticky tidak akan berpindah ke kandidat mana pun
  // yang ditemukan kemudian — jadi mencarinya hanya membakar CPU pada mesin
  // yang waktu komputasinya sedang diukur. Terukur satu penerbangan: kandidat
  // naik dari 15 ke 44 selama turun, tidak satu pun terpakai.
  //
  // false mengembalikan perilaku lama, untuk membandingkan saat debugging.
  bool   freeze_after_commit_;

  // Bagian cakram yang harus tersentuh data frame ini sebelum sebuah
  // kandidat lama boleh kena strike. Lihat grid_coverage.hpp.
  double observe_coverage_min_;

  // ── TF ────────────────────────────────────────────────────────────────────
  std::unique_ptr<tf2_ros::Buffer>              tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ── Publishers & subscriber ───────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_center_point_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_stats_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // ── State lintas-frame ────────────────────────────────────────────────────
  std::unique_ptr<LandingRegistry> registry_;

  // Menahan penerbitan TF `safety_point` sampai ada N detik pengamatan
  // sungguhan. Lihat commit_gate.hpp.
  std::unique_ptr<CommitGate>      commit_gate_;
  Eigen::Vector3f                  selected_center_{Eigen::Vector3f::Zero()};
  bool                             has_selection_ = false;

  // Acuan seleksi frame terakhir (posisi base_frame dalam `map`). Disimpan
  // supaya publishStats bisa melaporkan skor tiap kandidat tanpa mengulang
  // lookup TF — dan supaya angka yang dilaporkan berasal dari acuan yang sama
  // dengan yang dipakai seleksinya.
  Eigen::Vector3f                  score_ref_{Eigen::Vector3f::Zero()};

  // Stempel waktu frame sebelumnya, untuk menurunkan dt yang dipakai registry.
  rclcpp::Time                     last_stamp_;
  bool                             have_last_stamp_ = false;

  // ── Cache disk offsets — rebuilt hanya saat R berubah ────────────────────
  std::vector<std::pair<int, int>> disk_offsets_safe_;
  int                              cached_R_safe_cells_ = -1;
};
