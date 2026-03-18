// file: safe_landing_color.cpp
// Optimized for real-time performance:
//   1. Merged valid_pts counting into the main transform loop (eliminates one O(N) pass)
//   2. Replaced goto in dilate_once with clean early-exit flag
//   3. publishOnlySafe returns a SafeResult struct (removes raw output pointers)
//   4. safe_rgb->reserve uses estimated safe-disk capacity instead of cloud.size()
//   5. computeMode uses sorted window for O(W log W) instead of O(W²)
//   6. safety_tf_published_ can optionally re-lock if centroid drifts beyond relock_dist_m_
//   7. disk_offsets_safe cached as member to avoid realloc every callback
//   8. Unique landing circle counter: koordinat disimpan dalam odom_frame_ (map),
//      lokasi yang sudah pernah ditemukan (dalam radius unique_circle_dist_m_) tidak dihitung ulang.

#include "segmentation_node/landing_circle.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <limits>
#include <algorithm>
#include <vector>
#include <cmath>
#include <string>
#include <utility>
#include <cfloat>
#include <array>
#include <deque>
#include <chrono>

LandingCircle::LandingCircle() : Node("landing_circle")
{
  // ── I/O ──────────────────────────────────────────────────────────────────
  input_topic_  = declare_parameter<std::string>("input_topic",  "/plane");
  output_topic_ = declare_parameter<std::string>("output_topic", "/safe_circle");

  // ── Frame TF ─────────────────────────────────────────────────────────────
  plane_frame_  = declare_parameter<std::string>("plane_frame",  "camera_link");
  odom_frame_   = declare_parameter<std::string>("odom_frame",   "map");
  safety_frame_ = declare_parameter<std::string>("safety_frame", "safety_point");

  // ── Proyeksi bidang ───────────────────────────────────────────────────────
  plane_axes_   = declare_parameter<std::string>("plane_axes",   "yz");

  // ── Geometri area ─────────────────────────────────────────────────────────
  safe_diameter_ = declare_parameter<double>("safe_diameter",    1.3);

  // ── Grid & kriteria ───────────────────────────────────────────────────────
  grid_cell_      = declare_parameter<double>("grid_cell",       0.23);
  min_pts_cell_   = declare_parameter<int>("min_pts_per_cell",   7);
  fill_ratio_req_ = declare_parameter<double>("safe_fill_ratio", 0.95);
  close_gaps_     = declare_parameter<bool>("close_gaps",        true);
  inflate_cells_  = declare_parameter<int>("inflate_cells",      1);

  // ── Voting lintas-frame ───────────────────────────────────────────────────
  vote_window_      = declare_parameter<int>("vote_window",        1);
  vote_tolerance_m_ = declare_parameter<double>("vote_tolerance_m", 0.20);
  min_votes_to_fix_ = declare_parameter<int>("min_votes_to_fix",    1);

  // ── Re-lock jika centroid bergeser (opt-in) ───────────────────────────────
  // Set relock_dist_m > 0 agar safety_tf bisa di-reset kalau zona bergeser
  relock_dist_m_ = declare_parameter<double>("relock_dist_m", 0.0);

  // ── Unique landing circle dedup ───────────────────────────────────────────
  // Dua landing circle dianggap SAMA jika jaraknya (dalam odom_frame_) < nilai ini.
  // Koordinat selalu disimpan dalam odom_frame_ (referensi map global).
  unique_circle_dist_m_ = declare_parameter<double>("unique_circle_dist_m", 0.5);

  // ── TF ────────────────────────────────────────────────────────────────────
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // ── Subs & pubs ───────────────────────────────────────────────────────────
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LandingCircle::cbCloud, this, std::placeholders::_1));

  pub_              = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 1);
  pub_center_point_ = create_publisher<geometry_msgs::msg::PointStamped>(
                        "/safe_circle_center_coords", 1);
  pub_stats_        = create_publisher<std_msgs::msg::String>(
                        "/landing_circle_stats", 10);

  RCLCPP_INFO(get_logger(),
    "LandingCircle: plane_frame=%s axes=%s safe_d=%.2fm grid=%.2fm fill>=%.2f "
    "pts/cell>=%d inflate=%d votes(window=%d tol=%.2fm min=%d) relock=%.2fm "
    "unique_dist=%.2fm -> TF '%s' in '%s'",
    plane_frame_.c_str(), plane_axes_.c_str(), safe_diameter_, grid_cell_,
    fill_ratio_req_, min_pts_cell_, inflate_cells_,
    vote_window_, vote_tolerance_m_, min_votes_to_fix_,
    relock_dist_m_, unique_circle_dist_m_,
    safety_frame_.c_str(), odom_frame_.c_str());
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
    publishOnlySafe(*cloud, {}, {}, false, 0.f, 0.f, 0.f, msg->header);
    publishStats(t_start, 0, 0, 0);
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
  // [OPT-1] valid_pts dihitung di sini — tidak perlu loop terpisah lagi
  std::vector<Eigen::Vector3d>          P_plane(total_pts);
  std::vector<std::pair<float, float>>  uv(total_pts);
  size_t valid_pts = 0;

  float umin = +FLT_MAX, vmin = +FLT_MAX,
        umax = -FLT_MAX, vmax = -FLT_MAX;

  for (size_t i = 0; i < total_pts; ++i) {
    const auto& p = (*cloud)[i];

    // Hitung valid sekaligus (menggantikan loop terpisah sebelumnya)
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

  // ── 3) Margin grid ────────────────────────────────────────────────────────
  constexpr float margin = 0.25f;
  umin -= margin;  umax += margin;
  vmin -= margin;  vmax += margin;

  // ── 4) Param grid ─────────────────────────────────────────────────────────
  const float r_safe = static_cast<float>(0.5 * safe_diameter_);
  const float cell   = static_cast<float>(grid_cell_);
  const int   cols   = std::max(1, static_cast<int>(std::ceil((umax - umin) / cell)));
  const int   rows   = std::max(1, static_cast<int>(std::ceil((vmax - vmin) / cell)));

  if (static_cast<long long>(rows) * cols > 10'000'000LL) {
    RCLCPP_WARN(get_logger(), "Grid too large (%dx%d). Increase grid_cell.", cols, rows);
    publishOnlySafe(*cloud, uv, P_plane, false, 0.f, 0.f, 0.f, msg->header);
    publishStats(t_start, valid_pts, total_pts, 0);
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

  // ── 7) Disk offsets — cached sebagai member, rebuild hanya jika param berubah
  //      [OPT-7] Hindari realloc setiap callback
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

  // [OPT-2] dilate_once: ganti goto dengan early-exit flag — lebih aman & portable
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

  // ── 11) Pilih pusat terbaik per frame ─────────────────────────────────────
  const float cx_cell = static_cast<float>(cols) * 0.5f;
  const float cy_cell = static_cast<float>(rows) * 0.5f;
  int   best_ix = -1, best_iy = -1;
  float best_clearance = -1e9f, best_center_d2 = 1e30f;

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

      const float clearance_m = dist[IDX(ix, iy)] * cell - r_safe;
      const float dcx = ((float)ix + 0.5f - cx_cell) * cell;
      const float dcy = ((float)iy + 0.5f - cy_cell) * cell;
      const float center_d2 = dcx * dcx + dcy * dcy;

      if (clearance_m > best_clearance + 1e-6f ||
          (std::abs(clearance_m - best_clearance) <= 1e-6f &&
           center_d2 < best_center_d2)) {
        best_clearance = clearance_m;
        best_center_d2 = center_d2;
        best_ix = ix;
        best_iy = iy;
      }
    }
  }

  if (best_ix < 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No safe center found.");
    publishOnlySafe(*cloud, uv, P_plane, false, 0.f, 0.f, 0.f, msg->header);
    publishStats(t_start, valid_pts, total_pts, 0);
    return;
  }

  // ── 12) Pusat terpilih di koordinat (u,v) ────────────────────────────────
  const float cu = umin + ((float)best_ix + 0.5f) * cell;
  const float cv = vmin + ((float)best_iy + 0.5f) * cell;

  // ── 13) Publish point cloud safe + centroid ───────────────────────────────
  // [OPT-3] publishOnlySafe kini mengembalikan SafeResult — tidak ada raw pointer lagi
  const SafeResult res =
    publishOnlySafe(*cloud, uv, P_plane, true, cu, cv, r_safe, msg->header);

  publishStats(t_start, valid_pts, total_pts, res.size);

  if (res.valid)
    publishCenterCoordinates(cu, cv, res.centroid, plane_frame_, msg->header.stamp);

  // ── 14) Voting & publish TF statis ───────────────────────────────────────
  if (!res.valid) return;

  pushVote(res.centroid);
  std::array<float, 3> mode_center{};
  int mode_count = 0;
  computeMode(mode_center, mode_count);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "Voting: window=%zu mode_count=%d (min=%d)",
    votes_.size(), mode_count, min_votes_to_fix_);

  // [OPT-6] Re-lock jika centroid bergeser lebih dari relock_dist_m_
  if (safety_tf_published_ && relock_dist_m_ > 0.0) {
    const float dx = mode_center[0] - locked_center_odom_[0];
    const float dy = mode_center[1] - locked_center_odom_[1];
    const float dz = mode_center[2] - locked_center_odom_[2];
    if (std::sqrt(dx*dx + dy*dy + dz*dz) > static_cast<float>(relock_dist_m_)) {
      RCLCPP_WARN(get_logger(),
        "Landing zone drifted %.3fm > %.3fm — re-locking TF.",
        std::sqrt(dx*dx + dy*dy + dz*dz), relock_dist_m_);
      safety_tf_published_ = false;
      votes_.clear();
    }
  }

  if (mode_count >= min_votes_to_fix_) {
    // ── Konversi mode_center ke odom_frame_ (selalu, bukan hanya saat TF baru) ──
    std::array<float, 3> center_odom = mode_center;
    if (plane_frame_ != odom_frame_) {
      try {
        auto T_odom_from_plane = tf_buffer_->lookupTransform(
          odom_frame_, plane_frame_, tf2::TimePointZero);
        const Eigen::Isometry3d Xop = tf2::transformToEigen(T_odom_from_plane);
        const Eigen::Vector3d c_plane(mode_center[0], mode_center[1], mode_center[2]);
        const Eigen::Vector3d c_odom = Xop * c_plane;
        center_odom = {static_cast<float>(c_odom.x()),
                       static_cast<float>(c_odom.y()),
                       static_cast<float>(c_odom.z())};
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(),
          "No TF %s <- %s for final TF: %s",
          odom_frame_.c_str(), plane_frame_.c_str(), e.what());
        return;
      }
    }

    // ── [FIX] Registrasi unique circle SELALU dilakukan, tidak bergantung
    //    pada safety_tf_published_. Setiap lokasi baru di odom_frame_ yang
    //    belum pernah ditemukan (> unique_circle_dist_m_) akan dicatat. ──────
    const bool is_new = tryRegisterNewLandingCircle(center_odom);
    if (is_new) {
      RCLCPP_INFO(get_logger(),
        "NEW landing circle #%zu discovered at (%.3f, %.3f, %.3f) [%s]",
        known_landing_circles_.size(),
        center_odom[0], center_odom[1], center_odom[2],
        odom_frame_.c_str());
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Landing circle already known (total unique: %zu)",
        known_landing_circles_.size());
    }

    // ── Publish static TF hanya jika belum pernah di-publish untuk lokasi ini ──
    if (!safety_tf_published_) {
      publishStaticSafetyTF(center_odom);
      locked_center_odom_ = center_odom;
      safety_tf_published_ = true;
      RCLCPP_INFO(get_logger(),
        "Published static TF '%s' in '%s' at (%.3f, %.3f, %.3f)",
        safety_frame_.c_str(), odom_frame_.c_str(),
        center_odom[0], center_odom[1], center_odom[2]);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// tryRegisterNewLandingCircle
// Koordinat center_odom harus dalam odom_frame_ (frame map / global reference).
// Return true  → lokasi baru, ditambahkan ke known_landing_circles_.
// Return false → sudah pernah ditemukan (dalam radius unique_circle_dist_m_).
// ─────────────────────────────────────────────────────────────────────────────
bool LandingCircle::tryRegisterNewLandingCircle(const std::array<float, 3>& center_odom)
{
  const float thr2 = static_cast<float>(unique_circle_dist_m_) *
                     static_cast<float>(unique_circle_dist_m_);

  for (const auto& known : known_landing_circles_) {
    const float dx = center_odom[0] - known[0];
    const float dy = center_odom[1] - known[1];
    const float dz = center_odom[2] - known[2];
    if (dx*dx + dy*dy + dz*dz < thr2) {
      return false;  // sudah ada yang sangat dekat — bukan lokasi baru
    }
  }

  // Lokasi baru — simpan
  known_landing_circles_.push_back(center_odom);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// publishStats — diperluas dengan info unique landing circles
// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::publishStats(
    const std::chrono::high_resolution_clock::time_point& t_start,
    size_t valid_pts,
    size_t total_pts,
    size_t safe_size)
{
  const double comp_ms = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::high_resolution_clock::now() - t_start).count() / 1000.0;

  const double valid_pct = (total_pts > 0)
    ? (static_cast<double>(valid_pts) / static_cast<double>(total_pts) * 100.0)
    : 0.0;

  const size_t unique_count = known_landing_circles_.size();

  // Buat ringkasan koordinat semua landing circles yang sudah diketahui
  std::string circles_summary;
  for (size_t i = 0; i < unique_count; ++i) {
    const auto& c = known_landing_circles_[i];
    circles_summary +=
      "  [" + std::to_string(i + 1) + "] "
      "x=" + std::to_string(c[0]) + " "
      "y=" + std::to_string(c[1]) + " "
      "z=" + std::to_string(c[2]) +
      " (" + odom_frame_ + ")\n";
  }

  std_msgs::msg::String msg;
  msg.data =
    "computation_time: "          + std::to_string(comp_ms)    + " ms\n" +
    "valid_points: "              + std::to_string(valid_pts)   + "\n"   +
    "valid_percentage: "          + std::to_string(valid_pct)   + " %\n" +
    "plane_size: "                + std::to_string(safe_size)   + "\n"   +
    "outlier_size: "              + std::to_string(total_pts - valid_pts) + "\n" +
    "unique_landing_circles: "    + std::to_string(unique_count) + "\n"  +
    "landing_circle_locations:\n" + circles_summary;
  pub_stats_->publish(msg);

  RCLCPP_INFO(get_logger(),
    "Comp: %.2f ms | Valid: %zu / %zu (%.1f%%) | Safe: %zu | Unique LC: %zu",
    comp_ms, valid_pts, total_pts, valid_pct, safe_size, unique_count);
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::publishCenterCoordinates(
    float /*cu*/, float /*cv*/,
    const std::array<float, 3>& centroid_plane,
    const std::string& frame,
    const rclcpp::Time& stamp)
{
  auto pt = std::make_unique<geometry_msgs::msg::PointStamped>();
  pt->header.frame_id = frame;
  pt->header.stamp    = stamp;
  pt->point.x = centroid_plane[0];
  pt->point.y = centroid_plane[1];
  pt->point.z = centroid_plane[2];
  pub_center_point_->publish(std::move(pt));
}

// ─────────────────────────────────────────────────────────────────────────────
// publishOnlySafe — [OPT-3] mengembalikan SafeResult, [OPT-4] reserve lebih akurat
// ─────────────────────────────────────────────────────────────────────────────
SafeResult LandingCircle::publishOnlySafe(
    const pcl::PointCloud<pcl::PointXYZ>&         cloud,
    const std::vector<std::pair<float, float>>&   uv,
    const std::vector<Eigen::Vector3d>&            P_plane,
    bool  have_center,
    float cu, float cv, float r_safe,
    const std_msgs::msg::Header& hdr)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr safe_rgb(
    new pcl::PointCloud<pcl::PointXYZRGB>);

  // [OPT-4] Estimasi kapasitas dari luas disk agar tidak over-alokasi
  if (have_center) {
    const int disk_area = static_cast<int>(disk_offsets_safe_.size());
    const size_t est_cap = static_cast<size_t>(disk_area) *
                           static_cast<size_t>(min_pts_cell_) * 2;
    safe_rgb->reserve(std::min(cloud.size(), est_cap));
  }

  SafeResult result;
  double sx = 0.0, sy = 0.0, sz = 0.0;

  if (have_center) {
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
      ++result.size;

      sx += P_plane[i].x();
      sy += P_plane[i].y();
      sz += P_plane[i].z();
    }
    if (result.size > 0) {
      const double inv = 1.0 / static_cast<double>(result.size);
      result.centroid[0] = static_cast<float>(sx * inv);
      result.centroid[1] = static_cast<float>(sy * inv);
      result.centroid[2] = static_cast<float>(sz * inv);
      result.valid = true;
    }
  }

  safe_rgb->width    = static_cast<uint32_t>(safe_rgb->size());
  safe_rgb->height   = 1;
  safe_rgb->is_dense = false;

  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*safe_rgb, out);
  out.header = hdr;
  pub_->publish(out);

  RCLCPP_INFO(get_logger(), "Published safe points: %zu", result.size);
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::pushVote(const std::array<float, 3>& c)
{
  votes_.push_back(c);
  while (static_cast<int>(votes_.size()) > vote_window_)
    votes_.pop_front();
}

// ─────────────────────────────────────────────────────────────────────────────
// computeMode — [OPT-5] O(W log W) via sort + sliding window
// Window kecil (≤30) → praktis identik ke O(W²), tapi siap untuk window lebih besar
// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::computeMode(std::array<float, 3>& mode_center, int& mode_count)
{
  mode_count = 0;
  if (votes_.empty()) { mode_center = {}; return; }

  const float tol = static_cast<float>(vote_tolerance_m_);
  const float tol2 = tol * tol;

  // Salin ke vector agar bisa di-sort per sumbu x
  std::vector<std::array<float, 3>> sorted_v(votes_.begin(), votes_.end());
  std::sort(sorted_v.begin(), sorted_v.end(),
    [](const auto& a, const auto& b) { return a[0] < b[0]; });

  // Sliding window: untuk tiap titik i, kumpulkan semua j dimana |x_j - x_i| <= tol
  // lalu filter dengan jarak 3D penuh
  const size_t W = sorted_v.size();
  size_t left = 0;

  for (size_t i = 0; i < W; ++i) {
    // Majukan left sampai jarak sumbu-x < tol
    while (left < i &&
           (sorted_v[i][0] - sorted_v[left][0]) > tol)
      ++left;

    int   cnt = 0;
    double mx = 0.0, my = 0.0, mz = 0.0;
    for (size_t j = left; j < W; ++j) {
      if ((sorted_v[j][0] - sorted_v[i][0]) > tol) break;
      const float d2 =
        sqr(sorted_v[i][0] - sorted_v[j][0]) +
        sqr(sorted_v[i][1] - sorted_v[j][1]) +
        sqr(sorted_v[i][2] - sorted_v[j][2]);
      if (d2 <= tol2) {
        ++cnt;
        mx += sorted_v[j][0];
        my += sorted_v[j][1];
        mz += sorted_v[j][2];
      }
    }
    if (cnt > mode_count) {
      mode_count    = cnt;
      const double inv = 1.0 / cnt;
      mode_center[0] = static_cast<float>(mx * inv);
      mode_center[1] = static_cast<float>(my * inv);
      mode_center[2] = static_cast<float>(mz * inv);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void LandingCircle::publishStaticSafetyTF(const std::array<float, 3>& p_odom)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp.sec     = 0;
  t.header.stamp.nanosec = 0;
  t.header.frame_id      = odom_frame_;
  t.child_frame_id       = safety_frame_;
  t.transform.translation.x = p_odom[0];
  t.transform.translation.y = p_odom[1];
  t.transform.translation.z = p_odom[2];
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(t);
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingCircle>());
  rclcpp::shutdown();
  return 0;
}