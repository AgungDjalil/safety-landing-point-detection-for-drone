// file: safe_landing_color.cpp

#include "segmentation_node/landing_circle.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

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

LandingCircle::LandingCircle() : Node("landing_circle")
{
  // I/O
  input_topic_    = declare_parameter<std::string>("input_topic",   "/plane");
  output_topic_   = declare_parameter<std::string>("output_topic",  "/safe_circle");

  // Frame TF
  plane_frame_    = declare_parameter<std::string>("plane_frame",   "camera_link"); // tempat proyeksi & perhitungan
  odom_frame_     = declare_parameter<std::string>("odom_frame",    "odom"); // tempat publish TF safety
  safety_frame_   = declare_parameter<std::string>("safety_frame",  "safety_point");

  // Proyeksi bidang
  plane_axes_     = declare_parameter<std::string>("plane_axes",    "yz"); // "xy","yz","xz"

  // Geometri area (HANYA diameter aman)
  safe_diameter_  = declare_parameter<double>("safe_diameter",      1.3);  // meter (disk hijau)

  // Grid & kriteria
  grid_cell_      = declare_parameter<double>("grid_cell",          0.23); // meter/sel
  min_pts_cell_   = declare_parameter<int>("min_pts_per_cell",      10);
  fill_ratio_req_ = declare_parameter<double>("safe_fill_ratio",    0.95);
  close_gaps_     = declare_parameter<bool>("close_gaps",           true);
  inflate_cells_  = declare_parameter<int>("inflate_cells",         1);

  // Voting lintas-frame
  vote_window_        = declare_parameter<int>("vote_window",        15);
  vote_tolerance_m_   = declare_parameter<double>("vote_tolerance_m",0.20);
  min_votes_to_fix_   = declare_parameter<int>("min_votes_to_fix",   8);

  // TF
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Subs & pubs
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LandingCircle::cbCloud, this, std::placeholders::_1));

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 1);
  pub_center_point_ = create_publisher<geometry_msgs::msg::PointStamped>(
  "/safe_circle_center_coords", 1);

  RCLCPP_INFO(get_logger(),
    "LandingCircle: plane_frame=%s axes=%s safe_d=%.2fm grid=%.2fm fill>=%.2f pts/sel>=%d inflate=%d votes(window=%d tol=%.2fm min=%d) -> TF '%s' in '%s'",
    plane_frame_.c_str(), plane_axes_.c_str(), safe_diameter_, grid_cell_, fill_ratio_req_,
    min_pts_cell_, inflate_cells_, vote_window_, vote_tolerance_m_, min_votes_to_fix_,
    safety_frame_.c_str(), odom_frame_.c_str());
}

void LandingCircle::cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // 0) konversi cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input cloud empty");
    publishOnlySafe(*cloud, {}, {}, false, 0.f, 0.f, 0.f, msg->header, nullptr);
    return;
  }

  // 1) ambil TF plane <- cloud
  const std::string cloud_frame = msg->header.frame_id;
  geometry_msgs::msg::TransformStamped T_plane_from_cloud;
  try {
    T_plane_from_cloud = tf_buffer_->lookupTransform(
        plane_frame_, cloud_frame, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
    "No TF %s <- %s: %s", plane_frame_.c_str(), cloud_frame.c_str(), e.what());
    return;
  }

  Eigen::Isometry3d Xpc = tf2::transformToEigen(T_plane_from_cloud);
  
  // 2) transform semua titik → plane_frame, kumpulkan u,v & bbox
  std::vector<Eigen::Vector3d> P_plane; P_plane.resize(cloud->size());
  std::vector<std::pair<float,float>> uv; uv.resize(cloud->size());
  
  float umin=+FLT_MAX, vmin=+FLT_MAX, umax=-FLT_MAX, vmax=-FLT_MAX;
  
  for (size_t i=0;i<cloud->size();++i) {
    const auto& p = (*cloud)[i];
    Eigen::Vector3d pc(p.x, p.y, p.z);
    Eigen::Vector3d pp = Xpc * pc; // di plane_frame
    P_plane[i] = pp;

    float u,v; pickUV(pp, u, v);
    uv[i] = {u,v};
    umin = std::min(umin,u); umax = std::max(umax,u);
    vmin = std::min(vmin,v); vmax = std::max(vmax,v);
  }
  // 3) margin grid
  const float margin = 0.25f;
  umin -= margin; umax += margin;
  vmin -= margin; vmax += margin;
  
  // 4) param grid
  const float r_safe  = static_cast<float>(0.5 * safe_diameter_);
  const float cell    = static_cast<float>(grid_cell_);
  const int   cols    = std::max(1, static_cast<int>(std::ceil((umax - umin)/cell)));
  const int   rows    = std::max(1, static_cast<int>(std::ceil((vmax - vmin)/cell)));
  if ((long long)rows * (long long)cols > 10'000'000LL) {
    RCLCPP_WARN(get_logger(), "Grid too large (%dx%d). Increase grid_cell.", cols, rows);
    publishOnlySafe(*cloud, uv, P_plane, false, 0.f, 0.f, 0.f, msg->header, nullptr);
    return;
  }
  auto IDX = [cols](int ix, int iy){ return iy * cols + ix; };
  auto in_bounds = [cols,rows](int x,int y){ return (x>=0 && x<cols && y>=0 && y<rows); };

  // 5) histogram per cell
  std::vector<int> counts_raw(rows*cols, 0);
  for (const auto& q : uv) {
    int ix = static_cast<int>(std::floor((q.first  - umin)/cell));
    int iy = static_cast<int>(std::floor((q.second - vmin)/cell));
    if (in_bounds(ix,iy)) counts_raw[IDX(ix,iy)]++;
  }

  // 6) close gaps (opsional)
  std::vector<int> counts = counts_raw;
  if (close_gaps_) {
    std::vector<int> counts2 = counts;
    for (int y=1;y<rows-1;++y){
      for (int x=1;x<cols-1;++x){
        int sum=0;
        for(int dy=-1;dy<=1;++dy)
          for(int dx=-1;dx<=1;++dx)
            sum += (counts[IDX(x+dx,y+dy)]>0);
        if (sum>=5 && counts[IDX(x,y)]==0) counts2[IDX(x,y)]=1;
      }
    }
    counts.swap(counts2);
  }

  // 7) disk offsets (unit cell)
  const int R_safe_cells = std::max(1, static_cast<int>(std::round(r_safe / cell)));
  std::vector<std::pair<int,int>> disk_offsets_safe;
  disk_offsets_safe.reserve((2*R_safe_cells+1)*(2*R_safe_cells+1));
  for (int dy=-R_safe_cells; dy<=R_safe_cells; ++dy)
    for (int dx=-R_safe_cells; dx<=R_safe_cells; ++dx)
      if (dx*dx + dy*dy <= R_safe_cells*R_safe_cells)
        disk_offsets_safe.emplace_back(dx,dy);
  const int disk_area_safe = static_cast<int>(disk_offsets_safe.size());

  // 8) kandidat pusat berdasarkan fill ratio
  std::vector<uint8_t> safe_center(rows*cols, 0);
  for (int iy=0; iy<rows; ++iy) {
    for (int ix=0; ix<cols; ++ix) {
      if (ix < R_safe_cells || ix >= cols - R_safe_cells ||
          iy < R_safe_cells || iy >= rows - R_safe_cells) continue;

      int filled_cells=0;
      for (const auto& o : disk_offsets_safe) {
        const int jx = ix + o.first, jy = iy + o.second;
        if (!in_bounds(jx,jy)) continue;
        if (counts[IDX(jx,jy)] >= min_pts_cell_) filled_cells++;
      }
      const float ratio = static_cast<float>(filled_cells) / static_cast<float>(disk_area_safe);
      if (ratio >= static_cast<float>(fill_ratio_req_)) safe_center[IDX(ix,iy)] = 1;
    }
  }

  // 9) peta obstacle + inflasi
  std::vector<uint8_t> obstacle(rows*cols, 0);
  for (int iy=0; iy<rows; ++iy)
    for (int ix=0; ix<cols; ++ix)
      obstacle[IDX(ix,iy)] = (counts_raw[IDX(ix,iy)] < min_pts_cell_) ? 1 : 0;

  auto dilate_once = [&](std::vector<uint8_t>& src){
    std::vector<uint8_t> dst = src;
    for (int y=0; y<rows; ++y) {
      for (int x=0; x<cols; ++x) {
        if (src[IDX(x,y)]) continue;
        for (int dy=-1; dy<=1; ++dy) for (int dx=-1; dx<=1; ++dx) {
          int nx = x+dx, ny = y+dy;
          if (nx<0||nx>=cols||ny<0||ny>=rows) continue;
          if (src[IDX(nx,ny)]) { dst[IDX(x,y)] = 1; goto next_pix; }
        }
        next_pix: ;
      }
    }
    src.swap(dst);
  };
  for (int it=0; it<inflate_cells_; ++it) dilate_once(obstacle);

  // 10) distance transform 8-neigh (integer grid steps)
  const int INF = 1e9;
  std::vector<int> dist(rows*cols, INF);
  std::deque<std::pair<int,int>> dq;
  for (int iy=0; iy<rows; ++iy)
    for (int ix=0; ix<cols; ++ix)
      if (obstacle[IDX(ix,iy)]) { dist[IDX(ix,iy)] = 0; dq.emplace_back(ix,iy); }

  if (dq.empty()) std::fill(dist.begin(), dist.end(), std::max(rows,cols)*2);

  const int dx8[8]={-1,0,1,-1,1,-1,0,1};
  const int dy8[8]={-1,-1,-1,0,0,1,1,1};
  while(!dq.empty()){
    auto [x,y]=dq.front(); dq.pop_front();
    int dcur = dist[IDX(x,y)];
    for(int k=0;k<8;++k){
      int nx=x+dx8[k], ny=y+dy8[k];
      if(nx<0||nx>=cols||ny<0||ny>=rows) continue;
      if(dist[IDX(nx,ny)] > dcur + 1){
        dist[IDX(nx,ny)] = dcur + 1;
        dq.emplace_back(nx,ny);
      }
    }
  }

  // 11) pilih pusat terbaik per frame
  const float cx_cell = static_cast<float>(cols) * 0.5f, cy_cell = static_cast<float>(rows) * 0.5f;
  int best_ix=-1, best_iy=-1;
  float best_clearance=-1e9f, best_center_d2=1e30f;

  for (int iy=0; iy<rows; ++iy) {
    for (int ix=0; ix<cols; ++ix) {
      if (!safe_center[IDX(ix,iy)]) continue;

      bool obstacle_free = true;
      for (const auto& o : disk_offsets_safe) {
        int jx = ix + o.first, jy = iy + o.second;
        if (jx<0||jx>=cols||jy<0||jy>=rows || obstacle[IDX(jx,jy)]) { obstacle_free = false; break; }
      }
      if (!obstacle_free) continue;

      float clearance_m = dist[IDX(ix,iy)] * cell - r_safe;
      float dcx = ((float)ix + 0.5f - cx_cell) * cell;
      float dcy = ((float)iy + 0.5f - cy_cell) * cell;
      float center_d2 = dcx*dcx + dcy*dcy;

      if (clearance_m > best_clearance + 1e-6f ||
         (std::abs(clearance_m - best_clearance) <= 1e-6f && center_d2 < best_center_d2)) {
        best_clearance = clearance_m;
        best_center_d2 = center_d2;
        best_ix=ix; best_iy=iy;
      }
    }
  }

  if (best_ix < 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No safe center found.");
    publishOnlySafe(*cloud, uv, P_plane, false, 0.f, 0.f, 0.f, msg->header, nullptr);
    return;
  }

  // 12) pusat terpilih di koordinat (u,v)
  const float cu = umin + ((float)best_ix + 0.5f) * cell;
  const float cv = vmin + ((float)best_iy + 0.5f) * cell;

  // 13) publish point hijau + centroid (di plane_frame)
  std::array<float,3> centroid_plane{};
  bool have_centroid = false;
  publishOnlySafe(*cloud, uv, P_plane, true, cu, cv, r_safe, msg->header, &centroid_plane, &have_centroid);

  if (have_centroid) {
    publishCenterCoordinates(cu, cv, centroid_plane, plane_frame_, msg->header.stamp);
  }

  // 14) voting & publish TF statis ketika stabil
  if (have_centroid) {
    pushVote(centroid_plane);
    std::array<float,3> mode_center{};
    int mode_count = 0;
    computeMode(mode_center, mode_count);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Voting: window=%zu mode_count=%d (min=%d)", votes_.size(), mode_count, min_votes_to_fix_);

    if (!safety_tf_published_ && mode_count >= min_votes_to_fix_) {
      // jika plane_frame != odom_frame, transform mode_center ke odom
      std::array<float,3> center_odom = mode_center;
      if (plane_frame_ != odom_frame_) {
        try {
          auto T_odom_from_plane = tf_buffer_->lookupTransform(
    odom_frame_, plane_frame_,
    tf2::TimePointZero);  // ambil transform terbaru
          Eigen::Isometry3d Xop = tf2::transformToEigen(T_odom_from_plane);
          Eigen::Vector3d c_plane(mode_center[0], mode_center[1], mode_center[2]);
          Eigen::Vector3d c_odom  = Xop * c_plane;
          center_odom = {static_cast<float>(c_odom.x()),
                         static_cast<float>(c_odom.y()),
                         static_cast<float>(c_odom.z())};
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "No TF %s <- %s for final TF: %s",
                      odom_frame_.c_str(), plane_frame_.c_str(), e.what());
          return;
        }
      }
      publishStaticSafetyTF(center_odom);
      safety_tf_published_ = true;
      RCLCPP_INFO(get_logger(),
        "Published static TF '%s' in '%s' at (%.3f, %.3f, %.3f)",
        safety_frame_.c_str(), odom_frame_.c_str(), center_odom[0], center_odom[1], center_odom[2]);
    }
  }
}

void LandingCircle::publishCenterCoordinates(
    float cu, float cv,
    const std::array<float,3>& centroid_plane,
    const std::string& plane_frame,
    const rclcpp::Time& stamp)
{
  auto center_pt = std::make_unique<geometry_msgs::msg::PointStamped>();
  center_pt->header.frame_id = plane_frame;
  center_pt->header.stamp = stamp;
  
  center_pt->point.x = centroid_plane[0];  // ← X SEBENARNYA!
  center_pt->point.y = centroid_plane[1];  // Y SEBENARNYA!
  center_pt->point.z = centroid_plane[2];  // Z SEBENARNYA!
  
  pub_center_point_->publish(std::move(center_pt));
}

void LandingCircle::publishOnlySafe(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                       const std::vector<std::pair<float,float>>& uv,
                                       const std::vector<Eigen::Vector3d>& P_plane,
                                       bool have_center, float cu, float cv, float r_safe,
                                       const std_msgs::msg::Header& hdr,
                                       std::array<float,3>* out_centroid_plane,
                                       bool* out_have_centroid)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr safe_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  safe_rgb->reserve(cloud.size());

  size_t kept = 0;
  double sx=0.0, sy=0.0, sz=0.0;

  if (have_center) {
    const float r2 = r_safe * r_safe;
    for (size_t i=0;i<cloud.size();++i) {
      float du = uv[i].first  - cu;
      float dv = uv[i].second - cv;
      if (du*du + dv*dv <= r2) {
        const auto& p = cloud[i];
        pcl::PointXYZRGB q;
        q.x = p.x; q.y = p.y; q.z = p.z;
        q.r = 30; q.g = 255; q.b = 30;  // hijau
        safe_rgb->push_back(q);
        ++kept;

        const auto& pp = P_plane[i]; // centroid dihitung di plane_frame
        sx += pp.x(); sy += pp.y(); sz += pp.z();
      }
    }
  }

  safe_rgb->width  = static_cast<uint32_t>(safe_rgb->size());
  safe_rgb->height = 1;
  safe_rgb->is_dense = false;

  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*safe_rgb, out);
  out.header = hdr; // frame_id tetap frame cloud; ini hanya untuk visualisasi titik hijau
  pub_->publish(out);

  if (out_have_centroid) *out_have_centroid = false;
  if (out_centroid_plane && kept > 0) {
    (*out_centroid_plane)[0] = static_cast<float>(sx / static_cast<double>(kept));
    (*out_centroid_plane)[1] = static_cast<float>(sy / static_cast<double>(kept));
    (*out_centroid_plane)[2] = static_cast<float>(sz / static_cast<double>(kept));
    if (out_have_centroid) *out_have_centroid = true;
  }

  RCLCPP_INFO(get_logger(), "Published safe points: %zu", kept);
}

void LandingCircle::pushVote(const std::array<float,3>& c)
{
  votes_.push_back(c);
  while (static_cast<int>(votes_.size()) > vote_window_) votes_.pop_front();
}

void LandingCircle::computeMode(std::array<float,3>& mode_center, int& mode_count)
{
  mode_count = 0;
  if (votes_.empty()) { mode_center = {0.f,0.f,0.f}; return; }

  const float tol2 = static_cast<float>(vote_tolerance_m_ * vote_tolerance_m_);
  for (size_t i=0;i<votes_.size();++i) {
    int cnt = 0;
    double mx=0.0,my=0.0,mz=0.0;
    for (size_t j=0;j<votes_.size();++j) {
      const float d2 = sqr(votes_[i][0]-votes_[j][0]) + sqr(votes_[i][1]-votes_[j][1]) + sqr(votes_[i][2]-votes_[j][2]);
      if (d2 <= tol2) {
        cnt++;
        mx += votes_[j][0]; my += votes_[j][1]; mz += votes_[j][2];
      }
    }
    if (cnt > mode_count) {
      mode_count = cnt;
      mode_center[0] = static_cast<float>(mx / cnt);
      mode_center[1] = static_cast<float>(my / cnt);
      mode_center[2] = static_cast<float>(mz / cnt);
    }
  }
}

void LandingCircle::publishStaticSafetyTF(const std::array<float,3>& p_odom)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp.sec = 0;   // penting untuk /tf_static (latched)
  t.header.stamp.nanosec = 0;
  t.header.frame_id = odom_frame_;
  t.child_frame_id  = safety_frame_;
  t.transform.translation.x = p_odom[0];
  t.transform.translation.y = p_odom[1];
  t.transform.translation.z = p_odom[2];
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(t);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingCircle>());
  rclcpp::shutdown();
  return 0;
}
