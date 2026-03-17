// file: landing_circle.hpp
// Header untuk versi optimasi safe_landing_color.cpp

#pragma once

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Geometry>

// STL
#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Struct hasil publishOnlySafe — didefinisikan di sini agar .cpp bisa pakai return type-nya
struct SafeResult {
  std::array<float, 3> centroid{};
  bool   valid = false;
  size_t size  = 0;
};

class LandingCircle : public rclcpp::Node
{
public:
  LandingCircle();

private:
  // ── Callback utama ────────────────────────────────────────────────────────
  void cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // ── Helpers publish ───────────────────────────────────────────────────────
  SafeResult publishOnlySafe(
      const pcl::PointCloud<pcl::PointXYZ>&        cloud,
      const std::vector<std::pair<float, float>>&  uv,
      const std::vector<Eigen::Vector3d>&           P_plane,
      bool  have_center,
      float cu, float cv, float r_safe,
      const std_msgs::msg::Header& hdr);

  void publishCenterCoordinates(
      float cu, float cv,
      const std::array<float, 3>& centroid_plane,
      const std::string& frame,
      const rclcpp::Time& stamp);

  void publishStats(
      const std::chrono::high_resolution_clock::time_point& t_start,
      size_t valid_pts,
      size_t total_pts,
      size_t safe_size);

  void publishStaticSafetyTF(const std::array<float, 3>& p_odom);

  // ── Voting ────────────────────────────────────────────────────────────────
  void pushVote(const std::array<float, 3>& c);
  void computeMode(std::array<float, 3>& mode_center, int& mode_count);

  // ── Proyeksi UV (implementasi tergantung plane_axes_) ─────────────────────
  inline void pickUV(const Eigen::Vector3d& p, float& u, float& v) const
  {
    if      (plane_axes_ == "xy") { u = p.x(); v = p.y(); }
    else if (plane_axes_ == "xz") { u = p.x(); v = p.z(); }
    else                           { u = p.y(); v = p.z(); } // default "yz"
  }

  // ── Helper kuadrat ─────────────────────────────────────────────────────────
  static inline float sqr(float x) { return x * x; }

  // ── Parameter ROS ─────────────────────────────────────────────────────────
  std::string input_topic_, output_topic_;
  std::string plane_frame_, odom_frame_, safety_frame_;
  std::string plane_axes_;

  double safe_diameter_;
  double grid_cell_;
  int    min_pts_cell_;
  double fill_ratio_req_;
  bool   close_gaps_;
  int    inflate_cells_;

  int    vote_window_;
  double vote_tolerance_m_;
  int    min_votes_to_fix_;
  double relock_dist_m_;           // [OPT-6] parameter baru

  // ── TF ────────────────────────────────────────────────────────────────────
  std::unique_ptr<tf2_ros::Buffer>                    tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>         tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // ── Publishers & subscriber ───────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_center_point_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_stats_;

  // ── State voting ──────────────────────────────────────────────────────────
  std::deque<std::array<float, 3>> votes_;
  bool                             safety_tf_published_ = false;
  std::array<float, 3>             locked_center_odom_  = {};  // [OPT-6]

  // ── Cache disk offsets — [OPT-7] rebuilt hanya saat R berubah ────────────
  std::vector<std::pair<int, int>> disk_offsets_safe_;
  int                              cached_R_safe_cells_ = -1;
};