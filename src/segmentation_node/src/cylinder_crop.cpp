#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif

class CylinderCrop : public rclcpp::Node
{
public:
  CylinderCrop() : Node("cylinder_crop")
  {
    // ---------- topics ----------
    declare_parameter<std::string>("input_topic",  "/depth_camera/points");
    declare_parameter<std::string>("output_topic", "/circle_cloud");

    // ---------- cylinder geometry (filter di bidang X-Y) ----------
    declare_parameter<double>("diameter_m",       5.0);   // diameter luar (m)
    declare_parameter<double>("inner_diameter_m", 0.0);   // diameter dalam / hollow (m)
    declare_parameter<double>("center_x",         0.0);   // pusat silinder sumbu X
    declare_parameter<double>("center_y",         0.0);   // pusat silinder sumbu Y

    // ---------- filter kedalaman (sumbu Z = depth kamera bawah) ----------
    declare_parameter<bool>  ("use_z_filter", false);
    declare_parameter<double>("z_min",        -100.0);
    declare_parameter<double>("z_max",         100.0);

    // ---------- voxel downsampling ----------
    declare_parameter<double>("leaf_size", 0.05);  // meter

    const auto in  = get_parameter("input_topic").as_string();
    const auto out = get_parameter("output_topic").as_string();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        in, rclcpp::SensorDataQoS(),
        std::bind(&CylinderCrop::cb, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, 10);

    RCLCPP_INFO(get_logger(),
                "[cylinder_crop] axis=Z (depth), filter bidang X-Y: %s -> %s",
                in.c_str(), out.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;

  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ---- 1. Konversi ROS msg -> PCL ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Point cloud kosong, skip.");
      return;
    }

    // ---- 2. Baca parameter ----
    const double cx      = get_parameter("center_x").as_double();
    const double cy      = get_parameter("center_y").as_double();
    const double radius_out = 0.5 * get_parameter("diameter_m").as_double();
    const double radius_in  = 0.5 * std::max(0.0, get_parameter("inner_diameter_m").as_double());
    const bool   use_z   = get_parameter("use_z_filter").as_bool();
    const double z_min   = get_parameter("z_min").as_double();
    const double z_max   = get_parameter("z_max").as_double();
    const double leaf    = get_parameter("leaf_size").as_double();

    const double r2_out = radius_out * radius_out;
    const double r2_in  = radius_in  * radius_in;

    // ---- 3. Pre-filter: buang NaN dan inf SEBELUM VoxelGrid ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr clean(new pcl::PointCloud<pcl::PointXYZ>);
    clean->reserve(cloud->size());
    for (const auto &p : cloud->points)
    {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
        clean->push_back(p);
    }

    if (clean->empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Semua titik NaN/inf, skip.");
      return;
    }

    // ---- 4. Voxel Grid downsampling ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(clean);
      vg.setLeafSize(static_cast<float>(leaf),
                     static_cast<float>(leaf),
                     static_cast<float>(leaf));
      vg.filter(*down);
    }

    if (down->empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Point cloud kosong setelah downsampling, skip.");
      return;
    }

    // ---- 5. Filter silinder di bidang X-Y (kamera menghadap bawah) ----
    //   - Z kamera = depth (ke bawah)
    //   - Filter jarak dihitung di bidang X-Y (horizontal)
    //   - Opsional: filter Z untuk membatasi rentang ketinggian

    const int n_threads =
#ifdef _OPENMP
        std::max(1, omp_get_max_threads());
#else
        1;
#endif

    std::vector<std::vector<pcl::PointXYZRGB>> buckets(n_threads);

    constexpr uint8_t COL_R = 255, COL_G = 165, COL_B = 0;  // oranye

    #pragma omp parallel for schedule(static)
    for (long i = 0; i < static_cast<long>(down->points.size()); ++i)
    {
      const auto &p = down->points[static_cast<size_t>(i)];

      // Buang titik NaN / inf ✅
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;

      // Filter kedalaman (Z) — opsional
      if (use_z && (p.z < z_min || p.z > z_max))
        continue;

      // Filter silinder di bidang X-Y ✅
      const double dx = p.x - cx;
      const double dy = p.y - cy;
      const double d2 = dx * dx + dy * dy;

      if (d2 <= r2_out && d2 >= r2_in)
      {
        pcl::PointXYZRGB q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;

        const uint32_t rgb_val =
            (static_cast<uint32_t>(COL_R) << 16) |
            (static_cast<uint32_t>(COL_G) <<  8) |
             static_cast<uint32_t>(COL_B);
        q.rgb = *reinterpret_cast<const float *>(&rgb_val);

#ifdef _OPENMP
        buckets[omp_get_thread_num()].push_back(q);
#else
        buckets[0].push_back(q);
#endif
      }
    }

    // ---- 6. Gabungkan hasil dari semua thread ----
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    {
      std::size_t total = 0;
      for (const auto &v : buckets) total += v.size();
      out_cloud->reserve(total);
      for (auto &v : buckets)
        out_cloud->insert(out_cloud->end(), v.begin(), v.end());
    }

    out_cloud->width    = static_cast<uint32_t>(out_cloud->size());
    out_cloud->height   = 1;
    out_cloud->is_dense = false;

    // ---- 7. Debug log ----
    RCLCPP_DEBUG(get_logger(),
                 "Input: %zu pts | Clean: %zu pts | Downsampled: %zu pts | Output: %zu pts",
                 cloud->size(), clean->size(), down->size(), out_cloud->size());

    // ---- 8. Publish ----
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*out_cloud, out_msg);
    out_msg.header = msg->header;
    pub_->publish(out_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CylinderCrop>());
  rclcpp::shutdown();
  return 0;
}