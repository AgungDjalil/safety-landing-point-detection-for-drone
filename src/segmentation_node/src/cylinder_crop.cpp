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
      declare_parameter<std::string>("input_topic", "/zed/zed_node/point_cloud/cloud_registered");
      declare_parameter<std::string>("output_topic", "/circle_cloud");

      declare_parameter<double>("diameter_m", 5.0);
      declare_parameter<double>("inner_diameter_m", 0.0);
      declare_parameter<double>("center_y", 0.0);
      declare_parameter<double>("center_z", 0.0);

      declare_parameter<bool>("use_x_filter", false);
      declare_parameter<double>("x_min", -10.0);
      declare_parameter<double>("x_max", 10.0);

      declare_parameter<double>("leaf_size", 0.05); // meter

      const auto in = get_parameter("input_topic").as_string();
      const auto out = get_parameter("output_topic").as_string();

      sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          in, rclcpp::SensorDataQoS(),
          std::bind(&CylinderCrop::cb, this, std::placeholders::_1));

      pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, 10);

      RCLCPP_INFO(get_logger(), "circle_crop (axis=X, VoxelGrid+OpenMP, RGB): %s -> %s", in.c_str(), out.c_str());
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud);

      if (cloud->empty())
        return;

      const double cy = get_parameter("center_y").as_double();
      const double cz = get_parameter("center_z").as_double();
      const double d = get_parameter("diameter_m").as_double();
      const double din = get_parameter("inner_diameter_m").as_double();
      const bool usex = get_parameter("use_x_filter").as_bool();
      const double xmin = get_parameter("x_min").as_double();
      const double xmax = get_parameter("x_max").as_double();
      const double leaf = get_parameter("leaf_size").as_double();

      const double r_out = 0.5 * d;
      const double r_in = 0.5 * std::max(0.0, din);
      const double r2o = r_out * r_out;
      const double r2i = r_in * r_in;

      pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
      {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(leaf, leaf, leaf);
        vg.filter(*down);
      }
      if (down->empty())
        return;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
      out_rgb->reserve(down->size());

      const uint8_t OR_R = 255, OR_G = 165, OR_B = 0;

      std::vector<std::vector<pcl::PointXYZRGB>> buckets(
        #ifdef _OPENMP
                std::max(1, omp_get_max_threads())
        #else
                1
        #endif
      );

      #pragma omp parallel for
      for (long i = 0; i < static_cast<long>(down->points.size()); ++i)
      {
        const auto &p = down->points[static_cast<size_t>(i)];
        if (usex && (p.x < xmin || p.x > xmax))
          continue;

        const double dy = p.y - cy;
        const double dz = p.z - cz;
        const double d2 = dy * dy + dz * dz; 
        if (d2 <= r2o && d2 >= r2i)
        {
          pcl::PointXYZRGB q;
          q.x = p.x;
          q.y = p.y;
          q.z = p.z;

          uint32_t rgb_val = (uint32_t(OR_R) << 16 | uint32_t(OR_G) << 8 | uint32_t(OR_B));
          q.rgb = *reinterpret_cast<float *>(&rgb_val);

          #ifdef _OPENMP
                  buckets[omp_get_thread_num()].push_back(q);
          #else
                  buckets[0].push_back(q);
          #endif
        }
      }

      for (auto &v : buckets)
      {
        if (!v.empty())
        {
          out_rgb->insert(out_rgb->end(), v.begin(), v.end());
        }
      }

      out_rgb->width = static_cast<uint32_t>(out_rgb->size());
      out_rgb->height = 1;
      out_rgb->is_dense = false;

      sensor_msgs::msg::PointCloud2 out_msg;
      pcl::toROSMsg(*out_rgb, out_msg);
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