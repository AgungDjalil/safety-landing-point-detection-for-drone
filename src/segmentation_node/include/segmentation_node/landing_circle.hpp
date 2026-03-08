#pragma once

#include <string>
#include <vector>
#include <deque>
#include <array>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/point_stamped.hpp>

class LandingCircle : public rclcpp::Node {
    public:
        LandingCircle();

    private:
        inline void pickUV(const Eigen::Vector3d &pp, float &u, float &v) const 
        {
            if (plane_axes_ == "xy")      { u = static_cast<float>(pp.x()); v = static_cast<float>(pp.y()); }
            else if (plane_axes_ == "yz") { u = static_cast<float>(pp.y()); v = static_cast<float>(pp.z()); }
            else /* xz */                 { u = static_cast<float>(pp.x()); v = static_cast<float>(pp.z()); }
        }

        void cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

        void publishOnlySafe(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                            const std::vector<std::pair<float,float>>& uv,
                            const std::vector<Eigen::Vector3d>& P_plane,
                            bool have_center, float cu, float cv, float r_safe,
                            const std_msgs::msg::Header& hdr,
                            std::array<float,3>* out_centroid_plane,
                            bool* out_have_centroid=nullptr);

        void pushVote(const std::array<float,3>& c);
        static inline float sqr(float x){ return x*x; }
        void computeMode(std::array<float,3>& mode_center, int& mode_count);
        void publishStaticSafetyTF(const std::array<float,3>& p_odom);
        void publishCenterCoordinates(float cu, float cv,
                                    const std::array<float,3>& centroid_plane,
                                    const std::string& plane_frame,
                                    const rclcpp::Time& stamp);

        std::string input_topic_, output_topic_, plane_axes_;
        std::string plane_frame_, odom_frame_, safety_frame_;
        double safe_diameter_{1.0}, grid_cell_{0.21}, fill_ratio_req_{0.95};
        int    min_pts_cell_{20}, inflate_cells_{1};
        bool   close_gaps_{true};

        int vote_window_{15};
        double vote_tolerance_m_{0.20};
        int min_votes_to_fix_{8};
        std::deque<std::array<float,3>> votes_;
        bool safety_tf_published_{false};

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_center_point_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};
