#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <deque>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct GNGNode 
{
  Eigen::Vector3f w;
  float error = 0.f;
  std::unordered_map<int,int> neigh_age;
};

struct GNGParams 
{
  int   max_nodes      = 40;
  int   max_age        = 50;
  int   lambda_insert  = 100;
  float eps_b          = 0.05f;
  float eps_n          = 0.005f;
  float alpha          = 0.5f;
  float d              = 0.995f;
  int   train_steps    = 5000;
};

class GrowingNeuralGas 
{
  public:
    explicit GrowingNeuralGas(const GNGParams& p);

    void train(const std::vector<Eigen::Vector3f>& X, unsigned seed = 1);
    int  nearest(const Eigen::Vector3f& x) const;

    const std::vector<GNGNode>& nodes() const { return nodes_; }

  private:
    GNGParams params;
    std::vector<GNGNode> nodes_;

    void connect_(int i,int j);
    void disconnect_(int i,int j);
    void prune_old_edges_();
};

class PlaneSegmentationGNG : public rclcpp::Node 
{
  public:
    explicit PlaneSegmentationGNG(const std::string& name = "plane_segmentation_gng");

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_debug_;

    std::deque<Eigen::Vector3f> centroid_buf_;
    std::deque<Eigen::Vector3f> normal_buf_;
    int  smooth_window_{5};
    bool snap_to_plane_{true};

    void push_and_smooth_(const Eigen::Vector3f& c_new,
                          const Eigen::Vector3f& n_new,
                          Eigen::Vector3f& c_avg,
                          Eigen::Vector3f& n_avg);

    void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};
