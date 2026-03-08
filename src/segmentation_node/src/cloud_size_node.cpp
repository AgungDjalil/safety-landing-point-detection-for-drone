#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>

using std::placeholders::_1;

class CloudSizeNode : public rclcpp::Node
{
public:
    CloudSizeNode() : Node("cloud_size_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed/zed_node/point_cloud/cloud_registered",
            rclcpp::SensorDataQoS(),
            std::bind(&CloudSizeNode::callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Cloud Size Node (Valid Counter) Started");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        size_t total_grid = msg->width * msg->height;
        size_t valid_points = 0;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (std::isfinite(*iter_x) &&
                std::isfinite(*iter_y) &&
                std::isfinite(*iter_z))
            {
                valid_points++;
            }
        }

        double percentage = 0.0;
        if (total_grid > 0)
        {
            percentage = (static_cast<double>(valid_points) /
                          static_cast<double>(total_grid)) * 100.0;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Total Grid: %ld | Valid: %ld | Valid %%: %.2f%% | Data: %ld bytes",
                    total_grid,
                    valid_points,
                    percentage,
                    msg->data.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudSizeNode>());
    rclcpp::shutdown();
    return 0;
}