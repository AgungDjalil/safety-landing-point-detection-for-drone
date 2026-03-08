#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <cmath>
#include <memory>

class DroneKinematic : public rclcpp::Node 
{
    public:
        DroneKinematic(const std::string &name) : Node(name)
        {
            tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            cam_link_tf_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

            sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&DroneKinematic::odom_callback, this, std::placeholders::_1));
            map_to_odom_tf_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            
            make_transforms();
            make_map_to_odom_tf();

            RCLCPP_INFO(get_logger(), "Listening PX4 odometry on: /fmu/out/vehicle_odometry");
            RCLCPP_INFO(get_logger(), "Publishing TF: odom -> base_link");
        }

    private:
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> cam_link_tf_br_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> map_to_odom_tf_br_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;

        void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
        {
            double xe, yn, zu;
            ned_to_enu_pos(msg->position[0], msg->position[1], msg->position[2], xe, yn, zu);
            auto qenu = quat_ned_to_enu(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
            const double qw=qenu[0], qx=qenu[1], qy=qenu[2], qz=qenu[3];

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = stamp_now();
            t.header.frame_id = "odom";
            t.child_frame_id  = "base_link";
            t.transform.translation.x = xe;
            t.transform.translation.y = yn;
            t.transform.translation.z = zu;
            t.transform.rotation.w = qw;
            t.transform.rotation.x = qx;
            t.transform.rotation.y = qy;
            t.transform.rotation.z = qz;
            tf_br_->sendTransform(t);   
        }

        void make_map_to_odom_tf()
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = rclcpp::Time(0);  // static transform
            t.header.frame_id = "map";
            t.child_frame_id = "odom";

            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;

            map_to_odom_tf_br_->sendTransform(t);
            RCLCPP_INFO(this->get_logger(), "Static TF published (map → odom)");
        }

        void make_transforms()
        {
            geometry_msgs::msg::TransformStamped transform;

            transform.header.stamp = rclcpp::Time(0);
            transform.header.frame_id = "base_link";
            transform.child_frame_id = "camera_link";

            transform.transform.translation.x = 0.30;
            transform.transform.translation.y = 0.03;
            transform.transform.translation.z = 0.21;

            tf2::Quaternion q;
            q.setRPY(0.0, 1.57, 0.0);
            transform.transform.rotation = tf2::toMsg(q);

            cam_link_tf_br_->sendTransform(transform);

            RCLCPP_INFO(this->get_logger(), "Static TF published (base_link → camera_link)");
        }

        builtin_interfaces::msg::Time stamp_now() 
        {
            const rclcpp::Time t = this->get_clock()->now();
            builtin_interfaces::msg::Time out;
            const int64_t ns = t.nanoseconds();
            out.sec     = static_cast<int32_t>(ns / 1000000000LL);
            out.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
            return out;
        }

        void ned_to_enu_pos(float xn, float ye, float zd, double &xe, double &yn, double &zu) 
        {
            xe = static_cast<double>(ye);
            yn = static_cast<double>(xn);
            zu = static_cast<double>(-zd);
        }

        std::array<double,4> quat_ned_to_enu(double w, double x, double y, double z)
        {
            const double xx = x*x, yy = y*y, zz = z*z;
            const double wx = w*x, wy = w*y, wz = w*z;
            const double xy = x*y, xz = x*z, yz = y*z;

            double R[3][3] = {
                {1-2*(yy+zz), 2*(xy - wz),   2*(xz + wy)},
                {2*(xy + wz), 1-2*(xx+zz),   2*(yz - wx)},
                {2*(xz - wy), 2*(yz + wx),   1-2*(xx+yy)}
            };

            const double Cw[3][3] = {{0,1,0},{1,0,0},{0,0,-1}};     // world
            const double Cb[3][3] = {{1,0,0},{0,-1,0},{0,0,-1}};    // body

            double T[3][3];
            for (int i=0;i<3;i++)
                for (int j=0;j<3;j++)
                T[i][j] = R[i][0]*Cb[0][j] + R[i][1]*Cb[1][j] + R[i][2]*Cb[2][j];

            double Re[3][3];
            for (int i=0;i<3;i++)
                for (int j=0;j<3;j++)
                Re[i][j] = Cw[i][0]*T[0][j] + Cw[i][1]*T[1][j] + Cw[i][2]*T[2][j];

            const double tr = Re[0][0] + Re[1][1] + Re[2][2];
            double qw,qx,qy,qz;
            if (tr > 0.0) {
                const double S = std::sqrt(tr + 1.0)*2.0;
                qw = 0.25 * S;
                qx = (Re[2][1] - Re[1][2]) / S;
                qy = (Re[0][2] - Re[2][0]) / S;
                qz = (Re[1][0] - Re[0][1]) / S;
            } else if ((Re[0][0] > Re[1][1]) && (Re[0][0] > Re[2][2])) {
                const double S = std::sqrt(1.0 + Re[0][0] - Re[1][1] - Re[2][2]) * 2.0;
                qw = (Re[2][1] - Re[1][2]) / S;
                qx = 0.25 * S;
                qy = (Re[0][1] + Re[1][0]) / S;
                qz = (Re[0][2] + Re[2][0]) / S;
            } else if (Re[1][1] > Re[2][2]) {
                const double S = std::sqrt(1.0 + Re[1][1] - Re[0][0] - Re[2][2]) * 2.0;
                qw = (Re[0][2] - Re[2][0]) / S;
                qx = (Re[0][1] + Re[1][0]) / S;
                qy = 0.25 * S;
                qz = (Re[1][2] + Re[2][1]) / S;
            } else {
                const double S = std::sqrt(1.0 + Re[2][2] - Re[0][0] - Re[1][1]) * 2.0;
                qw = (Re[1][0] - Re[0][1]) / S;
                qx = (Re[0][2] + Re[2][0]) / S;
                qy = (Re[1][2] + Re[2][1]) / S;
                qz = 0.25 * S;
            }

            return {qw,qx,qy,qz};
        }
};

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneKinematic>("drone_kinematics"));
  rclcpp::shutdown();
  return 0;
}