#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class Pub : public rclcpp::Node
{
public:
    Pub() : Node("lidar2px4")
    {
        // Parameter initialization
        declare_parameter("lidar_x", 0.0);
        declare_parameter("lidar_y", 0.0);
        declare_parameter("lidar_z", 0.0);
        declare_parameter("lidar_roll", 0.0);
        declare_parameter("lidar_pitch", 0.0);
        declare_parameter("lidar_yaw", 0.0);

        get_parameter("lidar_x", x_);
        get_parameter("lidar_y", y_);
        get_parameter("lidar_z", z_);
        get_parameter("lidar_roll", roll_);
        get_parameter("lidar_pitch", pitch_);
        get_parameter("lidar_yaw", yaw_);
        
        // Transform initialization
        tf2::Quaternion lidar_quaternion;
        lidar_quaternion.setRPY(roll_, pitch_, yaw_);
        lidar_quaternion.normalize();
        lidar_mount_transform_ = tf2::Transform(lidar_quaternion, tf2::Vector3(x_, y_, z_));

        tf2::Quaternion lidar_quaternion_init;

        lidar_quaternion_init.setRPY(roll_, pitch_, 0);
        lidar_quaternion_init.normalize();
        lidar_init_transform_ = tf2::Transform(lidar_quaternion, tf2::Vector3(0, 0, 0));


    lidar_quaternion = lidar_mount_transform_.getRotation();
    tf2::Vector3 xyz = lidar_mount_transform_.getOrigin();


        // Advertise and subscribe
        pub_vision_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 100000);
        sub_vision_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 1, std::bind(&Pub::odomCallback, this, std::placeholders::_1));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry &odom)
    {
        // geometry_msgs::msg::PoseStamped uav_pose;
        // uav_pose.header = odom->header;
        // uav_pose.header.frame_id = "body";

        // geometry_msgs::msg::Pose lidar_pose = odom->pose.pose;

          geometry_msgs::msg::PoseStamped lidar_pose;
             lidar_pose.header = odom.header;
             
          lidar_pose.header.frame_id = "body";
        
        

    lidar_pose.pose.position.x = odom.pose.pose.position.x;
    lidar_pose.pose.position.y = odom.pose.pose.position.y ;
    lidar_pose.pose.position.z = odom.pose.pose.position.z;
    lidar_pose.pose.orientation.x = odom.pose.pose.orientation.x;
    lidar_pose.pose.orientation.y = odom.pose.pose.orientation.y;
    lidar_pose.pose.orientation.z = odom.pose.pose.orientation.z;
    lidar_pose.pose.orientation.w = odom.pose.pose.orientation.w;

        // tf2::Transform lidar_transform;
        // tf2::fromMsg(lidar_pose, lidar_transform);

        // tf2::Transform uav_tranform = lidar_mount_transform_ * lidar_init_transform_.inverse() * lidar_transform * lidar_mount_transform_.inverse();
        // tf2::toMsg(uav_tranform, uav_pose.pose);

        pub_vision_pose_->publish(lidar_pose);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_vision_pose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vision_odom_;
    tf2::Transform lidar_mount_transform_;
    tf2::Transform lidar_init_transform_;
    double x_, y_, z_, roll_, pitch_, yaw_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pub>());
    rclcpp::shutdown();
    
    return 0;
}