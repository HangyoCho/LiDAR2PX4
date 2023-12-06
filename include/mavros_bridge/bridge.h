#ifndef REALSENSE_MAVROS_BRIDGE_H
#define REALSENSE_MAVROS_BRIDGE_H


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

class Bridge{
public:
    Bridge(ros::NodeHandle& nh);

protected:
    ros::NodeHandle& nh_;

    double x_=0, y_=0, z_=0, roll_=0, pitch_=0, yaw_=0;  // camera's pose w.r.t. FCU mavros origin (FLU)
    tf2::Transform lidar_mount_transform_;
    tf2::Transform lidar_init_transform_;
    tf2::Transform lidar_mount_transform_inverse_;
    tf2::Transform lidar_init_transform_inverse_;

    ros::Publisher pub_vision_pose_;
    ros::Subscriber sub_vision_odom_;

    void odomCallback_(const nav_msgs::Odometry& odom);

};

#endif
