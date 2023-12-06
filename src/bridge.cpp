#include "mavros_bridge/bridge.h"

Bridge::Bridge(ros::NodeHandle& nh): nh_(nh){
    if(nh.getParam("lidar_x", x_)){
        ROS_INFO("Got param lidar_x: %f", x_);
    }
    else{
        ROS_ERROR("Failed to get param lidar_x, using the default value 0");
    }

    if(nh.getParam("lidar_y", y_)){
        ROS_INFO("Got param lidar_y: %f", y_);
    }
    else{
        ROS_ERROR("Failed to get param lidar_y, using the default value 0");
    }

    if(nh.getParam("lidar_z", z_)){
        ROS_INFO("Got param lidar_z: %f", z_);
    }
    else{
        ROS_ERROR("Failed to get param lidar_z, using the default value 0");
    }

    if(nh.getParam("lidar_roll", roll_)){
        ROS_INFO("Got param lidar_roll: %f", roll_);
    }
    else{
        ROS_ERROR("Failed to get param lidar_roll, using the default value 0");
    }

    if(nh.getParam("lidar_pitch", pitch_)){
        ROS_INFO("Got param lidar_pitch: %f", pitch_);
    }
    else{
        ROS_ERROR("Failed to get param lidar_pitch, using the default value 0");
    }

    if(nh.getParam("lidar_yaw", yaw_)){
        ROS_INFO("Got param lidar_yaw: %f", yaw_);
    }
    else{
        ROS_ERROR("Failed to get param lidar_yaw, using the default value 0");
    }

    tf2::Quaternion lidar_quaternion;
    lidar_quaternion.setRPY( roll_, pitch_, yaw_);
    lidar_quaternion.normalize();

    lidar_mount_transform_ = tf2::Transform(lidar_quaternion, tf2::Vector3(x_, y_, z_));
    
    tf2::Quaternion lidar_quaternion_init;
    lidar_quaternion_init.setRPY( roll_, pitch_, 0);
    lidar_quaternion_init.normalize();
    lidar_init_transform_ = tf2::Transform(lidar_quaternion, tf2::Vector3(0, 0, 0));

    lidar_quaternion = lidar_mount_transform_.getRotation();
    tf2::Vector3 xyz = lidar_mount_transform_.getOrigin();
    ROS_INFO_STREAM("LiDAR position: \n"<<tf2::toMsg(xyz));

    geometry_msgs::Quaternion lidar_quaternion_msg = tf2::toMsg(lidar_quaternion);
    ROS_INFO_STREAM("LiDAR quaternion: \n" << lidar_quaternion_msg);
    
    lidar_mount_transform_inverse_ = lidar_mount_transform_.inverse();
    lidar_init_transform_inverse_ = lidar_init_transform_.inverse();

    pub_vision_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100000);
    sub_vision_odom_ = nh_.subscribe("/Odometry", 1, &Bridge::odomCallback_, this);
}

void Bridge::odomCallback_(const nav_msgs::Odometry& odom){
    geometry_msgs::PoseStamped uav_pose;
    uav_pose.header = odom.header;
    uav_pose.header.frame_id = "body";

    geometry_msgs::Pose lidar_pose = odom.pose.pose;
    tf2::Transform lidar_transform;
    tf2::fromMsg(lidar_pose, lidar_transform);

    tf2::Transform uav_tranform = lidar_mount_transform_*lidar_init_transform_inverse_*lidar_transform*lidar_mount_transform_inverse_;
    tf2::toMsg(uav_tranform, uav_pose.pose);

    pub_vision_pose_.publish(uav_pose);
}
