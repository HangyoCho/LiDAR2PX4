#include "mavros_bridge/bridge.h"

Bridge::Bridge(ros::NodeHandle& nh): nh_(nh){

    pub_vision_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100000);
    sub_vision_odom_ = nh_.subscribe("/Odometry", 1, &Bridge::odomCallback_, this);
}

void Bridge::odomCallback_(const nav_msgs::Odometry& odom){
    geometry_msgs::PoseStamped uav_pose;

    geometry_msgs::PoseStamped lidar_pose;
        lidar_pose.header = odom.header;
    lidar_pose.header.frame_id = "body";
    
    lidar_pose.pose.position.x = odom.pose.pose.position.x;
    lidar_pose.pose.position.y = odom.pose.pose.position.y ;
    lidar_pose.pose.position.z = odom.pose.pose.position.z;
    lidar_pose.pose.orientation.x = odom.pose.pose.orientation.x;
    lidar_pose.pose.orientation.y = odom.pose.pose.orientation.y;
    lidar_pose.pose.orientation.z = odom.pose.pose.orientation.z;
    lidar_pose.pose.orientation.w = odom.pose.pose.orientation.w;

    pub_vision_pose_.publish(lidar_pose);
}
