#include <ros/ros.h>
#include "mavros_bridge/bridge.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "bridge_ros_node");
	ros::NodeHandle nh;
	Bridge bridge(nh);

	ros::spin();

 	std::cout<<"Done!"<<std::endl;
	return 0;
}
