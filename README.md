## Topic
Node [/bridge_ros_node]  
Publications: 
 * /mavros/vision_pose/pose [geometry_msgs/PoseStamped]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /Odometry [nav_msgs/Odometry]

## Dependencies


## Installation

    mkdir -p ~/ws_px4_utils/src
    cd ~/ws_px4_utils/src
    gitclone https://github.com/HangyoCho/LiDAR2PX4.git
    cd ..
    catkin build
    source devel/setup.bash

## Usage

sudo chmod 777 /dev/tty*

cd ~/ws_px4_utils
sd
roslaunch lidar2px4 mavros_lio.launch 


