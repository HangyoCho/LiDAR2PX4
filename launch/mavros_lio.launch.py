import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    package_path = get_package_share_directory('lidar2px4')

    x_ = LaunchConfiguration('lidar_x')
    y_ = LaunchConfiguration('lidar_y')
    z_ = LaunchConfiguration('lidar_z')
    roll_ = LaunchConfiguration('lidar_roll')
    pitch_ = LaunchConfiguration('lidar_pitch')
    yaw_ = LaunchConfiguration('lidar_yaw')

    declare_lidar_x = DeclareLaunchArgument(
        'lidar_x', default_value='0.',
        description='LiDAR x'
    )
    declare_lidar_y = DeclareLaunchArgument(
        'lidar_y', default_value='0.',
        description='LiDAR y'
    )
    declare_lidar_z = DeclareLaunchArgument(
        'lidar_z', default_value='0.',
        description='LiDAR z'
    )
    declare_lidar_roll = DeclareLaunchArgument(
       'lidar_roll', default_value='3.1415',
       description='LiDAR Roll'
    )
    declare_lidar_pitch = DeclareLaunchArgument(
       'lidar_pitch', default_value='0.',
       description='LiDAR Pitch'
    )
    declare_lidar_yaw = DeclareLaunchArgument(
       'lidar_yaw', default_value='0.',
       description='LiDAR Yaw'
    )
# x : right        y : forward         z : up
    lidar2px4 = Node(
        package='lidar2px4',
        executable='lidar2px4',
        parameters=[{'lidar_x': x_},
                    {'lidar_y': y_},
                    {'lidar_z': z_},
                    {'lidar_roll': roll_},
                    {'lidar_pitch': pitch_},
                    {'lidar_yaw': yaw_}],
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(declare_lidar_x)
    ld.add_action(declare_lidar_y)
    ld.add_action(declare_lidar_z)
    ld.add_action(declare_lidar_roll)
    ld.add_action(declare_lidar_pitch)
    ld.add_action(declare_lidar_yaw)


    ld.add_action(lidar2px4)

    return ld