import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro
def generate_launch_description():

    return LaunchDescription([
        Node(package='joint_state_publisher', executable='joint_state_publisher', output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', output='screen'),
        
    ])