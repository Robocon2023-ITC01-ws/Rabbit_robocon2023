import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    

    return LaunchDescription([
        Node(package='rabbit_can',executable='can2'),
        Node(package='rabbit_can',executable='odometry'),
        Node(package='rabbit_shooter', executable='shooter'),
        #Node(package='joy', executable='joy_node'),
        #Node(package='rabbit_remote', executable='ps4_node'),
    ])
