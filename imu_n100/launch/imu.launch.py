import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    debug_arg = DeclareLaunchArgument(
        "debug", default_value = TextSubstitution(text = "false")
    )

    port_arg = DeclareLaunchArgument(
        "port", default_value= TextSubstitution(text = "/dev/ttyUSB0")
    )

    baud_arg = DeclareLaunchArgument(
        "baud", default_value= TextSubstitution(text = "921600")
    )

    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic", default_value= TextSubstitution(text = "/imu/data")
    )

    imu_frame_arg = DeclareLaunchArgument(
        "imu_frame", default_value= TextSubstitution(text = "gyro_link")
    )

    device_type_args = DeclareLaunchArgument(
        "device_type", default_value= TextSubstitution(text = "1")
    )
    mag_pose_args = DeclareLaunchArgument(
        "mag_pose_2d_topic", default_value= TextSubstitution(text="/mag_pose_2d")
    )

    ros_imu_node = Node(
        package= 'imu_n100',
        name = 'imu_node',
        executable= 'imu',
        parameters= [{
        "debug": LaunchConfiguration('debug'),
        "port": LaunchConfiguration('port'),
        "baud": LaunchConfiguration('baud'),
        "imu_topic": LaunchConfiguration('imu_topic'),
        "imu_frame": LaunchConfiguration('imu_frame'),
        "device_type": LaunchConfiguration('device_type'),
        }]
    )


    return LaunchDescription([
        debug_arg,
        mag_pose_args,
        port_arg,
        baud_arg,
        imu_frame_arg,
        imu_topic_arg,
        device_type_args,
        ros_imu_node,
    ])