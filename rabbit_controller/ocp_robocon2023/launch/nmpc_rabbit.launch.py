import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('ocp_robocon2023'),
        'params',
        'mpc_params.yaml'
    )

    node = Node(
        package='ocp_robocon2023',
        name = 'nmpc_rabbit_node',
        executable= 'nmpc_rabbit_v2',
        parameters=[config]
    )

    launch_description.add_action(node)

    return launch_description