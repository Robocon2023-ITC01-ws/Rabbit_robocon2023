import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('rabbit_control'),
        'params',
        'nmpc_params.yaml'
        )
        
    node=Node(
        package = 'rabbit_control',
        name = 'nmpc_rabbit',
        executable = 'nmpc_params2',
        parameters = [config]
    )

    ld.add_action(node)
    return ld