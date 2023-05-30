import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    params = os.path.join(
        get_package_share_directory('ocp_robocon2023'),
        'params',
        'nmpc_params.yaml'
        )
        
    node=Node(
        package = 'ocp_robocon2023',
        name = 'nmpc_params_node',
        executable = 'nmpc_params',
        parameters = [params]
    )

    ld.add_action(node)
    return ld