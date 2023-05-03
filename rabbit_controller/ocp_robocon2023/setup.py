from setuptools import setup
import os
from glob import glob

package_name = 'ocp_robocon2023'
submodules = 'ocp_robocon2023/library'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sokhengdin',
    maintainer_email='dinsokhengbds@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_timer = ocp_robocon2023.test_timer:main",
            "test_wait_timer = ocp_robocon2023.test_wait_timer:main",
            "nmpc_elephant = ocp_robocon2023.nmpc_elephant:main",
            "fibonacci_action_server = ocp_robocon2023.fibonacci_action_server:main",
            "fibonacci_action_client = ocp_robocon2023.fibonacci_action_client:main",
            "nmpc_rabbit = ocp_robocon2023.nmpc_rabbit:main",
            "nmpc_rabbit_v2 = ocp_robocon2023.nmpc_rabbit_v2:main",
            "mpc_action_server = ocp_robocon2023.mpc_action_server:main",
            "mpc_action_client = ocp_robocon2023.mpc_action_client:main",
            "nmpc_omni = ocp_robocon2023.nmpc_omni:main",
            "nmpc_omni_v2 = ocp_robocon2023.nmpc_omni_v2:main",
            "beizer_path = ocp_robocon2023.beizer_node:main"
        ],
    },
)
