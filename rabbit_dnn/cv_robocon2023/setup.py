from setuptools import setup

import os
from glob import glob
from urllib.request import urlretrieve
from setuptools import find_packages

package_name = 'cv_robocon2023'
setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kenotic',
    maintainer_email='kenotic@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node_v1 = cv_robocon2023.image_publisher:main',
            'camera_node_v2 = cv_robocon2023.image_publisher_cv:main',
            'yolov5_node = cv_robocon2023.yolov5_detection:main',
        ],
    },
)
