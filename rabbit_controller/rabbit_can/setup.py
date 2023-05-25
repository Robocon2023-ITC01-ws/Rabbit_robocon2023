from setuptools import setup

package_name = 'rabbit_can'
package_name_in = 'model_kinematic'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name_in],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotic',
    maintainer_email='chetsokhpanha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can = rabbit_can.can_pubsub:main',
            'joy = rabbit_can.teleop_joy:main',
            'rotary_node = rabbit_can.external_rotary:main',
            'can2 = rabbit_can.can_node:main',
        ],
    },
)
