import os
from glob import glob
from setuptools import setup

package_name = 'delta_robot_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DJIDEL Abdelali Rayan',
    maintainer_email='djidelabdelali@gmail.com',
    description='ROS 2 Package for Dual Delta Robot Simulation (Ghost IK vs Real SMC Dynamics)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delta_robot_node = delta_robot_pkg.delta_robot_node:main',
            'trajectory_publisher = delta_robot_pkg.trajectory_publisher:main',
        ],
    },
)
