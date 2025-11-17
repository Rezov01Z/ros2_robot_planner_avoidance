from setuptools import setup
import os
from glob import glob

package_name = 'obstacle_avoidance_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vinay Bukka',
    maintainer_email='vinay06@umd.edu',
    description='TurtleBot3 obstacle avoidance with path planning',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = obstacle_avoidance_tb3.obstacle_avoidance:main',
            'turtlebot_teleop = obstacle_avoidance_tb3.turtlebot_teleop:main',
            'path_planner = obstacle_avoidance_tb3.path_planner:main',
            'smart_navigation = obstacle_avoidance_tb3.smart_navigation:main',
        ],
    },
)