from setuptools import setup
import os
from glob import glob

package_name = 'team2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line tells colcon build to include all files in the launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install the RGB-D camera settings used by the optional House launcher.
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drpuc',
    maintainer_email='drpuc@todo.todo',
    description='Team 2 navigation, exploration and RGB-D human detection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigation = team2.navigation:main',
            # Make the exploration node available through ros2 run.
            'explore = team2.explore:main',
            'human_location_node = team2.human_location_node:main',
            # Make the depth-obstacle node available through ros2 run.
            'depth_obstacles = team2.depth_obstacles:main',
        ],
    },
)
