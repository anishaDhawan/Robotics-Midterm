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
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drpuc',
    maintainer_email='drpuc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation = team2.navigation:main',
            'explore = team2.explore:main',
            'human_location_node = team2.human_location_node:main',
            'depth_obstacles = team2.depth_obstacles:main',
            'goal1 = team2.goal1:main',
            'goal2 = team2.goal2:main',
            'goal3 = team2.goal3:main',
        ],
    },
)