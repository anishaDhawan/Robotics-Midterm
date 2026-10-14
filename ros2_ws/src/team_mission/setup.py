from setuptools import find_packages, setup

package_name = 'team_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Team',
    maintainer_email='team@example.com',
    description='TurtleBot3 goals with priority emergency and human reporting',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mission_manager = team_mission.mission_manager:main',
        ],
    },
)
