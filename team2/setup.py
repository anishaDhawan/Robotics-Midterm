from setuptools import find_packages, setup

package_name = 'team2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jia',
    maintainer_email='jia@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal1 = team2.goal1:main',
            'goal2 = team2.goal2:main',
            'goal3 = team2.goal3:main',
        ],
    },
)
