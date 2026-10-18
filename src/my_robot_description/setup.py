from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),

    ('share/' + package_name,
        ['package.xml']),

    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),

    (os.path.join('share', package_name, 'urdf'),
        glob('urdf/*.urdf') + glob('urdf/*.xacro')),

    (os.path.join('share', package_name, 'worlds'),
        glob('worlds/*.sdf')),

    (os.path.join('share', package_name, 'config'),
        glob('config/*.yaml')),

    (os.path.join('share', package_name, 'maps'),
        glob('maps/map_*.yaml') + glob('maps/map_*.pgm')),

    (os.path.join('share', package_name, 'rviz'),
        glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thinh',
    maintainer_email='thinh@example.com',
    description='Differential-drive robot: description, Gazebo simulation, SLAM and Nav2 bringup',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)