import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'armrs_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mwatma',
    maintainer_email='8626150+mwsatman@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ROS2_sim = armrs_package.ROS2_sim:main',
            'ROS2_sensors = armrs_package.ROS2_sensors:main',
            'ROS2_visualizer = armrs_package.ROS2_visualizer:main',
            'ROS2_controller = armrs_package.ROS2_controller:main',
            'ROS2_dist_controller = armrs_package.ROS2_dist_controller:main',
            'ROS2_fleet_evaluator = armrs_package.ROS2_fleet_evaluator:main',
        ],
    },
)
