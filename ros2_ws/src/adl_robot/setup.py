from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'adl_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'opencv-contrib-python',
        'numpy',
        'scipy',
        'transforms3d',
    ],
    zip_safe=True,
    maintainer='tripham',
    maintainer_email='tripham@todo.todo',
    description='ADL Robot package for object detection and manipulation using ArUco markers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = adl_robot.aruco_detector:main',
            'object_pickup_service = adl_robot.object_pickup_service:main',
            'kinova_controller = adl_robot.kinova_controller:main',
            'adl_coordinator = adl_robot.adl_coordinator:main',
        ],
    },
)
