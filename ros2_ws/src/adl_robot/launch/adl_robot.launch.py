#!/usr/bin/env python3
"""
Launch file for ADL Robot system

This launch file starts all necessary nodes for the ADL robot including:
- RealSense camera
- ArUco object detection
- Kinova arm control
- Object pickup service
- ADL task coordinator

Author: ADL Robot Team
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )
    
    camera_serial_no_arg = DeclareLaunchArgument(
        'camera_serial_no',
        default_value='',
        description='RealSense camera serial number (empty for any camera)'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    camera_serial_no = LaunchConfiguration('camera_serial_no')
    
    # Package directories
    adl_robot_share = FindPackageShare('adl_robot')
    realsense_share = FindPackageShare('realsense2_camera')
    
    # Configuration file paths
    adl_config_file = PathJoinSubstitution([
        adl_robot_share, 'config', 'adl_robot_config.yaml'
    ])
    
    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                realsense_share, 'launch', 'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            'serial_no': camera_serial_no,
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
        }.items()
    )
    
    # ArUco detector node
    aruco_detector_node = Node(
        package='adl_robot',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[adl_config_file],
        remappings=[
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/depth/image_rect_raw', '/camera/aligned_depth_to_color/image_raw'),
            ('/camera/color/camera_info', '/camera/color/camera_info'),
        ],
        output='screen'
    )
    
    # Kinova controller node
    kinova_controller_node = Node(
        package='adl_robot', 
        executable='kinova_controller',
        name='kinova_controller',
        parameters=[adl_config_file],
        output='screen'
    )
    
    # Object pickup service node
    object_pickup_service_node = Node(
        package='adl_robot',
        executable='object_pickup_service', 
        name='object_pickup_service',
        parameters=[adl_config_file],
        output='screen'
    )
    
    # ADL coordinator node
    adl_coordinator_node = Node(
        package='adl_robot',
        executable='adl_coordinator',
        name='adl_coordinator', 
        parameters=[adl_config_file],
        output='screen'
    )
    
    # RViz node for visualization
    rviz_config_file = PathJoinSubstitution([
        adl_robot_share, 'config', 'adl_robot_rviz.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(start_rviz),
        output='screen'
    )
    
    # Static transform publishers for camera-to-robot calibration
    # These transforms should be calibrated for your specific setup
    camera_base_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_transform',
        arguments=[
            '0.5', '0.0', '0.8',  # x, y, z translation (meters)
            '0.0', '0.5', '0.0', '0.866',  # x, y, z, w quaternion (30° pitch down)
            'base_link', 'camera_link'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        start_rviz_arg,
        camera_serial_no_arg,
        
        # Camera and transforms
        realsense_launch,
        camera_base_transform,
        
        # ADL Robot nodes
        aruco_detector_node,
        kinova_controller_node,
        object_pickup_service_node,
        adl_coordinator_node,
        
        # Visualization
        rviz_node,
    ])