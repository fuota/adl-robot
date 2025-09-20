# Simple launch for ArUco detection only (for testing)
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    camera_serial_no_arg = DeclareLaunchArgument(
        'camera_serial_no',
        default_value='',
        description='RealSense camera serial number'
    )
    
    show_debug_image_arg = DeclareLaunchArgument(
        'show_debug_image',
        default_value='false',
        description='Show debug image with detected markers (requires image_view package)'
    )
    
    # Get launch configurations
    camera_serial_no = LaunchConfiguration('camera_serial_no')
    show_debug_image = LaunchConfiguration('show_debug_image')
    
    # Package directories
    adl_robot_share = FindPackageShare('adl_robot')
    realsense_share = FindPackageShare('realsense2_camera')
    
    # Configuration file
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
    
    # Static transform for camera (simple setup for testing)
    camera_base_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_transform',
        arguments=[
            '0.0', '0.0', '0.5',  # x, y, z translation (50cm above base)
            '0.0', '0.0', '0.0', '1.0',  # x, y, z, w quaternion (no rotation)
            'base_link', 'camera_link'
        ],
        output='screen'
    )
    
    # Image view for debugging (optional, only if image_view package is installed)
    image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='debug_image_view',
        remappings=[('image', '/aruco_debug_image')],
        condition=IfCondition(show_debug_image),
        output='screen'
    )

    return LaunchDescription([
        camera_serial_no_arg,
        show_debug_image_arg,
        realsense_launch,
        camera_base_transform,
        aruco_detector_node,
        image_view_node,
    ])