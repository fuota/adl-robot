#!/usr/bin/env python3
"""
ArUco Object Detection Node for ADL Robot

This node detects ArUco markers in RealSense camera images and publishes
detected object information including 3D poses for robotic manipulation.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from adl_robot.msg import DetectedObject, DetectedObjectArray
import transforms3d.euler as euler

class ArucoDetector(Node):
    """
    ROS2 Node for detecting ArUco markers and estimating object poses
    """
    
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Initialize parameters
        self.declare_parameter('aruco_dict_type', 'DICT_4X4_250')
        self.declare_parameter('marker_size', 0.05)  # 5cm markers
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        aruco_dict_name = self.get_parameter('aruco_dict_type').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        # Initialize ArUco detector
        aruco_dict_mapping = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        }
        
        self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_mapping[aruco_dict_name])
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Object type mapping (ArUco ID -> Object Type)
        self.object_mapping = {
            0: "book",
            1: "water_bottle", 
            2: "cup",
            3: "bowl",
            4: "phone",
            5: "remote",
            # Add more mappings as needed
        }
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # TF2 buffer and listener for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.detected_objects_pub = self.create_publisher(
            DetectedObjectArray, 'detected_objects', 10)
        self.debug_image_pub = self.create_publisher(
            Image, 'aruco_debug_image', 1)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 1)
        
        # Store latest depth image
        self.latest_depth_image = None
        
        self.get_logger().info("ArUco Detector Node initialized")
    
    def camera_info_callback(self, msg):
        """Process camera calibration information"""
        if not self.camera_info_received:
            # Extract camera intrinsic parameters
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera calibration parameters received")
    
    def depth_callback(self, msg):
        """Store the latest depth image"""
        try:
            self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")
    
    def image_callback(self, msg):
        """Process RGB camera images for ArUco detection"""
        if not self.camera_info_received:
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect ArUco markers
            detected_objects = self.detect_aruco_markers(cv_image, msg.header)
            
            # Publish detected objects
            if detected_objects.objects:
                self.detected_objects_pub.publish(detected_objects)
            
            # Publish debug image
            debug_image = self.create_debug_image(cv_image, detected_objects)
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def detect_aruco_markers(self, image, header):
        """
        Detect ArUco markers and estimate their 3D poses
        
        Args:
            image: OpenCV image
            header: ROS message header
            
        Returns:
            DetectedObjectArray: Array of detected objects with poses
        """
        detected_objects = DetectedObjectArray()
        detected_objects.header = header
        
        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            # Estimate pose for each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            for i, marker_id in enumerate(ids.flatten()):
                # Create detected object message
                detected_obj = DetectedObject()
                detected_obj.marker_id = int(marker_id)
                detected_obj.object_type = self.object_mapping.get(marker_id, "unknown")
                
                # Convert rotation vector to rotation matrix then to quaternion
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                
                # Create pose message
                pose_msg = PoseStamped()
                pose_msg.header = header
                pose_msg.header.frame_id = self.camera_frame
                
                # Set position (translation vector)
                pose_msg.pose.position.x = float(tvec[0])
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])
                
                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1] 
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]
                
                detected_obj.pose = pose_msg
                
                # Calculate bounding box
                corner_points = corners[i][0]
                x_coords = corner_points[:, 0]
                y_coords = corner_points[:, 1]
                
                detected_obj.bbox_x = int(np.min(x_coords))
                detected_obj.bbox_y = int(np.min(y_coords))
                detected_obj.bbox_width = int(np.max(x_coords) - np.min(x_coords))
                detected_obj.bbox_height = int(np.max(y_coords) - np.min(y_coords))
                
                # Set confidence (could be improved with additional validation)
                detected_obj.confidence = 0.9  # High confidence for valid ArUco detections
                
                # Set timestamp
                detected_obj.timestamp = header.stamp
                
                # Add depth information if available
                if self.latest_depth_image is not None:
                    detected_obj = self.add_depth_information(detected_obj, corner_points)
                
                detected_objects.objects.append(detected_obj)
                
                self.get_logger().debug(f"Detected {detected_obj.object_type} (ID: {marker_id}) "
                                      f"at position: {tvec}")
        
        return detected_objects
    
    def add_depth_information(self, detected_obj, corner_points):
        """
        Add depth information to improve 3D pose estimation
        
        Args:
            detected_obj: DetectedObject message
            corner_points: ArUco marker corner points in image coordinates
            
        Returns:
            DetectedObject: Updated object with refined depth information
        """
        try:
            # Calculate center point of marker
            center_x = int(np.mean(corner_points[:, 0]))
            center_y = int(np.mean(corner_points[:, 1]))
            
            # Sample depth values around the center
            depth_region = self.latest_depth_image[
                max(0, center_y-5):min(self.latest_depth_image.shape[0], center_y+5),
                max(0, center_x-5):min(self.latest_depth_image.shape[1], center_x+5)
            ]
            
            # Filter out zero/invalid depth values
            valid_depths = depth_region[depth_region > 0]
            
            if len(valid_depths) > 0:
                # Use median depth for robustness
                depth_mm = np.median(valid_depths)
                depth_m = depth_mm / 1000.0  # Convert mm to meters
                
                # Update Z coordinate with depth information
                detected_obj.pose.pose.position.z = depth_m
                
        except Exception as e:
            self.get_logger().warn(f"Error adding depth information: {str(e)}")
        
        return detected_obj
    
    def rotation_matrix_to_quaternion(self, R):
        """
        Convert rotation matrix to quaternion
        
        Args:
            R: 3x3 rotation matrix
            
        Returns:
            tuple: (x, y, z, w) quaternion
        """
        # Use transforms3d for reliable conversion
        quat_wxyz = euler.mat2quat(R)
        # Convert from (w, x, y, z) to (x, y, z, w)
        return (quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0])
    
    def create_debug_image(self, image, detected_objects):
        """
        Create debug image with ArUco markers and object labels
        
        Args:
            image: Original OpenCV image
            detected_objects: DetectedObjectArray with detected objects
            
        Returns:
            OpenCV image with debug annotations
        """
        debug_image = image.copy()
        
        for obj in detected_objects.objects:
            # Draw bounding box
            x, y, w, h = obj.bbox_x, obj.bbox_y, obj.bbox_width, obj.bbox_height
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw object label
            label = f"{obj.object_type} (ID: {obj.marker_id})"
            cv2.putText(debug_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw coordinate axes (if pose is available)
            if self.camera_matrix is not None:
                # Convert pose back to rvec/tvec for visualization
                position = [obj.pose.pose.position.x, 
                           obj.pose.pose.position.y, 
                           obj.pose.pose.position.z]
                
                # Note: For full visualization, we'd need to convert quaternion back to rvec
                # For now, just mark the center
                center_x = x + w // 2
                center_y = y + h // 2
                cv2.circle(debug_image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        return debug_image


def main(args=None):
    """Main function to start the ArUco detector node"""
    rclpy.init(args=args)
    
    try:
        node = ArucoDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()