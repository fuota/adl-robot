#!/usr/bin/env python3
"""
Object Pickup Service for ADL Robot

This service node coordinates between ArUco object detection and Kinova arm
manipulation to pick up detected objects for ADL tasks.
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from adl_robot.msg import DetectedObject, DetectedObjectArray
from adl_robot.srv import PickupObject, PlaceObject
# Import KinovaController class directly instead of the module
from .kinova_controller import KinovaController
import threading
import time

class ObjectPickupService(Node):
    """
    ROS2 Service Node for coordinating object pickup operations
    """
    
    def __init__(self):
        super().__init__('object_pickup_service')
        
        # Initialize parameters
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('detection_timeout', 5.0)
        
        # Get parameters
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.detection_timeout = self.get_parameter('detection_timeout').get_parameter_value().double_value
        
        # TF2 buffer and listener for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize Kinova controller
        self.kinova_controller = KinovaController()
        
        # Service servers
        self.pickup_service = self.create_service(
            PickupObject, 'pickup_object', self.pickup_object_callback)
        self.place_service = self.create_service(
            PlaceObject, 'place_object', self.place_object_callback)
        
        # Subscriber for detected objects
        self.detected_objects_sub = self.create_subscription(
            DetectedObjectArray, 'detected_objects', self.detected_objects_callback, 10)
        
        # Store latest detected objects
        self.latest_detected_objects = []
        self.detection_lock = threading.Lock()
        
        self.get_logger().info("Object Pickup Service initialized")
    
    def detected_objects_callback(self, msg):
        """Store latest detected objects"""
        with self.detection_lock:
            self.latest_detected_objects = msg.objects
    
    def find_object_by_id(self, marker_id):
        """
        Find detected object by ArUco marker ID
        
        Args:
            marker_id: ArUco marker ID to search for
            
        Returns:
            DetectedObject or None: Found object or None if not found
        """
        with self.detection_lock:
            for obj in self.latest_detected_objects:
                if obj.marker_id == marker_id:
                    return obj
        return None
    
    def transform_pose_to_base_frame(self, pose_stamped):
        """
        Transform pose from camera frame to robot base frame
        
        Args:
            pose_stamped: PoseStamped in camera frame
            
        Returns:
            PoseStamped in base frame or None if transform fails
        """
        try:
            # Wait for transform to be available
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, 
                pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Transform pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(
                pose_stamped, transform)
            
            return transformed_pose
            
        except Exception as e:
            self.get_logger().error(f"Failed to transform pose: {str(e)}")
            return None
    
    def validate_pickup_pose(self, pose_stamped):
        """
        Validate if the pickup pose is reachable and safe
        
        Args:
            pose_stamped: Target pose in base frame
            
        Returns:
            bool: True if pose is valid and reachable
        """
        # Check if position is within robot workspace
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        z = pose_stamped.pose.position.z
        
        # Kinova Gen3 approximate workspace limits (adjust based on your setup)
        min_reach = 0.2  # meters
        max_reach = 0.9  # meters
        min_height = 0.1  # meters (table height)
        max_height = 1.2  # meters
        
        distance = (x**2 + y**2)**0.5
        
        if distance < min_reach:
            self.get_logger().warn(f"Target too close: {distance:.3f}m < {min_reach}m")
            return False
        
        if distance > max_reach:
            self.get_logger().warn(f"Target too far: {distance:.3f}m > {max_reach}m")
            return False
        
        if z < min_height:
            self.get_logger().warn(f"Target too low: {z:.3f}m < {min_height}m")
            return False
        
        if z > max_height:
            self.get_logger().warn(f"Target too high: {z:.3f}m > {max_height}m")
            return False
        
        # Additional safety checks could be added here
        # (collision detection, joint limits, etc.)
        
        return True
    
    def pickup_object_callback(self, request, response):
        """
        Service callback for pickup object requests
        
        Args:
            request: PickupObject request
            response: PickupObject response
            
        Returns:
            PickupObject response
        """
        start_time = time.time()
        
        try:
            self.get_logger().info(f"Pickup request received for marker ID: {request.marker_id}")
            
            # Method 1: Use provided pose if available
            if request.target_pose.header.frame_id:
                self.get_logger().info("Using provided target pose")
                target_pose = request.target_pose
            else:
                # Method 2: Find object from latest detections
                self.get_logger().info("Searching for object in latest detections...")
                detected_obj = self.find_object_by_id(request.marker_id)
                
                if detected_obj is None:
                    # Wait a bit and try again
                    self.get_logger().info("Object not found, waiting for detection...")
                    time.sleep(1.0)
                    detected_obj = self.find_object_by_id(request.marker_id)
                
                if detected_obj is None:
                    response.success = False
                    response.message = f"Object with marker ID {request.marker_id} not found"
                    response.execution_time = time.time() - start_time
                    return response
                
                target_pose = detected_obj.pose
                self.get_logger().info(f"Found {detected_obj.object_type} at marker ID {request.marker_id}")
            
            # Transform pose to base frame if needed
            if target_pose.header.frame_id != self.base_frame:
                self.get_logger().info(f"Transforming pose from {target_pose.header.frame_id} to {self.base_frame}")
                transformed_pose = self.transform_pose_to_base_frame(target_pose)
                
                if transformed_pose is None:
                    response.success = False
                    response.message = "Failed to transform pose to base frame"
                    response.execution_time = time.time() - start_time
                    return response
                
                target_pose = transformed_pose
            
            # Validate pickup pose
            if not self.validate_pickup_pose(target_pose):
                response.success = False
                response.message = "Target pose is not reachable or safe"
                response.execution_time = time.time() - start_time
                return response
            
            # Execute pickup sequence
            self.get_logger().info("Executing pickup sequence...")
            success = self.kinova_controller.execute_pick_sequence(target_pose)
            
            if success:
                response.success = True
                response.message = "Object pickup completed successfully"
                self.get_logger().info("Pickup operation successful!")
            else:
                response.success = False
                response.message = "Pickup sequence failed during execution"
                self.get_logger().error("Pickup operation failed!")
            
        except Exception as e:
            self.get_logger().error(f"Error during pickup operation: {str(e)}")
            response.success = False
            response.message = f"Pickup failed with error: {str(e)}"
        
        response.execution_time = time.time() - start_time
        return response
    
    def place_object_callback(self, request, response):
        """
        Service callback for place object requests
        
        Args:
            request: PlaceObject request
            response: PlaceObject response
            
        Returns:
            PlaceObject response
        """
        start_time = time.time()
        
        try:
            self.get_logger().info("Place request received")
            
            # Get target pose
            target_pose = request.target_pose
            
            # Transform pose to base frame if needed
            if target_pose.header.frame_id != self.base_frame:
                self.get_logger().info(f"Transforming pose from {target_pose.header.frame_id} to {self.base_frame}")
                transformed_pose = self.transform_pose_to_base_frame(target_pose)
                
                if transformed_pose is None:
                    response.success = False
                    response.message = "Failed to transform pose to base frame"
                    response.execution_time = time.time() - start_time
                    return response
                
                target_pose = transformed_pose
            
            # Validate place pose
            if not self.validate_pickup_pose(target_pose):  # Same validation can be used
                response.success = False
                response.message = "Target pose is not reachable or safe"
                response.execution_time = time.time() - start_time
                return response
            
            # Execute place sequence
            self.get_logger().info("Executing place sequence...")
            success = self.kinova_controller.execute_place_sequence(target_pose)
            
            if success:
                response.success = True
                response.message = "Object placement completed successfully"
                self.get_logger().info("Place operation successful!")
            else:
                response.success = False
                response.message = "Place sequence failed during execution"
                self.get_logger().error("Place operation failed!")
            
        except Exception as e:
            self.get_logger().error(f"Error during place operation: {str(e)}")
            response.success = False
            response.message = f"Place failed with error: {str(e)}"
        
        response.execution_time = time.time() - start_time
        return response


def main(args=None):
    """Main function to start the object pickup service"""
    rclpy.init(args=args)
    
    try:
        node = ObjectPickupService()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()