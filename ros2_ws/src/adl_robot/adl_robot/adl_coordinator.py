#!/usr/bin/env python3
"""
ADL Coordinator for ADL Robot

This node provides high-level coordination for Activities of Daily Living tasks,
managing the workflow between object detection, manipulation, and user interface.

Author: ADL Robot Team
"""

import rclpy
from rclpy.node import Node
from rclpy.client import Client
import time
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from adl_robot.msg import DetectedObject, DetectedObjectArray
from adl_robot.srv import PickupObject, PlaceObject

class ADLCoordinator(Node):
    """
    ROS2 Node for coordinating high-level ADL tasks
    """
    
    def __init__(self):
        super().__init__('adl_coordinator')
        
        # Initialize parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('service_timeout', 30.0)
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.service_timeout = self.get_parameter('service_timeout').get_parameter_value().double_value
        
        # Service clients
        self.pickup_client = self.create_client(PickupObject, 'pickup_object')
        self.place_client = self.create_client(PlaceObject, 'place_object')
        
        # Subscriber for detected objects
        self.detected_objects_sub = self.create_subscription(
            DetectedObjectArray, 'detected_objects', self.detected_objects_callback, 10)
        
        # Wait for services
        self.get_logger().info("Waiting for pickup service...")
        self.pickup_client.wait_for_service()
        
        self.get_logger().info("Waiting for place service...")
        self.place_client.wait_for_service()
        
        # Store latest detected objects
        self.latest_detected_objects = []
        
        # Pre-defined locations for placing objects
        self.placement_locations = {
            'shelf_high': self.create_pose(0.6, -0.3, 0.8),
            'shelf_middle': self.create_pose(0.6, -0.3, 0.6),
            'shelf_low': self.create_pose(0.6, -0.3, 0.4),
            'table_left': self.create_pose(0.5, 0.3, 0.3),
            'table_center': self.create_pose(0.5, 0.0, 0.3),
            'table_right': self.create_pose(0.5, -0.3, 0.3),
            'drawer': self.create_pose(0.4, 0.2, 0.2),
        }
        
        self.get_logger().info("ADL Coordinator initialized")
        
        # Demo: Start a simple task after initialization
        self.create_timer(5.0, self.demo_task_timer_callback)
    
    def create_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        Create a PoseStamped message
        
        Args:
            x, y, z: Position coordinates
            qx, qy, qz, qw: Quaternion orientation
            
        Returns:
            PoseStamped message
        """
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        return pose
    
    def detected_objects_callback(self, msg):
        """Store latest detected objects"""
        self.latest_detected_objects = msg.objects
        
        # Log detected objects
        if msg.objects:
            object_info = [f"{obj.object_type}(ID:{obj.marker_id})" for obj in msg.objects]
            self.get_logger().info(f"Detected objects: {', '.join(object_info)}")
    
    def pickup_object_by_id(self, marker_id, object_type=""):
        """
        Pickup object by ArUco marker ID
        
        Args:
            marker_id: ArUco marker ID
            object_type: Optional object type for validation
            
        Returns:
            bool: Success status
        """
        try:
            # Create service request
            request = PickupObject.Request()
            request.marker_id = marker_id
            request.object_type = object_type
            request.use_precise_grasp = True
            
            # Create empty pose (service will find object from detections)
            request.target_pose = PoseStamped()
            
            self.get_logger().info(f"Requesting pickup of object with marker ID: {marker_id}")
            
            # Call service
            future = self.pickup_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Pickup successful: {response.message}")
                    self.get_logger().info(f"Execution time: {response.execution_time:.2f}s")
                    return True
                else:
                    self.get_logger().error(f"Pickup failed: {response.message}")
            else:
                self.get_logger().error("Pickup service call failed")
                
        except Exception as e:
            self.get_logger().error(f"Error during pickup: {str(e)}")
        
        return False
    
    def place_object_at_location(self, location_name):
        """
        Place object at predefined location
        
        Args:
            location_name: Name of predefined location
            
        Returns:
            bool: Success status
        """
        if location_name not in self.placement_locations:
            self.get_logger().error(f"Unknown location: {location_name}")
            return False
        
        try:
            # Create service request
            request = PlaceObject.Request()
            request.target_pose = self.placement_locations[location_name]
            request.release_object = True
            
            self.get_logger().info(f"Requesting placement at location: {location_name}")
            
            # Call service
            future = self.place_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Placement successful: {response.message}")
                    self.get_logger().info(f"Execution time: {response.execution_time:.2f}s")
                    return True
                else:
                    self.get_logger().error(f"Placement failed: {response.message}")
            else:
                self.get_logger().error("Place service call failed")
                
        except Exception as e:
            self.get_logger().error(f"Error during placement: {str(e)}")
        
        return False
    
    def execute_pick_and_place_task(self, marker_id, placement_location):
        """
        Execute complete pick and place task
        
        Args:
            marker_id: ArUco marker ID of object to pickup
            placement_location: Name of location to place object
            
        Returns:
            bool: Success status
        """
        self.get_logger().info(f"Starting pick and place task: ID {marker_id} -> {placement_location}")
        
        # Step 1: Pickup object
        if not self.pickup_object_by_id(marker_id):
            self.get_logger().error("Pick and place task failed at pickup stage")
            return False
        
        # Brief pause between pickup and place
        time.sleep(1.0)
        
        # Step 2: Place object
        if not self.place_object_at_location(placement_location):
            self.get_logger().error("Pick and place task failed at placement stage")
            return False
        
        self.get_logger().info("Pick and place task completed successfully!")
        return True
    
    def execute_adl_task_make_cereal(self):
        """
        Execute ADL task: Make a bowl of cereal
        
        This is an example of a complex ADL task broken down into steps
        """
        self.get_logger().info("=== Starting ADL Task: Make Cereal ===")
        
        tasks = [
            # (marker_id, object_type, placement_location, description)
            (2, "bowl", "table_center", "Place bowl on table"),
            (0, "cereal_box", "table_left", "Place cereal box near bowl"),
            (1, "milk_bottle", "table_right", "Place milk near bowl"),
            # Additional steps would involve pouring actions
        ]
        
        for marker_id, object_type, location, description in tasks:
            self.get_logger().info(f"Task step: {description}")
            
            # Wait for object to be detected
            max_wait_time = 10.0
            wait_start = time.time()
            
            while time.time() - wait_start < max_wait_time:
                object_found = any(obj.marker_id == marker_id for obj in self.latest_detected_objects)
                if object_found:
                    break
                time.sleep(0.5)
            
            if not object_found:
                self.get_logger().error(f"Object {object_type} (ID: {marker_id}) not detected within timeout")
                return False
            
            # Execute pick and place
            if not self.execute_pick_and_place_task(marker_id, location):
                self.get_logger().error(f"Failed to complete step: {description}")
                return False
            
            # Pause between steps
            time.sleep(2.0)
        
        self.get_logger().info("=== ADL Task: Make Cereal - COMPLETED ===")
        return True
    
    def execute_adl_task_organize_objects(self):
        """
        Execute ADL task: Organize objects on shelf
        """
        self.get_logger().info("=== Starting ADL Task: Organize Objects ===")
        
        # Define organization strategy: books to shelf, bottles to table
        organization_rules = {
            "book": "shelf_low",
            "water_bottle": "table_left", 
            "cup": "shelf_middle",
            "phone": "table_center",
        }
        
        # Process all detected objects
        organized_count = 0
        
        for obj in self.latest_detected_objects:
            if obj.object_type in organization_rules:
                target_location = organization_rules[obj.object_type]
                
                self.get_logger().info(f"Organizing {obj.object_type} to {target_location}")
                
                if self.execute_pick_and_place_task(obj.marker_id, target_location):
                    organized_count += 1
                    time.sleep(2.0)  # Pause between objects
                else:
                    self.get_logger().warn(f"Failed to organize {obj.object_type}")
        
        self.get_logger().info(f"=== Organization completed: {organized_count} objects organized ===")
        return organized_count > 0
    
    def demo_task_timer_callback(self):
        """Timer callback for demonstration tasks"""
        self.destroy_timer(self.demo_timer)  # Run only once
        
        # Wait a bit for objects to be detected
        time.sleep(2.0)
        
        if not self.latest_detected_objects:
            self.get_logger().info("No objects detected. Place ArUco markers on objects and try again.")
            return
        
        # Demo: Try to pickup the first detected object
        first_object = self.latest_detected_objects[0]
        self.get_logger().info(f"Demo: Attempting to pickup {first_object.object_type} (ID: {first_object.marker_id})")
        
        if self.pickup_object_by_id(first_object.marker_id, first_object.object_type):
            time.sleep(1.0)
            # Place it at a predefined location
            self.place_object_at_location("table_center")


def main(args=None):
    """Main function to start the ADL coordinator"""
    rclpy.init(args=args)
    
    try:
        node = ADLCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()