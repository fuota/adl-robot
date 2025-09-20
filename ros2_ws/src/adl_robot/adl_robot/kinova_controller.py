#!/usr/bin/env python3
"""
Kinova Gen3 Controller for ADL Robot

This node provides control interface for the Kinova Gen3 7-DoF robotic arm
including pick and place operations for ADL tasks.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
import time
import math

class KinovaController(Node):
    """
    ROS2 Node for controlling Kinova Gen3 7-DoF robotic arm
    """
    
    def __init__(self):
        super().__init__('kinova_controller')
        
        # Initialize parameters
        self.declare_parameter('arm_name', 'my_gen3')
        self.declare_parameter('gripper_joint_name', 'gripper_finger1_joint')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'end_effector_link')
        
        # Get parameters
        self.arm_name = self.get_parameter('arm_name').get_parameter_value().string_value
        self.gripper_joint = self.get_parameter('gripper_joint_name').get_parameter_value().string_value
        self.planning_frame = self.get_parameter('planning_frame').get_parameter_value().string_value
        self.end_effector_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        
        # Joint names for Kinova Gen3 7-DoF
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4',
            'joint_5', 'joint_6', 'joint_7'
        ]
        
        # Gripper joint names
        self.gripper_joint_names = [
            'gripper_finger1_joint',
            'gripper_finger2_joint'
        ]
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Action clients for arm and gripper control
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, f'/{self.arm_name}/joint_trajectory_controller/follow_joint_trajectory')
        
        self.gripper_action_client = ActionClient(
            self, FollowJointTrajectory, f'/{self.arm_name}/gripper_controller/follow_joint_trajectory')
        
        # Wait for action servers
        self.get_logger().info("Waiting for arm action server...")
        self.arm_action_client.wait_for_server()
        
        self.get_logger().info("Waiting for gripper action server...")
        self.gripper_action_client.wait_for_server()
        
        # Pre-defined poses
        self.home_position = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]  # Home pose
        self.pre_grasp_offset = [0.0, 0.0, 0.15]  # 15cm above target for pre-grasp
        
        # Current joint state (would be updated from joint_states subscriber)
        self.current_joint_positions = [0.0] * 7
        
        self.get_logger().info("Kinova Controller initialized")
    
    def move_to_joint_positions(self, joint_positions, duration=5.0):
        """
        Move arm to specified joint positions
        
        Args:
            joint_positions: List of 7 joint angles (radians)
            duration: Trajectory execution time (seconds)
            
        Returns:
            bool: Success status
        """
        if len(joint_positions) != 7:
            self.get_logger().error("Invalid number of joint positions. Expected 7.")
            return False
        
        # Create trajectory message
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * 7
        point.accelerations = [0.0] * 7
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.header.frame_id = self.planning_frame
        
        # Send goal and wait for result
        self.get_logger().info("Sending arm trajectory goal...")
        future = self.arm_action_client.send_goal_async(goal)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=duration + 2.0)
            if future.result() is not None:
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.get_logger().info("Arm trajectory goal accepted")
                    
                    # Wait for execution to complete
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
                    
                    if result_future.result() is not None:
                        result = result_future.result().result
                        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                            self.get_logger().info("Arm movement successful")
                            self.current_joint_positions = joint_positions
                            return True
                        else:
                            self.get_logger().error(f"Arm movement failed with error code: {result.error_code}")
                    else:
                        self.get_logger().error("Failed to get arm trajectory result")
                else:
                    self.get_logger().error("Arm trajectory goal rejected")
            else:
                self.get_logger().error("Failed to send arm trajectory goal")
                
        except Exception as e:
            self.get_logger().error(f"Error during arm movement: {str(e)}")
        
        return False
    
    def control_gripper(self, open_close_value, duration=2.0):
        """
        Control gripper opening/closing
        
        Args:
            open_close_value: 0.0 = fully closed, 1.0 = fully open
            duration: Trajectory execution time (seconds)
            
        Returns:
            bool: Success status
        """
        # Kinova gripper values (adjust based on your specific gripper)
        max_gripper_value = 0.8  # Max opening value for gripper fingers
        gripper_positions = [open_close_value * max_gripper_value] * 2
        
        # Create trajectory message
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = gripper_positions
        point.velocities = [0.0] * 2
        point.accelerations = [0.0] * 2
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.header.frame_id = self.planning_frame
        
        # Send goal and wait for result
        action_name = "open" if open_close_value > 0.5 else "close"
        self.get_logger().info(f"Sending gripper {action_name} goal...")
        future = self.gripper_action_client.send_goal_async(goal)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=duration + 2.0)
            if future.result() is not None:
                goal_handle = future.result()
                if goal_handle.accepted:
                    # Wait for execution to complete
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
                    
                    if result_future.result() is not None:
                        result = result_future.result().result
                        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                            self.get_logger().info(f"Gripper {action_name} successful")
                            return True
                        else:
                            self.get_logger().error(f"Gripper {action_name} failed with error code: {result.error_code}")
                    else:
                        self.get_logger().error(f"Failed to get gripper {action_name} result")
                else:
                    self.get_logger().error(f"Gripper {action_name} goal rejected")
            else:
                self.get_logger().error(f"Failed to send gripper {action_name} goal")
                
        except Exception as e:
            self.get_logger().error(f"Error during gripper {action_name}: {str(e)}")
        
        return False
    
    def move_to_home(self):
        """
        Move arm to home position
        
        Returns:
            bool: Success status
        """
        self.get_logger().info("Moving to home position...")
        return self.move_to_joint_positions(self.home_position, duration=6.0)
    
    def open_gripper(self):
        """
        Open gripper fully
        
        Returns:
            bool: Success status
        """
        return self.control_gripper(1.0)  # Fully open
    
    def close_gripper(self):
        """
        Close gripper to grasp object
        
        Returns:
            bool: Success status
        """
        return self.control_gripper(0.1)  # Mostly closed for grasping
    
    def cartesian_to_joint_positions(self, target_pose):
        """
        Convert Cartesian pose to joint positions using inverse kinematics
        
        Note: This is a placeholder. In a real implementation, you would use
        MoveIt! or the Kinova SDK for inverse kinematics.
        
        Args:
            target_pose: PoseStamped message
            
        Returns:
            list: Joint positions (7 values) or None if IK fails
        """
        # Placeholder implementation - in reality, use MoveIt! or Kinova IK
        self.get_logger().warn("Using placeholder IK. Implement proper inverse kinematics!")
        
        # Simple example: move to a position relative to current position
        # This would need to be replaced with proper IK solver
        x = target_pose.pose.position.x
        y = target_pose.pose.position.y
        z = target_pose.pose.position.z
        
        # Very basic approximation - NOT suitable for real use
        joint_positions = [
            math.atan2(y, x),  # Base rotation
            -0.5,              # Shoulder
            0.0,               # Elbow
            -1.57,             # Wrist 1
            0.0,               # Wrist 2
            1.57,              # Wrist 3
            0.0                # Wrist rotate
        ]
        
        return joint_positions
    
    def execute_pick_sequence(self, target_pose):
        """
        Execute complete pick sequence: approach, grasp, lift
        
        Args:
            target_pose: PoseStamped message for object location
            
        Returns:
            bool: Success status
        """
        try:
            # Step 1: Open gripper
            self.get_logger().info("Step 1: Opening gripper...")
            if not self.open_gripper():
                return False
            
            # Step 2: Move to pre-grasp position (above target)
            pre_grasp_pose = PoseStamped()
            pre_grasp_pose.header = target_pose.header
            pre_grasp_pose.pose = target_pose.pose
            pre_grasp_pose.pose.position.z += self.pre_grasp_offset[2]
            
            self.get_logger().info("Step 2: Moving to pre-grasp position...")
            pre_grasp_joints = self.cartesian_to_joint_positions(pre_grasp_pose)
            if pre_grasp_joints is None:
                self.get_logger().error("Failed to compute pre-grasp joint positions")
                return False
            
            if not self.move_to_joint_positions(pre_grasp_joints, duration=4.0):
                return False
            
            # Step 3: Move to grasp position
            self.get_logger().info("Step 3: Moving to grasp position...")
            grasp_joints = self.cartesian_to_joint_positions(target_pose)
            if grasp_joints is None:
                self.get_logger().error("Failed to compute grasp joint positions")
                return False
            
            if not self.move_to_joint_positions(grasp_joints, duration=3.0):
                return False
            
            # Step 4: Close gripper to grasp object
            self.get_logger().info("Step 4: Grasping object...")
            time.sleep(0.5)  # Brief pause before grasping
            if not self.close_gripper():
                return False
            
            # Step 5: Lift object
            self.get_logger().info("Step 5: Lifting object...")
            lift_pose = PoseStamped()
            lift_pose.header = target_pose.header
            lift_pose.pose = target_pose.pose
            lift_pose.pose.position.z += 0.10  # Lift 10cm
            
            lift_joints = self.cartesian_to_joint_positions(lift_pose)
            if lift_joints is None:
                self.get_logger().error("Failed to compute lift joint positions")
                return False
            
            if not self.move_to_joint_positions(lift_joints, duration=3.0):
                return False
            
            self.get_logger().info("Pick sequence completed successfully!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error during pick sequence: {str(e)}")
            return False
    
    def execute_place_sequence(self, target_pose):
        """
        Execute complete place sequence: approach, place, release, retract
        
        Args:
            target_pose: PoseStamped message for placement location
            
        Returns:
            bool: Success status
        """
        try:
            # Step 1: Move to pre-place position (above target)
            pre_place_pose = PoseStamped()
            pre_place_pose.header = target_pose.header
            pre_place_pose.pose = target_pose.pose
            pre_place_pose.pose.position.z += self.pre_grasp_offset[2]
            
            self.get_logger().info("Step 1: Moving to pre-place position...")
            pre_place_joints = self.cartesian_to_joint_positions(pre_place_pose)
            if pre_place_joints is None:
                self.get_logger().error("Failed to compute pre-place joint positions")
                return False
            
            if not self.move_to_joint_positions(pre_place_joints, duration=4.0):
                return False
            
            # Step 2: Move to place position
            self.get_logger().info("Step 2: Moving to place position...")
            place_joints = self.cartesian_to_joint_positions(target_pose)
            if place_joints is None:
                self.get_logger().error("Failed to compute place joint positions")
                return False
            
            if not self.move_to_joint_positions(place_joints, duration=3.0):
                return False
            
            # Step 3: Release object
            self.get_logger().info("Step 3: Releasing object...")
            time.sleep(0.5)  # Brief pause before releasing
            if not self.open_gripper():
                return False
            
            # Step 4: Retract
            self.get_logger().info("Step 4: Retracting...")
            if not self.move_to_joint_positions(pre_place_joints, duration=3.0):
                return False
            
            self.get_logger().info("Place sequence completed successfully!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error during place sequence: {str(e)}")
            return False


def main(args=None):
    """Main function to start the Kinova controller node"""
    rclpy.init(args=args)
    
    try:
        node = KinovaController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()