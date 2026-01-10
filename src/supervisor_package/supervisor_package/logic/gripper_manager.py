#!/usr/bin/env python3
import time
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperManager:
    """
    Helper class to manage the "Grasp-then-Lock" sequence for AR4 and ABB robots.
    """
    def __init__(self, node: Node):
        """
        Requires the parent node to create publishers.
        """
        self.node = node
        
        # --- AR4 SETUP ---
        # 1. Physical Controller (Moves the visual mesh)
        self.ar4_traj_pub = node.create_publisher(
            JointTrajectory, 
            '/ar4_gripper_controller/joint_trajectory', 
            10
        )
        # 2. Lock Plugin (The "Sticky" logic)
        self.ar4_lock_pub = node.create_publisher(
            Bool, 
            '/ar4/gripper/lock_trigger', 
            10
        )

        # --- ABB SETUP ---
        self.abb_traj_pub = node.create_publisher(
            JointTrajectory, 
            '/irb120_gripper_controller/joint_trajectory', 
            10
        )
        self.abb_lock_pub = node.create_publisher(
            Bool, 
            '/abb/gripper/lock_trigger', 
            10
        )

    def set_ar4_grip(self, state: str):
        """
        state: 'OPEN' or 'CLOSE'
        """
        traj = JointTrajectory()
        # FIXED: Must send BOTH joint names for the controller to accept it
        traj.joint_names = ['ar4_gripper_jaw1_joint', 'ar4_gripper_jaw2_joint']
        point = JointTrajectoryPoint()
        
        if state == 'CLOSE':
            # Fully Closed (0.0)
            point.positions = [0.0, 0.0]  
            should_lock = True
        else:
            # Open (Limit is 0.015 based on URDF)
            point.positions = [0.015, 0.015] 
            should_lock = False
            
        point.time_from_start.sec = 1
        traj.points = [point]
        
        self.node.get_logger().info(f"[Gripper] AR4 Moving to {state}...")
        self.ar4_traj_pub.publish(traj)
        
        # Wait for physical move before locking (Syncs visual with physics)
        # Note: This pauses the supervisor logic for 1.2 seconds
        time.sleep(1.2) 
        
        # Trigger the lock plugin
        self._trigger_lock(self.ar4_lock_pub, should_lock)

    def set_abb_grip(self, state: str):
        """
        state: 'OPEN' or 'CLOSE'
        """
        traj = JointTrajectory()
        # FIXED: Must send BOTH joint names
        traj.joint_names = ['gripper_ABB_Gripper_Finger_1_Joint', 'gripper_ABB_Gripper_Finger_2_Joint']
        point = JointTrajectoryPoint()
        
        if state == 'CLOSE':
            point.positions = [0.0, 0.0] 
            should_lock = True
        else:
            # Open (Limit is ~0.0135 based on URDF)
            point.positions = [0.0135, 0.0135]
            should_lock = False

        point.time_from_start.sec = 1
        traj.points = [point]
        
        self.node.get_logger().info(f"[Gripper] ABB Moving to {state}...")
        self.abb_traj_pub.publish(traj)
        
        time.sleep(1.2)
        
        self._trigger_lock(self.abb_lock_pub, should_lock)

    def _trigger_lock(self, publisher, state: bool):
        msg = Bool()
        msg.data = state
        publisher.publish(msg)