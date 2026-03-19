#!/usr/bin/env python3
"""
Mock GUI Node
Provides assembly plan service without requiring actual GUI
"""

import rclpy
from rclpy.node import Node
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.msg import SuperBrick
from geometry_msgs.msg import Pose, Point, Quaternion
import random


class MockGUINode(Node):
    """Mock GUI Service Provider"""
    
    def __init__(self):
        super().__init__('mock_gui_node')
        
        self.srv = self.create_service(
            GetAssemblyPlan,
            'get_assembly_plan',
            self.get_assembly_plan_callback
        )
        
        self.get_logger().info('Mock GUI Node started - serving /get_assembly_plan')
        self.plan_requested = False
    
    def _create_default_plan(self):
        """Create a targeted handover test plan"""
        plan = []
        
        # CREATE THE HANDOVER BRICK (ID: 99)
        handover_brick = SuperBrick()
        handover_brick.id = 99
        handover_brick.type = "brick_model_handover"
        
        # THIS TRIGGERS THE HANDOVER: Start at AR4, Target is ABB
        handover_brick.start_side = "AR4"
        handover_brick.target_side = "ABB" 
        
        # AR4 picks it up from its standard zone
        handover_brick.pickup_pose = Pose()
        handover_brick.pickup_pose.position = Point(x=0.65, y=0.10, z=0.05)
        handover_brick.pickup_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        
        # ABB places it in its standard assembly zone
        handover_brick.place_pose = Pose()
        handover_brick.place_pose.position = Point(x=0.50, y=-0.15, z=0.05)
        handover_brick.place_pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        
        plan.append(handover_brick)
        
        return plan
    def get_assembly_plan_callback(self, request, response):
        """
        Return assembly plan (can be parameterized)
        
        Returns:
            A list of SuperBrick objects representing the assembly sequence
        """
        self.get_logger().info('Received get_assembly_plan request')
        
        # Generate or return pre-configured plan
        plan = self._create_default_plan()
        
        response.plan = plan
        response.success = True
        
        self.get_logger().info(f'Returning assembly plan with {len(plan)} bricks')
        
        for i, brick in enumerate(plan):
            self.get_logger().info(
                f"  Brick {i}: {brick.type} - {brick.start_side} pickup -> {brick.target_side} placement"
            )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockGUINode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()