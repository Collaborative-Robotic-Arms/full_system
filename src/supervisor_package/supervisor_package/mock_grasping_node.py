#!/usr/bin/env python3
"""
Mock Grasping Pipeline Node
Provides grasp point service without requiring actual grasping model
Supports 4 test scenarios:
1. Parallel execution - no collision
2. Parallel with collision risk
3. AR4 hands over to ABB
4. ABB hands over to AR4
"""

import rclpy
from rclpy.node import Node
from dual_arms_msgs.srv import GetGrasp
from dual_arms_msgs.msg import GraspPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import random
import os


class MockGraspingNode(Node):
    """Mock Grasping Pipeline Service Provider"""
    
    def __init__(self):
        super().__init__('mock_grasping_node')
        
        self.srv = self.create_service(
            GetGrasp,
            'grasp/get_grasp_point',
            self.get_grasp_callback
        )
        
        # Get scenario from environment variable (default to scenario 1)
        self.scenario = int(os.getenv('TEST_SCENARIO', '1'))
        
        self.get_logger().info(f'Mock Grasping Pipeline Node started - Scenario {self.scenario} - serving /grasp/get_grasp_point')
    
    def _get_grasp_scenario_1(self, brick_id):
        """Scenario 1: Parallel execution - no collision"""
        grasp = GraspPoint()
        grasp.header = Header()
        grasp.header.frame_id = "abb_table"
        grasp.header.stamp = self.get_clock().now().to_msg()
        grasp.brick_id = brick_id
        grasp.pose = Pose()
        
        if brick_id == 0:
            # AR4 side - PROVEN WORKING COORDINATES
            grasp.pose.position = Point(x=0.65, y=0.10, z=0.05)
        else:
            # ABB side - PROVEN WORKING COORDINATES
            grasp.pose.position = Point(x=0.40, y=-0.10, z=0.05)
        
        grasp.quality = 0.9
        return grasp
    
    def _get_grasp_scenario_2(self, brick_id):
        """Scenario 2: Parallel with collision risk"""
        grasp = GraspPoint()
        grasp.header = Header()
        grasp.header.frame_id = "abb_table"
        grasp.header.stamp = self.get_clock().now().to_msg()
        grasp.brick_id = brick_id
        grasp.pose = Pose()
        
        if brick_id == 0:
            # AR4 side - slightly toward center
            grasp.pose.position = Point(x=0.60, y=0.05, z=0.05)
        else:
            # ABB side - moving toward center
            grasp.pose.position = Point(x=0.50, y=-0.05, z=0.05)
        
        grasp.quality = 0.85
        return grasp
    
    def _get_grasp_scenario_3(self, brick_id):
        """Scenario 3: AR4 hands over to ABB"""
        grasp = GraspPoint()
        grasp.header = Header()
        grasp.header.frame_id = "abb_table"
        grasp.header.stamp = self.get_clock().now().to_msg()
        grasp.brick_id = brick_id
        grasp.pose = Pose()
        
        if brick_id == 0:
            # Handover grasp point - ABB's approach
            grasp.pose.position = Point(x=0.55, y=0.02, z=0.25)
        
        grasp.quality = 0.88
        return grasp
    
    def _get_grasp_scenario_4(self, brick_id):
        """Scenario 4: ABB hands over to AR4"""
        grasp = GraspPoint()
        grasp.header = Header()
        grasp.header.frame_id = "abb_table"
        grasp.header.stamp = self.get_clock().now().to_msg()
        grasp.brick_id = brick_id
        grasp.pose = Pose()
        
        if brick_id == 0:
            # Handover grasp point - AR4's approach
            grasp.pose.position = Point(x=0.55, y=-0.02, z=0.25)
        
        grasp.quality = 0.87
        return grasp
    
    def get_grasp_callback(self, request, response):
        """
        Callback for grasp service request
        Returns grasp point based on scenario
        """
        try:
            brick_id = int(request.brick_index)
        except (ValueError, TypeError):
            brick_id = 0
        
        self.get_logger().info(f'[Scenario {self.scenario}] Received get_grasp request for brick {brick_id}')
        
        # Select grasp based on scenario
        if self.scenario == 1:
            grasp = self._get_grasp_scenario_1(brick_id)
            scenario_name = "Parallel (no collision)"
        elif self.scenario == 2:
            grasp = self._get_grasp_scenario_2(brick_id)
            scenario_name = "Parallel (collision risk)"
        elif self.scenario == 3:
            grasp = self._get_grasp_scenario_3(brick_id)
            scenario_name = "AR4 → ABB Handover"
        elif self.scenario == 4:
            grasp = self._get_grasp_scenario_4(brick_id)
            scenario_name = "ABB → AR4 Handover"
        else:
            grasp = self._get_grasp_scenario_1(brick_id)
            scenario_name = "Default"
        
        response.grasp_point = grasp
        response.success = True
        
        self.get_logger().info(
            f'[{scenario_name}] Grasping brick {brick_id}: quality={grasp.quality:.2f}, '
            f'pose=({grasp.pose.position.x:.3f}, {grasp.pose.position.y:.3f}, {grasp.pose.position.z:.3f})'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockGraspingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
