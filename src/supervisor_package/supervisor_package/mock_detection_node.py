#!/usr/bin/env python3
"""
Mock Detection Node
Provides brick detection service without requiring actual vision system
Supports 4 test scenarios:
1. Parallel execution - no collision
2. Parallel with collision risk
3. AR4 hands over to ABB
4. ABB hands over to AR4
"""

import rclpy
from rclpy.node import Node
from dual_arms_msgs.srv import DetectBricks
from dual_arms_msgs.msg import Brick
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import os


class MockDetectionNode(Node):
    """Mock Detection Service Provider"""
    
    def __init__(self):
        super().__init__('mock_detection_node')
        
        self.srv = self.create_service(
            DetectBricks,
            'detect_bricks',
            self.detect_bricks_callback
        )
        
        # Get scenario from environment variable (default to scenario 1)
        self.scenario = int(os.getenv('TEST_SCENARIO', '1'))
        
        self.get_logger().info(f'Mock Detection Node started - Scenario {self.scenario} - serving /detect_bricks')
    
    def _scenario_1_detections(self):
        """Scenario 1: Parallel execution - no collision"""
        bricks = []
        
        # Brick 0 (AR4 side) - PROVEN WORKING COORDINATES
        b1 = Brick()
        b1.header = Header()
        b1.header.stamp = self.get_clock().now().to_msg()
        b1.header.frame_id = "abb_table"
        b1.id = 0
        b1.side = 1  # AR4
        b1.pose = Pose(position=Point(x=0.65, y=0.10, z=0.05))
        b1.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        bricks.append(b1)
        
        # Brick 1 (ABB side) - PROVEN WORKING COORDINATES
        b2 = Brick()
        b2.header = Header()
        b2.header.stamp = self.get_clock().now().to_msg()
        b2.header.frame_id = "abb_table"
        b2.id = 1
        b2.side = 0  # ABB
        b2.pose = Pose(position=Point(x=0.40, y=-0.10, z=0.05))
        b2.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        bricks.append(b2)
        
        handover = Pose(position=Point(x=0.55, y=0.0, z=0.35))
        return bricks, handover
    
    def _scenario_2_detections(self):
        """Scenario 2: Parallel with collision risk"""
        bricks = []
        
        # Brick 0 (AR4, slightly toward center)
        b1 = Brick()
        b1.header = Header()
        b1.header.stamp = self.get_clock().now().to_msg()
        b1.header.frame_id = "abb_table"
        b1.id = 0
        b1.side = 1  # AR4
        b1.pose = Pose(position=Point(x=0.60, y=0.05, z=0.05))
        b1.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        bricks.append(b1)
        
        # Brick 1 (ABB, also moving toward center - COLLISION RISK)
        b2 = Brick()
        b2.header = Header()
        b2.header.stamp = self.get_clock().now().to_msg()
        b2.header.frame_id = "abb_table"
        b2.id = 1
        b2.side = 0  # ABB
        b2.pose = Pose(position=Point(x=0.50, y=-0.05, z=0.05))
        b2.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        bricks.append(b2)
        
        # Collision zone in the middle
        handover = Pose(position=Point(x=0.50, y=0.0, z=0.35))
        return bricks, handover
    
    def _scenario_3_detections(self):
        """Scenario 3: AR4 hands over to ABB"""
        bricks = []
        
        # Single brick for AR4 to pick (AR4 proven coordinates)
        b1 = Brick()
        b1.header = Header()
        b1.header.stamp = self.get_clock().now().to_msg()
        b1.header.frame_id = "abb_table"
        b1.id = 0
        b1.side = 1  # AR4 picks
        b1.pose = Pose(position=Point(x=0.65, y=0.10, z=0.05))
        b1.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        bricks.append(b1)
        
        # Handover zone between the arms
        handover = Pose(position=Point(x=0.55, y=0.0, z=0.35))
        return bricks, handover
    
    def _scenario_4_detections(self):
        """Scenario 4: ABB hands over to AR4"""
        bricks = []
        
        # Single brick for ABB to pick (ABB proven coordinates)
        b1 = Brick()
        b1.header = Header()
        b1.header.stamp = self.get_clock().now().to_msg()
        b1.header.frame_id = "abb_table"
        b1.id = 0
        b1.side = 0  # ABB picks
        b1.pose = Pose(position=Point(x=0.40, y=-0.10, z=0.05))
        b1.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        bricks.append(b1)
        
        # Handover zone between the arms
        handover = Pose(position=Point(x=0.55, y=0.0, z=0.35))
        return bricks, handover
    
    def detect_bricks_callback(self, request, response):
        """Generate mock brick detections based on scenario"""
        self.get_logger().info(f'[Scenario {self.scenario}] Received detect_bricks request')
        
        # Select detections based on scenario
        if self.scenario == 1:
            bricks, handover = self._scenario_1_detections()
            self.get_logger().info('✅ SCENARIO 1: Detecting 2 bricks (AR4 and ABB - separate workspace)')
        elif self.scenario == 2:
            bricks, handover = self._scenario_2_detections()
            self.get_logger().info('⚠️  SCENARIO 2: Detecting 2 bricks (moving toward collision zone)')
        elif self.scenario == 3:
            bricks, handover = self._scenario_3_detections()
            self.get_logger().info('🔄 SCENARIO 3: Detecting 1 brick (AR4 → ABB handover)')
        elif self.scenario == 4:
            bricks, handover = self._scenario_4_detections()
            self.get_logger().info('🔄 SCENARIO 4: Detecting 1 brick (ABB → AR4 handover)')
        else:
            bricks, handover = self._scenario_1_detections()
            self.get_logger().warn(f'Unknown scenario {self.scenario}, using default')
        
        response.bricks = bricks
        response.handover_pose = handover
        response.success = True
        
        self.get_logger().info(f'Returning {len(bricks)} detected bricks')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
