#!/usr/bin/env python3
"""
Mock GUI Node
Provides assembly plan service without requiring actual GUI
Supports 4 test scenarios:
1. Parallel execution - no collision
2. Parallel with collision risk
3. AR4 hands over to ABB
4. ABB hands over to AR4
"""

import rclpy
from rclpy.node import Node
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.msg import SuperBrick
from geometry_msgs.msg import Pose, Point, Quaternion
import os


class MockGUINode(Node):
    """Mock GUI Service Provider"""
    
    def __init__(self):
        super().__init__('mock_gui_node')
        
        self.srv = self.create_service(
            GetAssemblyPlan,
            'get_assembly_plan',
            self.get_assembly_plan_callback
        )
        
        # Get scenario from environment variable (default to scenario 1)
        self.scenario = int(os.getenv('TEST_SCENARIO', '1'))
        
        self.get_logger().info(f'Mock GUI Node started - Scenario {self.scenario} - serving /get_assembly_plan')
    
    def _scenario_1_parallel_no_collision(self):
        """Scenario 1: Parallel execution - no collision"""
        plan = []
        
        # Brick 1 for AR4
        b1 = SuperBrick()
        b1.id = 0
        b1.type = "brick_parallel_ar4"
        b1.start_side = "AR4"
        b1.target_side = "GRID"
        b1.pickup_pose = Pose(position=Point(x=0.65, y=0.15, z=0.05))
        b1.place_pose = Pose(position=Point(x=0.60, y=0.15, z=0.10))
        plan.append(b1)
        
        # Brick 2 for ABB (separate workspace)
        b2 = SuperBrick()
        b2.id = 1
        b2.type = "brick_parallel_abb"
        b2.start_side = "ABB"
        b2.target_side = "GRID"
        b2.pickup_pose = Pose(position=Point(x=0.40, y=-0.15, z=0.05))
        b2.place_pose = Pose(position=Point(x=0.40, y=-0.15, z=0.10))
        plan.append(b2)
        
        return plan
    
    def _scenario_2_parallel_with_collision(self):
        """Scenario 2: Parallel execution with collision risk"""
        plan = []
        
        # Brick 1 for AR4 - moves toward center
        b1 = SuperBrick()
        b1.id = 0
        b1.type = "brick_collision_ar4"
        b1.start_side = "AR4"
        b1.target_side = "GRID"
        b1.pickup_pose = Pose(position=Point(x=0.65, y=0.0, z=0.05))
        b1.place_pose = Pose(position=Point(x=0.50, y=0.0, z=0.10))  # Moves inward
        plan.append(b1)
        
        # Brick 2 for ABB - also moves toward center (COLLISION RISK!)
        b2 = SuperBrick()
        b2.id = 1
        b2.type = "brick_collision_abb"
        b2.start_side = "ABB"
        b2.target_side = "GRID"
        b2.pickup_pose = Pose(position=Point(x=0.45, y=0.0, z=0.05))
        b2.place_pose = Pose(position=Point(x=0.50, y=-0.05, z=0.10))  # Also moves inward
        plan.append(b2)
        
        return plan
    
    def _scenario_3_handover_ar4_to_abb(self):
        """Scenario 3: AR4 picks and hands over to ABB"""
        plan = []
        
        b1 = SuperBrick()
        b1.id = 0
        b1.type = "brick_handover_ar4_abb"
        b1.start_side = "AR4"      # AR4 picks
        b1.target_side = "ABB"      # ABB receives and places
        b1.pickup_pose = Pose(position=Point(x=0.65, y=0.10, z=0.05))
        b1.place_pose = Pose(position=Point(x=0.40, y=-0.10, z=0.10))  # Final placement (ABB places)
        plan.append(b1)
        
        return plan
    
    def _scenario_4_handover_abb_to_ar4(self):
        """Scenario 4: ABB picks and hands over to AR4"""
        plan = []
        
        b1 = SuperBrick()
        b1.id = 0
        b1.type = "brick_handover_abb_ar4"
        b1.start_side = "ABB"       # ABB picks
        b1.target_side = "AR4"      # AR4 receives and places
        b1.pickup_pose = Pose(position=Point(x=0.40, y=-0.10, z=0.05))
        b1.place_pose = Pose(position=Point(x=0.60, y=0.10, z=0.10))   # Final placement (AR4 places)
        plan.append(b1)
        
        return plan
    
    def get_assembly_plan_callback(self, request, response):
        """Return assembly plan based on scenario"""
        self.get_logger().info(f'[Scenario {self.scenario}] Received get_assembly_plan request')
        
        # Select plan based on scenario
        if self.scenario == 1:
            plan = self._scenario_1_parallel_no_collision()
            self.get_logger().info('✅ SCENARIO 1: Parallel execution - no collision')
        elif self.scenario == 2:
            plan = self._scenario_2_parallel_with_collision()
            self.get_logger().info('⚠️  SCENARIO 2: Parallel with collision risk')
        elif self.scenario == 3:
            plan = self._scenario_3_handover_ar4_to_abb()
            self.get_logger().info('🔄 SCENARIO 3: AR4 → ABB Handover')
        elif self.scenario == 4:
            plan = self._scenario_4_handover_abb_to_ar4()
            self.get_logger().info('🔄 SCENARIO 4: ABB → AR4 Handover')
        else:
            plan = self._scenario_1_parallel_no_collision()
            self.get_logger().warn(f'Unknown scenario {self.scenario}, using default')
        
        response.plan = plan
        response.success = True
        
        self.get_logger().info(f'Returning {len(plan)} brick(s)')
        for brick in plan:
            self.get_logger().info(
                f"  Brick {brick.id}: {brick.type} - {brick.start_side} pickup → {brick.target_side} placement"
            )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockGUINode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
