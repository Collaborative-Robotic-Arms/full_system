#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from supervisor_package.srv import GetAssemblyPlan, DetectBricks
from supervisor_package.action import MoveToPose, AlignToTarget, ExecuteTask
from supervisor_package.msg import Brick
from geometry_msgs.msg import Pose
from rclpy.action import ActionServer

class MockSystem(Node):
    def __init__(self):
        super().__init__('mock_system')
        self.plan_srv = self.create_service(GetAssemblyPlan, 'get_assembly_plan', self.plan_callback)
        self.detect_srv = self.create_service(DetectBricks, 'detect_bricks', self.detect_callback)
        self.get_logger().info("Mock System Ready - Sending real Brick objects now.")
        # AR4 Move Action
        self.ar4_move_server = ActionServer(
            self, MoveToPose, 'ar4_point_control', self.execute_move_callback)

        # AR4 VS Action
        self.ar4_vs_server = ActionServer(
            self, AlignToTarget, 'ar4_visual_servo', self.execute_vs_callback)

        # ABB Action
        self.abb_server = ActionServer(
            self, ExecuteTask, 'abb_control', self.execute_abb_callback)
        
    def plan_callback(self, request, response):
        self.get_logger().info("Providing Assembly Plan...")
        b1 = Brick()
        b1.id = 1
        b1.type = "red_brick"
        b1.start_side = "AR4"
        b1.target_side = "ABB"
        
        # This works now because the .srv expects Brick objects!
        response.plan = [b1] 
        response.success = True
        return response

    def detect_callback(self, request, response):
        self.get_logger().info("Providing Detected Bricks and Handover Pose...")
        
        # Existing brick logic
        b1 = Brick()
        b1.id = 1
        b1.pose.position.x = 0.5
        response.bricks = [b1]
        
        # New Handover Pose logic
        hp = Pose()
        hp.position.x = 0.3  # Example handover coordinates
        hp.position.y = 0.3
        hp.position.z = 0.2
        response.handover_pose = hp
        
        response.success = True
        return response

    async def execute_move_callback(self, goal_handle):
        self.get_logger().info(f'Executing Move Strategy: {goal_handle.request.strategy}')
        goal_handle.succeed()
        return MoveToPose.Result()

    async def execute_vs_callback(self, goal_handle):
        self.get_logger().info('Executing Visual Servoing...')
        goal_handle.succeed()
        return AlignToTarget.Result()

    async def execute_abb_callback(self, goal_handle):
        self.get_logger().info(f'Executing ABB Task: {goal_handle.request.task_type}')
        goal_handle.succeed()
        return ExecuteTask.Result()


def main():
    rclpy.init()
    node = MockSystem()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()