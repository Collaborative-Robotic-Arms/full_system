#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from supervisor_package.action import MoveToPose, AlignToTarget

class MockAR4(Node):
    def __init__(self):
        super().__init__('mock_ar4')
        self.point_server = ActionServer(self, MoveToPose, 'ar4_point_control', self.execute_point)
        self.vs_server = ActionServer(self, AlignToTarget, 'ar4_visual_servo', self.execute_vs)
        self.get_logger().info('Mock AR4: point_control and visual_servo action servers started')

    async def execute_point(self, goal_handle):
        self.get_logger().info(f"Mock AR4: MoveToPose received (strategy={goal_handle.request.strategy})")
        # Simulate approach/move time
        await asyncio.sleep(0.8)
        goal_handle.succeed()
        result = MoveToPose.Result()
        result.success = True
        return result

    async def execute_vs(self, goal_handle):
        self.get_logger().info(f"Mock AR4: AlignToTarget received (object_id={goal_handle.request.object_id})")
        # Simulate alignment iterations with feedback
        for i in range(3):
            await asyncio.sleep(0.3)
        goal_handle.succeed()
        result = AlignToTarget.Result()
        result.success = True
        return result


def main():
    rclpy.init()
    node = MockAR4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
