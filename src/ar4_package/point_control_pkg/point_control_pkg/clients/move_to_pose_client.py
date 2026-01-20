#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Quaternion
from point_control_pkg.action import MoveToPose
import sys
import time

class MoveToPoseClient(Node):
    def __init__(self):
        super().__init__('move_to_pose_client')
        self.get_logger().info('Initializing MoveToPoseClient...')
        self._client = ActionClient(self, MoveToPose, 'ar4_move_to_pose')

    def send_goal(self, x, y, z):
        goal_msg = MoveToPose.Goal()
        goal_msg.target_pose = Pose()
        goal_msg.target_pose.position.x = x
        goal_msg.target_pose.position.y = y
        goal_msg.target_pose.position.z = z
        goal_msg.target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info(f'[CLIENT] Preparing to send goal: x={x}, y={y}, z={z}')

        self.get_logger().info('[CLIENT] Waiting for action server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('[CLIENT] Action server not available after 10 seconds.')
            rclpy.shutdown()
            return
        self.get_logger().info('[CLIENT] Action server is available.')

        self.get_logger().info('[CLIENT] Sending goal...')
        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.get_logger().info('[CLIENT] Goal response received.')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[CLIENT] Goal rejected by server.')
            rclpy.shutdown()
            return

        self.get_logger().info('[CLIENT] Goal accepted by server, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('[CLIENT] Result callback triggered.')
        result = future.result().result
        self.get_logger().info(f'[CLIENT] Result received: success={result.success}, message="{result.message}"')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[CLIENT] Feedback received: progress={feedback.progress}%')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 4:
        print("Usage: python3 move_to_pose_client_cmd.py <x> <y> <z>")
        rclpy.shutdown()
        return

    x, y, z = map(float, sys.argv[1:4])
    client = MoveToPoseClient()
    client.get_logger().info('[MAIN] Sending goal to action server...')
    client.send_goal(x, y, z)

    try:
        client.get_logger().info('[MAIN] Spinning node...')
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().warn('[MAIN] KeyboardInterrupt, shutting down.')
    finally:
        client.get_logger().info('[MAIN] Destroying node and shutting down ROS2.')
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
