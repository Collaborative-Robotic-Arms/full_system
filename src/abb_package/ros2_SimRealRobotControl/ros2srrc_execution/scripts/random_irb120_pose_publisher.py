#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random
import math
import numpy as np

class RandomPosePublisher(Node):
    def __init__(self):
        super().__init__('random_pose_publisher')
        
        # Publisher for target poses
        self.publisher = self.create_publisher(PoseStamped, '/target_pose_irb120', 10)
        
        # Timer to publish at 1 Hz (adjustable)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_random_pose)
        
        # Workspace limits (adjust as needed)
        self.x_min, self.x_max = 0.3, 0.7    # Forward/backward
        self.y_min, self.y_max = -0.5, 0.5    # Left/right
        self.z_min, self.z_max = 0.1, 0.6     # Up/down
        
        self.get_logger().info("Random Pose Publisher Started!")

    def publish_random_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'

        # Random position within workspace
        pose_msg.pose.position.x = random.uniform(self.x_min, self.x_max)
        pose_msg.pose.position.y = random.uniform(self.y_min, self.y_max)
        pose_msg.pose.position.z = random.uniform(self.z_min, self.z_max)

        # Random orientation (quaternion)
        # Method 1: Random quaternion (normalized)
        q = self.random_quaternion()
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.publisher.publish(pose_msg)
        self.get_logger().info(f"Published Random Pose: {pose_msg.pose.position}")

    def random_quaternion(self):
        """Generate a random normalized quaternion."""
        u1, u2, u3 = random.random(), random.random(), random.random()
        w = math.sqrt(1 - u1) * math.sin(2 * math.pi * u2)
        x = math.sqrt(1 - u1) * math.cos(2 * math.pi * u2)
        y = math.sqrt(u1) * math.sin(2 * math.pi * u3)
        z = math.sqrt(u1) * math.cos(2 * math.pi * u3)
        return (x, y, z, w)

def main(args=None):
    rclpy.init(args=args)
    node = RandomPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
