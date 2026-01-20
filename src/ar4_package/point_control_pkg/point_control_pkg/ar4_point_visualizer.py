#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs.tf2_geometry_msgs as tf2_gm

class AR4Visualizer(Node):
    def __init__(self):
        super().__init__('ar4_point_visualizer')

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(MarkerArray, 'ar4_visualizer_markers', 10)

        # TF buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Keep a counter for marker IDs
        self.marker_id = 0

        # Example: You can change/add points dynamically
        self.points_abb_frame = [
            (0.7, 0.0, 0.2),
            (0.5, 0.3, 0.4),
            (0.3, -0.2, 0.1)
        ]

        # Timer to repeatedly update markers
        self.create_timer(1.0, self.publish_points)

    def publish_points(self):
        marker_array = MarkerArray()

        for point in self.points_abb_frame:
            x_abb, y_abb, z_abb = point

            # Create a PointStamped in base_link frame
            point_msg = PointStamped()
            point_msg.header.frame_id = "base_link"
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.point.x = x_abb
            point_msg.point.y = y_abb
            point_msg.point.z = z_abb

            try:
                # Transform to AR4 frame
                point_ar4 = self.tf_buffer.transform(point_msg, "ar4_base_link")
                x_ar4 = point_ar4.point.x
                y_ar4 = point_ar4.point.y
                z_ar4 = point_ar4.point.z

                self.get_logger().info(f"Point ABB({x_abb},{y_abb},{z_abb}) -> AR4({x_ar4:.3f},{y_ar4:.3f},{z_ar4:.3f})")

                # Create a marker for this point
                marker = Marker()
                marker.header.frame_id = "ar4_base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "points"
                marker.id = self.marker_id
                self.marker_id += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x_ar4
                marker.pose.position.y = y_ar4
                marker.pose.position.z = z_ar4
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                marker_array.markers.append(marker)

            except Exception as e:
                self.get_logger().warn(f"TF transform failed: {e}")

        # Publish all markers at once
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            # Reset marker ID if too high
            if self.marker_id > 1000:
                self.marker_id = 0


def main(args=None):
    rclpy.init(args=args)
    node = AR4Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
