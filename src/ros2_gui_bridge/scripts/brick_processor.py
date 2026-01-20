#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_gui_bridge.msg import Brick, BrickArray
from geometry_msgs.msg import Point
import json

# Configuration Constants
CELL_SIZE = 0.03  # 5cm per cell
Z_HEIGHT = 0.23   # Default placement height

# World Frame Offsets
WORLD_X_OFFSET = 0.51
WORLD_Y_OFFSET = -0.12

class BrickProcessor(Node):
    def __init__(self):
        super().__init__('brick_processor')
        # Subscribes to raw JSON strings from the GUI bridge [cite: 1]
        self.sub = self.create_subscription(String, '/incoming_bricks', self.listener_callback, 10)
        # Publishes structured BrickArray to the rest of the system [cite: 1, 3]
        self.pub = self.create_publisher(BrickArray, '/processed_bricks', 10)
        self.get_logger().info('--- BRICK PROCESSOR READY ---')

    def calculate_world_coords(self, row, col):
        """
        Transforms grid indices to world coordinates.
        - Grid (0,0) maps to World (60, 50).
        - Increase in Cols -> Decrease in X.
        - Increase in Rows -> Increase in Y.
        - References the CENTER of the cell.
        """
        center_offset = CELL_SIZE / 2.0
        # Formula: Origin - (index * size) - half-cell to reach center
        world_x = WORLD_X_OFFSET - (col * CELL_SIZE) - center_offset
        # Formula: Origin + (index * size) + half-cell to reach center
        world_y = WORLD_Y_OFFSET + (row * CELL_SIZE) + center_offset
        return world_x, world_y

    def listener_callback(self, msg):
        try:
            full_data = json.loads(msg.data)
            raw_shapes = full_data.get('shapes', [])

            if isinstance(raw_shapes, dict):
                raw_shapes = list(raw_shapes.values())

            output_msg = BrickArray()

            for shape in raw_shapes:
                brick = Brick()
                
                # Metadata extraction from JSON [cite: 1]
                center_data = shape.get('centerCell', {})
                pos_data = shape.get('position', {})
                
                brick.id = str(shape.get('id') or center_data.get('id') or 'unknown')
                brick.type = str(shape.get('type') or pos_data.get('type') or 'unknown')
                brick.layer = int(shape.get('layer') or 1)
                brick.color = str(shape.get('color') or 'red')

                # Grid-to-World Transformation
                r_val = int(center_data.get('row') or pos_data.get('row') or 0)
                c_val = int(center_data.get('col') or pos_data.get('col') or 0)
                world_x, world_y = self.calculate_world_coords(r_val, c_val)
                
                # --- CRITICAL FIX: Explicitly Instantiate Point Objects ---
                # You cannot assign x, y, z to a field that hasn't been initialized as a Point()
                brick.place_pose = Point() 
                brick.place_pose.x = float(world_x)
                brick.place_pose.y = float(world_y)
                brick.place_pose.z = float(Z_HEIGHT)

                # Initialize pickup_pose for later use by detection 
                brick.pickup_pose = Point()
                brick.pickup_pose.x = 0.0
                brick.pickup_pose.y = 0.0
                brick.pickup_pose.z = 0.0

                # Orientation and Location 
                raw_or = str(shape.get('orientation', 'default')).lower()
                brick.orientation = 1 if ("vertical" in raw_or or "rotated" in raw_or) else 0
                brick.location = 0

                output_msg.bricks.append(brick)

            # Sort and publish the results [cite: 3]
            output_msg.bricks.sort(key=lambda b: b.layer)
            self.pub.publish(output_msg)
            self.get_logger().info(f'Published {len(output_msg.bricks)}.')

        except Exception as e:
            # Detailed error logging to catch any remaining attribute issues
            self.get_logger().error(f"Processing Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = BrickProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()