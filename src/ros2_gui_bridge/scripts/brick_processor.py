#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_gui_bridge.msg import Brick, BrickArray, GridCell
import json

class BrickProcessor(Node):
    def __init__(self):
        super().__init__('brick_processor')
        self.sub = self.create_subscription(String, '/incoming_bricks', self.listener_callback, 10)
        self.pub = self.create_publisher(BrickArray, '/processed_bricks', 10)
        self.get_logger().info('--- BRICK PROCESSOR (AUTO-DETECT MODE) READY ---')

    def listener_callback(self, msg):
        try:
            full_data = json.loads(msg.data)
            raw_shapes = full_data.get('shapes', [])

            # --- FIX 1: Handle Map vs List ---
            # If Firestore sent a Dictionary {"0": {...}, "1": {...}}, convert it to a List
            if isinstance(raw_shapes, dict):
                self.get_logger().warn(f"Detected 'shapes' as Dictionary. Converting to List...")
                raw_shapes = list(raw_shapes.values())

            output_msg = BrickArray()

            # Orientation Map
            ORIENTATION_MAP = {
                "default": 0, "horizontal": 0, "rotated": 90, "vertical": 90,
                "inverted": 180, "flipped": 270, "upside_down": 180, "left": 270
            }

            for i, shape in enumerate(raw_shapes):
                # --- DEBUG: Print the first shape's keys to terminal ---
                if i == 0:
                    print(f"\n[DEBUG] Raw Keys found in shape 0: {list(shape.keys())}")
                    if 'centerCell' in shape:
                        print(f"[DEBUG] Inside centerCell: {list(shape['centerCell'].keys())}")
                    if 'position' in shape:
                        print(f"[DEBUG] Inside position: {list(shape['position'].keys())}")

                brick = Brick()
                
                # Helper to get sub-dictionaries safely
                center_data = shape.get('centerCell', {})
                pos_data = shape.get('position', {})
                
                # --- FIX 2: Check EVERYWHERE for ID and Type ---
                
                # ID Search Order: Root -> centerCell -> position
                brick.id = str(shape.get('id') or center_data.get('id') or pos_data.get('id') or 'unknown')
                
                # Type Search Order: Root -> position -> centerCell
                brick.type = str(shape.get('type') or pos_data.get('type') or center_data.get('type') or 'unknown')
                
                # Color Search Order: Root -> centerCell
                brick.color = str(shape.get('color') or center_data.get('color') or 'unknown')
                
                # Layer
                brick.layer = int(shape.get('layer') or center_data.get('layer') or 1)

                # Orientation
                raw_or = str(shape.get('orientation', 'default'))
                if raw_or.replace('-','').isdigit():
                    brick.orientation = int(raw_or)
                else:
                    brick.orientation = ORIENTATION_MAP.get(raw_or, 0)

                # --- Grid Coordinates (1-Based) ---
                # Check centerCell, then position, then root
                r_val = center_data.get('row') or pos_data.get('row') or shape.get('row') or 0
                c_val = center_data.get('col') or pos_data.get('col') or shape.get('col') or 0
                
                brick.center_cell = GridCell()
                brick.center_cell.row = int(r_val) + 1
                brick.center_cell.col = int(c_val) + 1

                # --- Occupied Cells ---
                occupied_list = shape.get('occupiedCells', shape.get('occupied_cells', []))
                for cell_data in occupied_list:
                    new_cell = GridCell()
                    new_cell.row = int(cell_data.get('row', 0)) + 1
                    new_cell.col = int(cell_data.get('col', 0)) + 1
                    brick.occupied_cells.append(new_cell)
                
                if not brick.occupied_cells:
                    brick.occupied_cells.append(brick.center_cell)

                output_msg.bricks.append(brick)

            output_msg.bricks.sort(key=lambda b: b.layer)
            self.pub.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")

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
