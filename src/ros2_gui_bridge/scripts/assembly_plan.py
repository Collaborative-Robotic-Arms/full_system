#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# --- IMPORT MESSAGES ---
# 1. GUI Messages (The "Plan")
from ros2_gui_bridge.msg import BrickArray as GuiArray
from ros2_gui_bridge.msg import Brick as GuiBrick

# 2. Detection Messages (The "Reality")
from dual_arms_msgs.msg import BricksArray as CamArray
from dual_arms_msgs.msg import Brick as CamBrick 

class AssemblySupervisor(Node):
    def __init__(self):
        super().__init__('assembly_supervisor')
        
        # Inputs
        self.gui_sub = self.create_subscription(
            GuiArray, '/processed_bricks', self.gui_callback, 10)
        self.cam_sub = self.create_subscription(
            CamArray, '/detected_bricks', self.cam_callback, 10)
        
        # Outputs
        self.plan_pub = self.create_publisher(GuiArray, '/robot/execution_plan', 10)
        self.flag_pub = self.create_publisher(Bool, '/supervisor/assembly_ready', 10)
        
        self.required_assembly = []
        self.available_supply = []
        
        self.timer = self.create_timer(0.5, self.validate_and_assign)
        self.get_logger().info('--- ASSEMBLY SUPERVISOR (LOCATION SPLIT ENABLED) READY ---')

    def gui_callback(self, msg):
        self.required_assembly = msg.bricks

    def cam_callback(self, msg):
        self.available_supply = msg.bricks

    def validate_and_assign(self):
        if not self.required_assembly:
            return

        # Track available supply using indices to avoid double-counting
        supply_indices = list(range(len(self.available_supply)))
        
        final_plan = GuiArray()
        missing_items = []

        # Iterate through the GUI requirements
        for target in self.required_assembly:
            match_found = False
            
            # 1. Identify what TYPE of brick we need (String -> Enum)
            target_enum_type = self.map_gui_str_to_cam_enum(target.type)
            
            # 2. Search the supply pile for this Type
            for idx in supply_indices:
                source = self.available_supply[idx]
                
                # Check for Match (Type vs Type)
                if source.type == target_enum_type:
                    
                    # --- MATCH FOUND ---
                    
                    # A. Transfer the Pose (Where to pick it up)
                    target.pickup_pose.x = source.pose.position.x
                    target.pickup_pose.y = source.pose.position.y
                    target.pickup_pose.z = source.pose.position.z
                    
                    # B. Transfer the UNIQUE ID (Which specific one it is)
                    target.id = str(source.id)

                    # --- C. TRANSFER LOCATION (NEW LOGIC) ---
                    # Map Camera "Side" (ABB=0, AR4=1) to Plan "Location" (1, 2)
                    if source.side == CamBrick.ABB:
                        target.location = 1  # ABB Section
                    elif source.side == CamBrick.AR4:
                        target.location = 2  # AR4 Section
                    else:
                        target.location = 0  # Grid
                    
                    # Add to final plan
                    final_plan.bricks.append(target)
                    
                    # Mark this specific brick ID as "Taken"
                    supply_indices.remove(idx)
                    match_found = True
                    break
            
            if not match_found:
                missing_items.append(f"{target.type}")

        # --- PUBLISH ---
        flag_msg = Bool()
        if not missing_items:
            flag_msg.data = True
            self.flag_pub.publish(flag_msg)
            self.plan_pub.publish(final_plan)
            
            # Useful log to verify assignments
            ids_locs = [f"ID:{b.id}->Loc:{b.location}" for b in final_plan.bricks]
            self.get_logger().info(f"✅ Assembly Ready. Assigned: {ids_locs}")
        else:
            flag_msg.data = False
            self.flag_pub.publish(flag_msg)
            self.get_logger().warn(f"❌ Missing types: {missing_items}", throttle_duration_sec=2)

    def map_gui_str_to_cam_enum(self, gui_string):
        """
        Maps GUI Strings to Camera Constants
        """
        mapping = {
            # Lowercase standard
            "i_shape": CamBrick.I_BRICK, 
            "l_shape": CamBrick.L_BRICK, 
            "t_shape": CamBrick.T_BRICK, 
            "z_shape": CamBrick.Z_BRICK, 
            
            # Uppercase/Database Variations
            "T_shape": CamBrick.T_BRICK,
            "L_shape": CamBrick.L_BRICK,
            "I_shape": CamBrick.I_BRICK,
            "Z_shape": CamBrick.Z_BRICK
        }
        return mapping.get(gui_string, 255)

def main(args=None):
    rclpy.init(args=args)
    node = AssemblySupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()