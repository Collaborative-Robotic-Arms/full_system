#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# --- IMPORT MESSAGES ---
# 1. Source: GUI / Brick Processor Messages
from ros2_gui_bridge.msg import BrickArray as GuiArray
from ros2_gui_bridge.msg import Brick as GuiBrick

# 2. Source: Camera / Detection Messages
from dual_arms_msgs.msg import BricksArray as CamArray
from dual_arms_msgs.msg import Brick as CamBrick 

# 3. Destination: Supervisor Service and Message Types
from supervisor_package.srv import GetAssemblyPlan 
from supervisor_package.msg import Brick as SupervisorBrick 

class AssemblyAllocator(Node):
    def __init__(self):
        super().__init__('assembly_allocator')
        
        # --- INPUTS ---
        # Subscribes to the Brick Processor (GUI World Coords)
        self.gui_sub = self.create_subscription(
            GuiArray, '/processed_bricks', self.gui_callback, 10)
        
        # Subscribes to the Detection Mock/Pipeline (Hardware Reality)
        self.cam_sub = self.create_subscription(
            CamArray, '/detected_bricks', self.cam_callback, 10)
        
        # --- OUTPUTS ---
        # Service Server for the Supervisor to fetch the plan
        self.srv = self.create_service(
            GetAssemblyPlan, 
            'get_assembly_plan', 
            self.handle_plan_request
        )
        
        # Debugging publishers
        self.flag_pub = self.create_publisher(Bool, '/debug/assembly_ready', 10)
        
        # Internal State
        self.required_assembly = []   # Bricks from GUI
        self.available_supply = []     # Bricks from Camera
        self.latest_valid_plan = []    # Matched plan ready for supervisor
        self.is_ready = False
        
        # Matching Timer: runs at 2Hz
        self.timer = self.create_timer(0.5, self.validate_and_assign)
        self.get_logger().info('--- ASSEMBLY ALLOCATOR (INTEGRATED VERSION) READY ---')

    def gui_callback(self, msg):
        """Stores the list of bricks requested by the GUI."""
        self.required_assembly = msg.bricks

    def cam_callback(self, msg):
        """Stores the list of bricks currently detected by the camera."""
        self.available_supply = msg.bricks

    def handle_plan_request(self, request, response):
        """Serves the matched plan to the Supervisor."""
        if self.is_ready and self.latest_valid_plan:
            self.get_logger().info("Serving dynamic plan to Supervisor...")
            
            final_plan = []
            for gui_brick in self.latest_valid_plan:
                sup_brick = SupervisorBrick()
                
                # 1. Standard Metadata
                sup_brick.id = int(gui_brick.id)
                sup_brick.type = str(gui_brick.type) 
                
                # 2. Assign the PICKUP Pose (From Camera/Mock)
                # This uses the dynamic coordinates captured in validate_and_assign
                sup_brick.pickup_pose.position.x = gui_brick.pickup_pose.x
                sup_brick.pickup_pose.position.y = gui_brick.pickup_pose.y
                sup_brick.pickup_pose.position.z = gui_brick.pickup_pose.z
                sup_brick.pickup_pose.orientation.w = 1.0

                # 3. Assign the PLACE Pose (From GUI/Brick Processor)
                # These are the coordinates calculated from the grid indices
                sup_brick.place_pose.position.x = gui_brick.place_pose.x
                sup_brick.place_pose.position.y = gui_brick.place_pose.y
                sup_brick.place_pose.position.z = gui_brick.place_pose.z
                sup_brick.place_pose.orientation.w = 1.0

                # 4. Map sides
                sup_brick.start_side = "ABB" if gui_brick.location == 1 else "AR4"
                sup_brick.target_side = "SHARED"
                
                final_plan.append(sup_brick)

            response.plan = final_plan
            response.success = True
        else:
            self.get_logger().warn("Supervisor requested plan but system is NOT READY")
            response.success = False
        return response

    def validate_and_assign(self):
        """Matches GUI requirements with Camera reality to update pickup_pose."""
        if not self.required_assembly or not self.available_supply:
            self.update_ready_status(False)
            return

        supply_pool = list(self.available_supply)
        temp_plan = [] 
        all_matched = True

        for target in self.required_assembly:
            match_found = False
            target_enum_type = self.map_gui_str_to_cam_enum(target.type)
            
            for idx, source in enumerate(supply_pool):
                if source.type == target_enum_type:
                    # --- DYNAMIC OVERWRITE ---
                    # We take the ID and coordinates from the CAMERA
                    target.id = str(source.id)
                    target.pickup_pose.x = float(source.pose.position.x)
                    target.pickup_pose.y = float(source.pose.position.y)
                    target.pickup_pose.z = float(source.pose.position.z)

                    # Determine robot side
                    if source.side == CamBrick.ABB:
                        target.location = 1 
                    elif source.side == CamBrick.AR4:
                        target.location = 2 
                    
                    temp_plan.append(target)
                    supply_pool.pop(idx) 
                    match_found = True
                    break
            
            if not match_found:
                all_matched = False

        # System is only ready if EVERY brick from the GUI has a physical match
        is_ready_now = all_matched and (len(temp_plan) == len(self.required_assembly))
        self.update_ready_status(is_ready_now)
        
        if self.is_ready:
            self.latest_valid_plan = temp_plan

    def update_ready_status(self, status):
        """Publishes the ready flag for debugging."""
        self.is_ready = status
        flag_msg = Bool()
        flag_msg.data = self.is_ready
        self.flag_pub.publish(flag_msg)

    def map_gui_str_to_cam_enum(self, gui_string):
        """Maps shape strings to camera message enum values."""
        mapping = {
            "i_shape": CamBrick.I_BRICK, "I_shape": CamBrick.I_BRICK,
            "l_shape": CamBrick.L_BRICK, "L_shape": CamBrick.L_BRICK,
            "t_shape": CamBrick.T_BRICK, "T_shape": CamBrick.T_BRICK,
            "z_shape": CamBrick.Z_BRICK, "Z_shape": CamBrick.Z_BRICK
        }
        return mapping.get(gui_string, 255)

def main(args=None):
    rclpy.init(args=args)
    node = AssemblyAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()