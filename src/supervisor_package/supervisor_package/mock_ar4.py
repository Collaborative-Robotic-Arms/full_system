#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose, Point

# Import the specific messages the SERVICES expect
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.msg import SuperBrick  

# Import these for the other services
from dual_arms_msgs.srv import GetGrasp, DetectBricks
from dual_arms_msgs.msg import Brick, GraspPoint

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts euler angles (radians) to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy # x
    q[1] = cr * sp * cy + sr * cp * sy # y
    q[2] = cr * cp * sy - sr * sp * cy # z
    q[3] = cr * cp * cy + sr * sp * sy # w
    return q
    
# --- DEFINE YOUR ANGLES HERE (in degrees) ---
roll = 180.0
pitch = 0.0 
yaw = 0.0

# Convert to Radians
r_rad = math.radians(roll)
p_rad = math.radians(pitch)
y_rad = math.radians(yaw)

# Convert to Quaternion
q = euler_to_quaternion(r_rad, p_rad, y_rad)
q1 = euler_to_quaternion(math.radians(180.0), math.radians(0.0), math.radians(90.0))

class MockRobotSystem(Node):
    def __init__(self):
        super().__init__('mock_robot_system')

        self.plan_srv = self.create_service(GetAssemblyPlan, 'get_assembly_plan', self.get_plan_callback)
        self.detect_srv = self.create_service(DetectBricks, 'detect_bricks', self.detect_bricks_callback)
        self.grasp_srv = self.create_service(GetGrasp, 'grasp/get_grasp_point', self.get_grasp_callback)

        self.get_logger().info('Mock System Ready. Testing AR4 sequence...')

    def get_plan_callback(self, request, response):
            self.get_logger().info('Mock: Sending Assembly Plan...')
            
            plan = []
            
            brick = SuperBrick()
            brick.id = 1
            brick.type = "I_BRICK"      
            brick.start_side = "AR4"    
            brick.target_side = "GRID"  
            
            brick.pickup_pose = Pose()
            brick.pickup_pose.position.x = 0.5
            brick.pickup_pose.orientation.w = 1.0
            
            brick.place_pose = Pose()
            brick.place_pose.position.x = 0.7
            brick.place_pose.position.y = 0.2
            brick.place_pose.position.z = 0.14
            brick.place_pose.orientation.x = q[0]
            brick.place_pose.orientation.y = q[1]
            brick.place_pose.orientation.z = q[2]
            brick.place_pose.orientation.w = q[3]
            
            plan.append(brick)

            brick2 = SuperBrick()
            brick2.id = 2
            brick2.type = "T_BRICK"      
            brick2.start_side = "ABB"    
            brick2.target_side = "GRID"  
            
            brick2.pickup_pose = Pose()
            brick2.pickup_pose.position.x = 0.5
            brick2.pickup_pose.orientation.y = 1.0
            
            brick2.place_pose = Pose()
            brick2.place_pose.position.x = 0.4
            brick2.place_pose.position.y = 0.08
            brick2.place_pose.position.z = 0.22
            brick2.place_pose.orientation.x = 0.0
            brick2.place_pose.orientation.y = 0.0
            brick2.place_pose.orientation.z = 1.0
            brick2.place_pose.orientation.w = 0.0
                        
            plan.append(brick2)         
            
            brick3 = SuperBrick()
            brick3.id = 3
            brick3.type = "T_BRICK"      
            brick3.start_side = "AR4"    
            brick3.target_side = "GRID"  
            
            brick3.pickup_pose = Pose()
            brick3.pickup_pose.position.x = 0.5
            brick3.pickup_pose.orientation.y = 1.0
        
            brick3.place_pose = Pose()
            brick3.place_pose.position.x = 0.6
            brick3.place_pose.position.y = -0.1
            brick3.place_pose.position.z = 0.14
            brick3.place_pose.orientation.x = q1[0]
            brick3.place_pose.orientation.y = q1[1]
            brick3.place_pose.orientation.z = q1[2]
            brick3.place_pose.orientation.w = q1[3]
                        
            plan.append(brick3)            
            
            brick4 = SuperBrick()
            brick4.id = 4
            brick4.type = "I_BRICK"      
            brick4.start_side = "ABB"    
            brick4.target_side = "GRID"  
        
            brick4.pickup_pose = Pose()
            brick4.pickup_pose.position.x = 0.5
            brick4.pickup_pose.orientation.y = 1.0
    
            brick4.place_pose = Pose()
            brick4.place_pose.position.x = 0.35
            brick4.place_pose.position.y = -0.15
            brick4.place_pose.position.z = 0.22
            brick4.place_pose.orientation.x = 0.0
            brick4.place_pose.orientation.y = 0.0
            brick4.place_pose.orientation.z = 1.0
            brick4.place_pose.orientation.w = 0.0
                        
            plan.append(brick4)
            
            response.plan = plan
            return response

    def detect_bricks_callback(self, request, response):
            self.get_logger().info('Mock: Sending Detected Bricks (IDs 1 & 2)...')
            
            # --- Brick 1 ---
            brick1 = Brick()
            brick1.id = 1
            brick1.pose.position.x = 0.5
            brick1.pose.position.z = 0.14
            brick1.pose.orientation.w = 1.0
            
            # --- Brick 2 ---
            brick2 = Brick()
            brick2.id = 2
            brick2.pose.position.x = 0.4 
            brick2.pose.position.z = 0.22
            brick2.pose.orientation.w = 1.0
            
            # --- Brick 3 ---
            brick3 = Brick()
            brick3.id = 3
            brick3.pose.position.x = 0.5 
            brick3.pose.position.z = 0.14
            brick3.pose.orientation.w = 1.0

            # --- Brick 4 ---
            brick4 = Brick()
            brick4.id = 4
            brick4.pose.position.x = 0.4 
            brick4.pose.position.z = 0.22
            brick4.pose.orientation.w = 1.0
            
            response.bricks = [brick1, brick2, brick3, brick4]
            
# --- THE TRUE HANDOVER POSE ---
            # Center of the table, elevated. 
            response.handover_pose = Pose()
            response.handover_pose.position.x = 0.50  
            response.handover_pose.position.y = 0.00
            response.handover_pose.position.z = 0.30  # Raised slightly higher for a "handshake"
            
            # The "Handshake" Orientation:
            # We want the gripper pointing horizontally along the X-axis towards the other robot,
            # not straight down into the table. 
            # A pure horizontal pointing orientation in quaternions (Pitch = 90 deg):
            response.handover_pose.orientation.x = 0.0
            response.handover_pose.orientation.y = 0.7071  # 90-degree pitch
            response.handover_pose.orientation.z = 0.0
            response.handover_pose.orientation.w = 0.7071
            
            return response

    def get_grasp_callback(self, request, response):
            self.get_logger().info(f'Mock: Processing Grasp Request for Brick ID: {request.brick_index}')
            
            response.success = True
            gp = GraspPoint()

            if request.brick_index == "1":
                self.get_logger().info("Mock: Providing Grasp Point for Brick 1")
                gp.pose.position = Point(x=0.0, y=0.0, z=0.14)
                gp.pose.orientation.x = 0.0
                gp.pose.orientation.y = 0.0
                gp.pose.orientation.z = 0.0
                gp.pose.orientation.w = 1.0

            elif request.brick_index == "2":
                self.get_logger().info("Mock: Providing Grasp Point for Brick 2")
                # FIX: X coordinate updated to be safely within the ABB arm's reach
                gp.pose.position = Point(x=0.75, y=-0.3, z=0.22)
                gp.pose.orientation.x = 0.0
                gp.pose.orientation.y = 0.0
                gp.pose.orientation.z = 1.0
                gp.pose.orientation.w = 0.0

            elif request.brick_index == "3":
                self.get_logger().info("Mock: Providing Grasp Point for Brick 3")
                gp.pose.position = Point(x=0.02, y=0.02, z=0.14)
                gp.pose.orientation.x = 0.0
                gp.pose.orientation.y = 0.0
                gp.pose.orientation.z = 0.0
                gp.pose.orientation.w = 1.0
                
            elif request.brick_index == "4":
                self.get_logger().info("Mock: Providing Grasp Point for Brick 4")
                # FIX: X coordinate updated to be safely within the ABB arm's reach
                gp.pose.position = Point(x=0.75, y=-0.25, z=0.22)
                gp.pose.orientation.x = 0.0
                gp.pose.orientation.y = 0.0
                gp.pose.orientation.z = 1.0
                gp.pose.orientation.w = 0.0
                
            else:
                self.get_logger().warn(f"Mock: Brick ID {request.brick_index} not recognized!")
                response.success = False

            response.grasp_point = gp
            return response

def main(args=None):
    rclpy.init(args=args)
    node = MockRobotSystem()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()