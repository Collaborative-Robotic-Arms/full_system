#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose, Point

# Import the specific messages the SERVICES expect
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.msg import SuperBrick  # <-- This is the one that was crashing

# Import these for the other services
from dual_arms_msgs.srv import GetGrasp, DetectBricks
from dual_arms_msgs.msg import Brick, GraspPoint

class MockRobotSystem(Node):
    def __init__(self):
        super().__init__('mock_robot_system')

        self.plan_srv = self.create_service(GetAssemblyPlan, 'get_assembly_plan', self.get_plan_callback)
        self.detect_srv = self.create_service(DetectBricks, 'detect_bricks', self.detect_bricks_callback)
        self.grasp_srv = self.create_service(GetGrasp, 'grasp/get_grasp_point', self.get_grasp_callback)


        self.get_logger().info('Mock System Ready. Testing AR4 sequence...')

    def euler_to_quaternion(self, roll, pitch, yaw):
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

    def get_plan_callback(self, request, response):
            self.get_logger().info('Mock: Sending Assembly Plan...')
            
            brick = SuperBrick()
            brick.id = 1
            brick.type = "I_BRICK"      # Must be a string based on your .msg
            brick.start_side = "AR4"    # Must be a string based on your .msg
            brick.target_side = "GRID"  # Must be a string based on your .msg
            
            # Initialize the poses so they aren't null
            brick.pickup_pose = Pose()
            brick.pickup_pose.position.x = 0.5
            brick.pickup_pose.orientation.w = 1.0
            
            brick.place_pose = Pose()
            brick.place_pose.position.x = 0.8
            brick.place_pose.orientation.w = 1.0
            
            response.plan = [brick]
            return response

    def detect_bricks_callback(self, request, response):
        self.get_logger().info('Mock: Sending Detected Bricks...')
        # DetectBricks uses dual_arms_msgs/Brick
        brick = Brick()
        brick.id = 1
        brick.pose.position.x = 0.5
        brick.pose.position.z = 0.1
        brick.pose.orientation.w = 1.0
        
        response.bricks = [brick]
        response.handover_pose = Pose()
        response.handover_pose.orientation.w = 1.0
        return response

    def get_grasp_callback(self, request, response):
        self.get_logger().info(f'Mock: Sending Grasp for Brick {request.brick_index}')
        response.success = True
        gp = GraspPoint()
        gp.pose.position = Point(x=0.65, y=0.0, z=0.14)
        
        # --- DEFINE YOUR ANGLES HERE (in degrees) ---
        roll = 180.0
        pitch = 0.0 # Example: Pointing the gripper straight down
        yaw = 90.0

        # Convert to Radians
        r_rad = math.radians(roll)
        p_rad = math.radians(pitch)
        y_rad = math.radians(yaw)

        # Convert to Quaternion
        q = self.euler_to_quaternion(r_rad, p_rad, y_rad)

        # Assign to the message
        gp.pose.orientation.x = q[0]
        gp.pose.orientation.y = q[1]
        gp.pose.orientation.z = q[2]
        gp.pose.orientation.w = q[3]
        self.get_logger().info(f'Quaternion: {q[0]}, {q[1]}, {q[2]}, {q[3]}') 
        response.grasp_point = gp
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MockRobotSystem()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()