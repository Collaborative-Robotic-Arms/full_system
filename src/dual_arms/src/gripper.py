#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GripperService(Node):
    def __init__(self):
        super().__init__('ar4_gripper_service')
        self.srv = self.create_service(SetBool, 'ar4_gripper/set', self.set_gripper)
        self.pub = self.create_publisher(JointTrajectory, '/ar4_gripper_controller/joint_trajectory', 10)

    def set_gripper(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ['ar4_gripper_jaw1_joint']

        point = JointTrajectoryPoint()
        if request.data:  # True = Open
            point.positions = [0.014]
            response.message = "Gripper opened"
        else:  # False = Close
            point.positions = [0.009]
            response.message = "Gripper closed"

        point.time_from_start.sec = 1
        traj.points.append(point)

        self.pub.publish(traj)
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()