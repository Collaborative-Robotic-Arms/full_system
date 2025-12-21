from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Point Control Node
        Node(
            package='point_control_pkg',
            executable='pose_commander',
            name='ar4_pose_commander'
        ),
        # 2. ViSP IBVS Node
        Node(
            package='perception_setup',
            executable='visp_ibvs_controller',
            name='visp_controller'
        ),
        # 3. Supervisor Node
        Node(
            package='supervisor_package',
            executable='supervisor',
            name='system_supervisor'
        )
    ])