import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # 1. Paths to your files
    dual_arms_share = get_package_share_directory('dual_arms')
    urdf_path = os.path.join(dual_arms_share, 'urdf', 'dual_arms_with_environment.xacro')
    srdf_path = os.path.join(dual_arms_share, 'config', 'dual_arms.srdf')

    # 2. Process Xacro to get URDF string
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Read SRDF
    with open(srdf_path, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # 4. Kinematics parameters
    kinematics_yaml = {
        'robot_description_kinematics': {
            'ar_manipulator': {
                'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin'
            }
        }
    }

    # 5. The Node
    return LaunchDescription([
        Node(
            package='point_control_pkg',
            executable='pose_commander_action',
            name='AR4_tool_control_final',
            output='screen',
            emulate_tty=True,
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                {'use_sim_time': True},
                # Relaxed tolerance for long-running simulations
                {'joint_state_monitor.max_joint_state_age': 0.1} 
            ]
        )
    ])