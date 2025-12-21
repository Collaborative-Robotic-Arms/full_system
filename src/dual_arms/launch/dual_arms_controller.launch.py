from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, file_path), 'r') as f:
        return f.read()

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, file_path), 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    pkg_name = 'dual_arms'
    robot_description = {'robot_description': load_file(pkg_name, 'urdf/dual_arms.urdf')}
    robot_description_semantic = {'robot_description_semantic': load_file(pkg_name, 'config/dual_arms.srdf')}
    robot_description_kinematics = {'robot_description_kinematics': load_yaml(pkg_name, 'config/kinematics.yaml')}
    joint_limits = {'robot_description_planning': load_yaml(pkg_name, 'config/joint_limits.yaml')}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Dual arms controller
    dual_arms_node = Node(
        package='dual_arms',
        executable='dual_arms_controller',
        name='dual_arms_controller',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        dual_arms_node
    ])
