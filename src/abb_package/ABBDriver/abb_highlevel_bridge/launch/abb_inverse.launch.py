from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # --- 1. Load MoveIt Configuration (CRITICAL FOR ABB_TASK_SERVER) ---
    # We load the 'dual_arms' config because that matches your running simulation.
    # This provides 'robot_description' and 'robot_description_semantic'.
    moveit_config = MoveItConfigsBuilder("dual_arms", package_name="dual_arms").to_moveit_configs()

    # --- 2. Existing Nodes (From your file) ---
    
    # A. SRDF Path Argument (Kept for your semantic_publisher)
    SRDF_PACKAGE_NAME = 'ros2srrc_irb120_moveit2' 
    pkg_share_dir = get_package_share_directory(SRDF_PACKAGE_NAME)
    srdf_file_path = os.path.join(pkg_share_dir, 'config', 'irb120.srdf')

    srdf_path_arg = DeclareLaunchArgument(
        'srdf_path',
        default_value=srdf_file_path,
        description='Path to SRDF'
    )

    # B. Semantic Publisher
    semantic_publisher_node = Node(
        package='abb_highlevel_bridge', 
        executable='semantic_publisher',
        name='semantic_publisher',
        output='screen',
        parameters=[{'srdf_path': LaunchConfiguration('srdf_path')}]
    )

    # C. Inverse Kinematics
    abb_inverse_node = Node(
        package='abb_highlevel_bridge', 
        executable='inverse_kinematics',
        name='abb_inverse_control',
        output='screen'
    )

    # D. Gripper Client
    gripper_node = Node(
        package='abb_highlevel_bridge',
        executable='abb_endeffector_control_client',
        name='gripper_control_client',
        output='screen'
    )

    # --- 3. THE MISSING NODE (ABB Task Server) ---
    # This is the one that was crashing. We add it here with the correct params.
    abb_task_server_node = Node(
        package="abb_highlevel_bridge",
        executable="abb_task_server",
        name="abb_task_server",
        output="screen",
        # Pass the MoveIt configs so it can construct the robot model
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True} # Important for syncing with Gazebo
        ],
    )

    return LaunchDescription([
        srdf_path_arg,
        abb_inverse_node,
        semantic_publisher_node,
        gripper_node,
        abb_task_server_node  # <--- Added this to the launch list
    ])