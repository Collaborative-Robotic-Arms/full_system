import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    dual_arms_share = get_package_share_directory("dual_arms")
    dual_arms_mtc_share = get_package_share_directory("dual_arms_mtc")
    mtc_config_path = os.path.join(dual_arms_mtc_share, 'config', 'hybrid_mtc_config.yaml')

    # 1. Manual Robot Description (URDF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([dual_arms_share, "urdf", "dual_arms_with_environment.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 2. Manual Semantic Description (SRDF)
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([dual_arms_share, "config", "dual_arms.srdf"]),
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # 3. Manual Kinematics & Planning
    kinematics_yaml = load_yaml("dual_arms", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("dual_arms", "config/joint_limits.yaml")
    
    # 4. Manual OMPL Pipeline
    ompl_config = load_yaml("dual_arms", "config/ompl_planning.yaml")
    ompl_planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "ompl": ompl_config if ompl_config else {}
    }

    return LaunchDescription([
        Node(
            package='dual_arms_mtc',
            executable='hybrid_mtc_controller',
            name='hybrid_mtc_controller',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {"robot_description_kinematics": kinematics_yaml},
                {"robot_description_planning": joint_limits_yaml},
                ompl_planning_pipeline,
                load_yaml('dual_arms_mtc', 'config/hybrid_mtc_config.yaml'),
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='dual_arms_mtc',
            executable='zone_detection_manager',
            name='zone_detection_manager',
            output='screen',
            parameters=[
                os.path.join(dual_arms_mtc_share, 'config', 'hybrid_mtc_config.yaml'),
                {'use_sim_time': True}
            ],
        ),
    ])