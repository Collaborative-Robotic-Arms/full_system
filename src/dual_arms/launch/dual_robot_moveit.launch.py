import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    # Define package paths
    dual_arms_pkg = FindPackageShare("dual_arms")

    # Load URDF using xacro
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([dual_arms_pkg, "urdf", "dual_arms.urdf.xacro"]),
        " ar_model:=mk3",
        " tf_prefix:="
    ])
    robot_description = {
        "robot_description": robot_description_content
    }

    # Build MoveIt2 config
    moveit_config = MoveItConfigsBuilder(
        robot_name="dual_arms",
        package_name="dual_arms"
    ).to_moveit_configs()
    print("moveit Config:", moveit_config)
    '''
    # Load controller configuration
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["moveit_simple_controller_manager"] = {
        "ros__parameters": {
            "controller_names": [
                "ar4_trajectory_controller",
                "irb120_trajectory_controller",
                "gripper_controller"
            ]
        }
    }
    moveit_config_dict["moveit_simple_controller_manager/ar4_trajectory_controller"] = {
        "ros__parameters": {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": [
                "ar4_joint_1",
                "ar4_joint_2",
                "ar4_joint_3",
                "ar4_joint_4",
                "ar4_joint_5",
                "ar4_joint_6"
            ]
        }
    }
    moveit_config_dict["moveit_simple_controller_manager/irb120_trajectory_controller"] = {
        "ros__parameters": {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6"
            ]
        }
    }
    
    moveit_config_dict["moveit_simple_controller_manager/gripper_controller"] = {
        "ros__parameters": {
            "action_ns": "gripper_action",
            "type": "GripperCommand",
            "default": True,
            "joints": [
                "ar4_gripper_jaw1_joint",
                "ar4_gripper_jaw2_joint"
            ]
        }
    }
    '''

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("dual_arms"),
                "launch",
                "dual_arms_gazebo.launch.py"
            ])
        ])
    )

    # Launch MoveIt2 move_group with a delay
    move_group = TimerAction(
        period=10.0,  # Delay to ensure Gazebo and controllers are ready
        actions=[generate_move_group_launch(moveit_config)]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("dual_arms"),
            "config",
            "moveit.rviz"
        ])],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        move_group,
        rviz_node
    ])
