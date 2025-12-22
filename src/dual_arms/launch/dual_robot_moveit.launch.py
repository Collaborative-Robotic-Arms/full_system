import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    # 1. Define package paths
    dual_arms_pkg = FindPackageShare("dual_arms")

    # 2. Build MoveIt2 config properly
    # Note: robot_name here is used to locate default folders, 
    # but we override them with explicit file paths.
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="dual_arms_with_environment", 
            package_name="dual_arms"
        )
        .robot_description(
            file_path="urdf/dual_arms_with_environment.xacro",
            mappings={
                "ar_model": "mk3",
                "tf_prefix": ""
            }
        )
        .robot_description_semantic(file_path="config/dual_arms.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 3. Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                dual_arms_pkg,
                "launch",
                "dual_arms_gazebo.launch.py"
            ])
        ])
    )

    # 4. Robot State Publisher (Broadcasts the URDF to /robot_description)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ]
    )

    # 5. Launch MoveIt2 move_group
    move_group = generate_move_group_launch(moveit_config)

    # 6. Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            dual_arms_pkg,
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
        robot_state_publisher,
        move_group,
        rviz_node
    ])