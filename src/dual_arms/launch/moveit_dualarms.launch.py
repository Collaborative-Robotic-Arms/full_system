import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# =============================================================================
# === Helper functions                                                       ===
# =============================================================================


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


# =============================================================================
# === Launch Description                                                     ===
# =============================================================================


def generate_launch_description():

    # =========================================================================
    # === 1. Gazebo / Ignition resource paths                                 ===
    # =========================================================================

    resource_paths = [
        os.path.join(get_package_share_directory("assembly_environment"), ".."),
        os.path.join(get_package_share_directory("annin_ar4_description"), ".."),
        os.path.join(get_package_share_directory("ros2srrc_robots"), ".."),
        os.path.join(get_package_share_directory("dual_arms"), "worlds"),
        os.path.join(get_package_share_directory("dual_arms"), "materials"),
        os.path.join(get_package_share_directory("dual_arms"), ".."),
    ]

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(resource_paths),
    )

    # =========================================================================
    # === 2. Paths & configuration files                                      ===
    # =========================================================================

    pkg_share = get_package_share_directory("dual_arms")
    world_file_path = os.path.join(pkg_share, "worlds", "dual.sdf")

    # --- Robot description (URDF / xacro) ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share, "urdf", "dual_arms_with_environment.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # --- Semantic description (SRDF) ---
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "config", "dual_arms.srdf"]),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    # --- Kinematics ---
    kinematics_yaml = load_yaml("dual_arms", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # --- Joint limits / planning ---
    joint_limits_yaml = load_yaml("dual_arms", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # =========================================================================
    # === 3. MoveIt planning pipeline configuration                            ===
    # =========================================================================

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_yaml = load_yaml("dual_arms", "config/ompl_planning.yaml")
    if ompl_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_yaml)

    moveit_controllers_yaml_content = load_yaml(
        "dual_arms", "config/moveit_controllers.yaml"
    )

    trajectory_execution = {
    "moveit_manage_controllers": True,
    "trajectory_execution.execution_duration_monitoring": False,
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    # THIS IS THE CRITICAL LINE:
    "trajectory_execution.execution_velocity_scaling": 1.0,
}

    servo_params = load_yaml("dual_arms", "config/ar4_servo.yaml")

    # =========================================================================
    # === 4. Gazebo / Ignition                                                 ===
    # =========================================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r {world_file_path}"}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "dual_arms_scene",
            "-J",
            "ar4_joint_2",
            "0.0",
            "-J",
            "ar4_joint_3",
            "1.0",
            "-J",
            "ar4_joint_5",
            "1.0",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # =========================================================================
    # === 5. Core state publishers                                             ===
    # =========================================================================

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    tf_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "ar4_ee_link", "ar4_camera_link"],
        parameters=[{"use_sim_time": True}],
    )

    # =========================================================================
    # === 6. MoveIt nodes                                                      ===
    # =========================================================================

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_yaml_content,
            planning_scene_monitor_parameters,
            {
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "capabilities": "move_group/ExecuteTaskSolutionCapability"
            },
            {"use_sim_time": True},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(pkg_share, "config", "moveit.rviz"),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": True},
        ],
    )
    dual_arms_command_node = Node(
        package="dual_arms",
        executable="dual_arms_controller",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # --- MoveIt Servo (currently disabled) ---
    moveit_servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            servo_params,
            {"use_sim_time": True},
            {
                "moveit_servo.command_out_topic": "/ar4_trajectory_controller/joint_trajectory"
            },
        ],
    )

    # =========================================================================
    # === 7. Perception / bridges                                              ===
    # =========================================================================

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={os.path.join(pkg_share, 'config', 'gz_bridge.yaml')}",
        ],
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/environment_camera/image_raw"],
    )

    visp_controller_node = Node(
        package="perception_setup",
        executable="visp_ibvs_controller",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # =========================================================================
    # === 8. Controllers & spawners                                            ===
    # =========================================================================

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--switch-timeout",
            "30.0",
        ],
        parameters=[{"use_sim_time": True}],
    )

    ar4_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ar4_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    other_controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["ar4_gripper_controller", "-c", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["irb120_trajectory_controller", "-c", "/controller_manager",],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["irb120_gripper_controller", "-c", "/controller_manager"],
        ),
    ]

    # =========================================================================
    # === 9. Optional diagnostics (disabled)                                   ===
    # =========================================================================

    diagnostic_topic_check = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "echo",
                    "\n========== DEBUG: SERVO NODE CONNECTIONS ==========",
                    ";",
                    "ros2",
                    "node",
                    "info",
                    "/servo_node",
                    ";",
                    "echo",
                    "\n========== DEBUG: ALL TRAJECTORY TOPICS ==========",
                    ";",
                    "ros2",
                    "topic",
                    "list",
                    "|",
                    "grep",
                    "trajectory",
                    ";",
                    "echo",
                    "===================================================\n",
                ],
                output="screen",
                shell=True,
            )
        ],
    )

    # =========================================================================
    # === 10. Launch description return                                        ===
    # =========================================================================

    return LaunchDescription(
        [
            gz_resource_path,
            gazebo,
            robot_state_publisher_node,
            spawn_entity,
            move_group_node,
            rviz_node,
            ros_gz_bridge,
            ros_gz_image_bridge,
            # moveit_servo_node,
            tf_camera_link,
            dual_arms_command_node,
            # diagnostic_topic_check,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[ar4_controller_spawner] + other_controllers,
                )
            ),
        ]
    )
