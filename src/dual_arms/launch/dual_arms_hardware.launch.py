import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution,LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError and WindowsError where available.
        return None

# EVALUATE INPUT ARGUMENTS:
def AssignArgument(ARGUMENT):
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():

    
    #CONFIGS available: ABB, AR4, Dual Arms
    CONFIG = AssignArgument("config")

    # Launch arguments for ar4
    serial_port = LaunchConfiguration("serial_port")
    calibrate = LaunchConfiguration("calibrate")
    arduino_serial_port = LaunchConfiguration("arduino_serial_port")
    use_sim_time = False
    include_gripper = LaunchConfiguration("include_gripper")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")
    moveit_servo = LaunchConfiguration("moveit_servo")




    PACKAGE_NAME = "dual_arms"
    # =============================================================================
    # === 2. CONFIGURATION LOADING                                              ===
    # =============================================================================
    pkg_share = get_package_share_directory('dual_arms')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([get_package_share_directory('dual_arms'), "urdf", "dual_arms_hardware.xacro"]),
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([get_package_share_directory('dual_arms'), "config/hardware", "dual_arms_hardware.srdf"]),
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}
    
    kinematics_yaml = load_yaml("dual_arms", "config/hardware/kinematics_hardware.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    joint_limits_yaml = load_yaml("dual_arms", "config/hardware/joint_limits_hardware.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}
    servo_params = {
        "moveit_servo": ParameterBuilder("dual_arms")
        .yaml("config/ar4_servo.yaml")
        .to_dict()
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_yaml = load_yaml("dual_arms", "config/ompl_planning.yaml")
    if ompl_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_yaml)
    
    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml("dual_arms", "config/hardware/moveit_controllers_hardware.yaml")

    # MoveIt!2 Parameters:
    moveit_controllers = moveit_simple_controllers_yaml
    
    # trajectory_execution = {"moveit_manage_controllers": True}
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    # planning_scene_monitor_parameters = {"publish_planning_scene": True}
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
    }


    
    # =============================================================================
    # === 3. NODE DEFINITIONS                                                   ===
    # =============================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": False}]
    )

    # --- MOVEIT SERVO NODE (Output Fix) ---
    # The logs showed it defaulting to /panda_arm_controller/joint_trajectory.
    # We fix this by explicitly setting the command_out_topic parameter.
 
    # visp_controller_node = Node(
    #     package="perception_setup",
    #     executable="visp_ibvs_controller",
    #     output="screen",
    #     parameters=[{"use_sim_time": True}]
    # )
    # tf_camera_link = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments = ["0", "0", "0", "0", "0", "0", "ar4_ee_link", "ar4_camera_link"],
    #     parameters=[{"use_sim_time": True}]
    # )

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
            moveit_controllers,
            servo_params,  
            {"moveit_servo.command_out_topic": "/ar4_trajectory_controller/joint_trajectory"},
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
        ],
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory('dual_arms'), "config", "moveit.rviz")],
        parameters=[robot_description, robot_description_semantic, ompl_planning_pipeline_config, robot_description_kinematics, robot_description_planning,{"use_sim_time": False}],
    )


        # ros2_control:
    ros2_controllers_path = os.path.join(get_package_share_directory("dual_arms"), "config/hardware", "dual_arms_controller_hardware.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
        #arguments=["--ros-args", "--log-level", "debug"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager","--switch-timeout", "30.0"],
        parameters=[{"use_sim_time": False}],
    )

    ar4_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["ar4_trajectory_controller", "--controller-manager", "/controller_manager" ,"--controller-manager-timeout", "120"],
        output="screen",
    )

    irb120_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["irb120_controller", "--controller-manager", "/controller_manager"],
    )


    # ============================================= #
    # ============== ABB: RWS CLIENT ============== #
    rws_client = Node(
        package="abb_rws_client",
        executable="rws_client",
        name="rws_client",
        output="screen",
        parameters=[
            {"robot_ip": '192.168.125.1'},
            {"robot_port": 80},
            {"robot_nickname": "ROB_1"},
            {"polling_rate": 5.0},
            {"no_connection_timeout": False},
            
        ],
        #arguments=["--ros-args", "--log-level", "debug"],
    )


    # ============================================================================= #
    # Servo Configuration
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "ar_manipulator"}

    # servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node",
    #     parameters=[
    #         servo_params,
    #         acceleration_filter_update_period,
    #         planning_group_name,
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #         joint_limits,
    #         planning_scene_monitor_parameters,
    #         {"use_sim_time": use_sim_time},
    #     ],
    #     output="screen",
    #     condition=IfCondition(moveit_servo),
    # )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"))
    ld.add_action(
        DeclareLaunchArgument(
            "calibrate", default_value="True", choices=["True", "False"]
        )
    )
    ld.add_action(
        DeclareLaunchArgument("arduino_serial_port", default_value="/dev/ttyUSB0")
    )
    # ld.add_action(
    #     DeclareLaunchArgument(
    #         "include_gripper", default_value="True", choices=["True", "False"]
    #     )
    # )
    ld.add_action(
        DeclareLaunchArgument(
            "moveit_servo",
            default_value="False",
            choices=["True", "False"],
            description="Run moveit2 servo",
        )
    )



        # Event-based controller spawners
    # ld.add_action(
    #     RegisterEventHandler(
    #         OnProcessStart(
    #             target_action=controller_manager_node,
    #             on_start=[
    #                 spawn_joint_state_broadcaster,
    #                 spawn_joint_controller,
    #                 spawn_gripper_controller,
    #             ],
    #         )
    #     )
    # )



    # ========== CELL INFORMATION ========== #
    print("")
    print("===== " + PACKAGE_NAME + ": Robot Bringup + MoveIt!2 Framework (" + PACKAGE_NAME + "_bringup) =====")
    print("ABB Robot IP Address -> " + '192.168.125.1')
    print("Robot configuration:")
    print("")

    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(move_group_node)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(rws_client)
    ld.add_action(RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[irb120_controller_spawner,ar4_controller_spawner])
        ))
    # =============================================================================
    # === 5. LAUNCH RETURN                                                      ===
    # =============================================================================
    return ld
