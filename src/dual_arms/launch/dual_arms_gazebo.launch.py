import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    ar_model_arg = DeclareLaunchArgument(
        "ar_model",
        default_value="mk3",
        choices=["mk1", "mk2", "mk3"],
        description="Model of AR4 robot (mk1, mk2, or mk3)"
    )

    tf_prefix_arg = DeclareLaunchArgument(
        "tf_prefix",
        default_value="",
        description="Prefix for TF frames (used for namespacing when multiple robots are spawned)"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    ar_model = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Define package paths
    dual_arms_pkg = FindPackageShare("dual_arms")
    annin_ar4_description_pkg = FindPackageShare("annin_ar4_description")

    # Load URDF using xacro
    robot_description_path = os.path.join(
        get_package_share_directory('dual_arms'),
        'urdf',
        "dual_arms.urdf")
        # Read the URDF file
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}
    # Launch Gazebo with an empty world
    world = os.path.join(
        get_package_share_directory("annin_ar4_gazebo"),
        "worlds",
        "empty.world"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )
    # Joint State Publisher node
    joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen'
    )
    # Spawn robot into Gazebo
    spawn_dual_arms = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_dual_arms",
        output="screen",
        arguments=[
            "-name", "dual_arms",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.01"
        ],
        parameters=[{"use_sim_time": True}]
    )

    # Controller Manager Nodes
    ros2_controllers = PathJoinSubstitution(
    [
    get_package_share_directory('dual_arms'),
    "config",
    "dual_arms_controller.yaml",
    ]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers,
            {"use_sim_time": True}
        ],
        output="both"
    )

    joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--param-file", ros2_controllers],
    )
    # Controller Manager Spawner node to launch the arm group controller
    ar4_trajectory_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['ar4_trajectory_controller',"--param-file", ros2_controllers],
    output='screen'
    )
        # Controller Manager Spawner node to launch the arm group controller
    irb120_trajectory_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['irb120_trajectory_controller',"--param-file", ros2_controllers],
    output='screen'
    )
    gripper_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['gripper_controller',"--param-file", ros2_controllers],
    output='screen'
    )
# add the bridge node for mapping the topics between ROS and GZ sim
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
    # Clock (IGN -> ROS2)
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    # Joint states (IGN -> ROS2)
    '/world/empty/robot_arm_urdf/rrbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    ],
    remappings=[
    ('/world/empty/model/dual_arms/joint_state', 'joint_states'),
    ],
    output='screen'
    )
    return LaunchDescription([
        ar_model_arg,
        tf_prefix_arg,
        use_sim_time_arg,
        bridge,
        gazebo,
        spawn_dual_arms,
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        #ros2_control_node,
        
        joint_state_broadcaster_spawner,
        ar4_trajectory_controller_spawner,
        irb120_trajectory_controller_spawner,
        gripper_controller_spawner
    ])
