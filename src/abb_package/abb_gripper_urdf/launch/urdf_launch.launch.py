import os
import yaml
import pprint
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

# --- HELPER: LOAD YAML ---
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

# --- HELPER: FLATTEN NESTED DICTS (Fixes MoveIt parameter lookup) ---
def flatten_config(dictionary, parent_key='', sep='.'):
    items = []
    for k, v in dictionary.items():
        new_key = parent_key + sep + k if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_config(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)

def generate_launch_description():
    print("\n" + "="*50)
    print("=== FINAL EFFORT CONTROL LAUNCH SEQUENCE ===")
    print("="*50 + "\n")

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_name = 'abb_gripper_urdf'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 1. ROBOT DESCRIPTION
    xacro_file = os.path.join(pkg_share, 'urdf', 'abb_gripper.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    srdf_file = os.path.join(pkg_share, 'config', 'abb_gripper.srdf')
    robot_description_semantic_content = Command(['xacro ', srdf_file])
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}

    # 2. OMPL CONFIGURATION (FLATTENING)
    ompl_yaml = load_yaml(pkg_name, "config/ompl_planning.yaml")
    ompl_config_flat = flatten_config(ompl_yaml) if ompl_yaml else {}

    print("--- FINAL FLATTENED OMPL CONFIG (MOVEIT) ---")
    pprint.pprint(ompl_config_flat)
    print("--------------------------------------------")
    
    moveit_controllers = load_yaml(pkg_name, 'config/moveit_controllers.yaml')
    kinematics_config = load_yaml(pkg_name, 'config/kinematics.yaml')

    # 3. SIMULATION SETUP
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_share]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(), 
    )

    # 4. NODES
    spawn_entity = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'abb_gripper', '-z', '0.5'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'], output='screen',
    )

    gripper_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller'], output='screen',
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_config_flat, 
            {'robot_description_kinematics': kinematics_config},
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'moveit_simple_controller_manager': moveit_controllers},
            {'use_sim_time': use_sim_time},
            {'moveit_manage_controllers': True},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_config_flat,
            {'robot_description_kinematics': kinematics_config},
            {'use_sim_time': use_sim_time}
        ],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[gripper_controller])),
        RegisterEventHandler(OnProcessExit(target_action=gripper_controller, on_exit=[move_group_node])),
        RegisterEventHandler(OnProcessExit(target_action=gripper_controller, on_exit=[rviz_node])),
    ])