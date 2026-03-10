"""
This launch file starts:
1. Hybrid MTC Controller (C++ node)
2. Zone Detection Manager (C++ node)
3. MoveIt configurations injected for MTC

Usage:
  ros2 launch dual_arms_mtc hybrid_mtc_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    # Get package directories
    dual_arms_mtc_dir = get_package_share_directory('dual_arms_mtc')
    
    # Configuration files
    mtc_config = os.path.join(dual_arms_mtc_dir, 'config', 'hybrid_mtc_config.yaml')
    
    # ================================================================
    # LOAD MOVEIT CONFIGURATIONS FOR MTC
    # ================================================================
    moveit_config = (
        MoveItConfigsBuilder("dual_arms", package_name="dual_arms")
        .robot_description(file_path="urdf/dual_arms_with_environment.xacro")
        .robot_description_semantic(file_path="config/dual_arms.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    return LaunchDescription([
        # ================================================================
        # CORE MTC CONTROLLER
        # ================================================================
        Node(
            package='dual_arms_mtc',
            executable='hybrid_mtc_controller',
            name='hybrid_mtc_controller',
            output='screen',
            parameters=[
                mtc_config,
                moveit_config.to_dict(),  # <--- INJECTS OMPL & KINEMATICS HERE
                {'use_sim_time': True}
            ],
            remappings=[
                ('ar4_controller/execute_task', '/ar4_controller/execute_task'),
                ('abb_controller/execute_task', '/abb_controller/execute_task'),
                ('ar4_gripper/set', '/ar4_gripper/set'),
                ('abb_gripper/set', '/abb_gripper/set'),
            ]
        ),
        
        # ================================================================
        # ZONE DETECTION MANAGER
        # ================================================================
        Node(
            package='dual_arms_mtc',
            executable='zone_detection_manager',
            name='zone_detection_manager',
            output='screen',
            parameters=[
                mtc_config,
                {'use_sim_time': True}
            ],
        ),
        
        # ================================================================
        # HYBRID SUPERVISOR (Python node)
        # ================================================================
        
        # Node(
        #     package='supervisor_package',
        #     executable='hybrid_supervisor_node',
        #     name='hybrid_supervisor',
        #     output='screen',
        #     parameters=[
        #         {'use_sim': True},
        #         {'enable_mtc_mode': True},
        #         {'handover_trigger_distance': 0.30},
        #     ]
        # ),
        
        # ================================================================
        # OPTIONAL: MoveIt Motion Planning Framework
        # ================================================================
        # Uncomment if you have separate MoveIt launch files
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(supervisor_dir, 'launch', 'moveit_planning.launch.py')
        #     )
        # ),
    ])
