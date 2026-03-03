"""
Launch file for the hybrid MTC controller

This launch file starts:
1. Hybrid MTC Controller (C++ node)
2. Zone Detection Manager (C++ node)
3. Hybrid Supervisor (Python node)
4. MoveIt motion planning framework

Usage:
  ros2 launch dual_arms_mtc hybrid_mtc_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    dual_arms_mtc_dir = get_package_share_directory('dual_arms_mtc')
    supervisor_dir = get_package_share_directory('supervisor_package')
    
    # Configuration files
    mtc_config = os.path.join(dual_arms_mtc_dir, 'config', 'hybrid_mtc_config.yaml')
    
    return LaunchDescription([
        # ================================================================
        # CORE MTC CONTROLLER AND ZONE DETECTION
        # ================================================================
        
        Node(
            package='dual_arms_mtc',
            executable='hybrid_mtc_controller',
            name='hybrid_mtc_controller',
            output='screen',
            parameters=[mtc_config],
            remappings=[
                ('ar4_controller/execute_task', '/ar4_controller/execute_task'),
                ('abb_controller/execute_task', '/abb_controller/execute_task'),
                ('ar4_gripper/set', '/ar4_gripper/set'),
                ('abb_gripper/set', '/abb_gripper/set'),
            ]
        ),
        
        Node(
            package='dual_arms_mtc',
            executable='zone_detection_manager',
            name='zone_detection_manager',
            output='screen',
            parameters=[mtc_config],
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
