#!/usr/bin/env python3
"""
Mock System Launch Configuration

This launch file starts all mock service providers:
- Mock Detection Node (for brick detection)
- Mock GUI Node (for assembly plan generation)
- Mock Grasping Pipeline Node (for grasp point calculation)

This allows testing the supervisor and other components without requiring:
- Real camera/vision system
- GUI interface
- Grasping ML model

Supports 4 test scenarios (via TEST_SCENARIO env variable):
1. Parallel execution - no collision
2. Parallel with collision risk
3. AR4 hands over to ABB
4. ABB hands over to AR4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import logging


def generate_launch_description():
    """Generate launch description with all mock nodes"""
    
    # Declare scenario argument
    test_scenario = LaunchConfiguration('test_scenario')
    
    declare_scenario = DeclareLaunchArgument(
        'test_scenario',
        default_value='1',
        description='Test scenario: 1=parallel no collision, 2=parallel with collision, 3=AR4→ABB handover, 4=ABB→AR4 handover'
    )
    
    return LaunchDescription([
        declare_scenario,
        
        # Mock Detection Node - provides /detect_bricks service
        Node(
            package='supervisor_package',
            executable='mock_detection_node',
            name='mock_detection_node',
            output='screen',
            additional_env={'TEST_SCENARIO': test_scenario},
            parameters=[]
        ),
        
        # Mock GUI Node - provides /get_assembly_plan service
        Node(
            package='supervisor_package',
            executable='mock_gui_node',
            name='mock_gui_node',
            output='screen',
            additional_env={'TEST_SCENARIO': test_scenario},
            parameters=[]
        ),
        
        # Mock Grasping Pipeline Node - provides /grasp/get_grasp_point service
        Node(
            package='supervisor_package',
            executable='mock_grasping_node',
            name='mock_grasping_node',
            output='screen',
            additional_env={'TEST_SCENARIO': test_scenario},
            parameters=[]
        ),
    ])
