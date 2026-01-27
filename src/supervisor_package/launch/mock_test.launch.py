from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, LogInfo

def generate_launch_description():

    # 1. Mock AR4 Node (Simulates Gripper/Robot)
    mock_ar4_node = Node(
        package='supervisor_package',
        executable='mock_ar4',
        name='mock_ar4',
        output='screen'
    )

    # 2. Static Transform Publisher (Camera to Base)
    # Args: x y z yaw pitch roll parent_frame child_frame
    tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=['0', '0', '1', '0', '0', '0', 'base_link', 'camera_color_optical_frame'],
        output='screen'
    )

    # 3. Supervisor Node (The Brain)
    supervisor_node = Node(
        package='supervisor_package',
        executable='supervisor_node',
        name='supervisor',
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg=">>> Starting Mock Environment & TF..."),
        mock_ar4_node,
        tf_broadcaster,

        # Wait 3 seconds to ensure TF is available before starting the Supervisor
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg=">>> TF Ready. Starting Supervisor..."),
                supervisor_node
            ]
        )
    ])