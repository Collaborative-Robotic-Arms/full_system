#!/usr/bin/env python3
"""
Hybrid Supervisor with MTC Integration

This enhanced supervisor adds:
1. Zone detection for handover areas
2. Dynamic switching between multithreaded and MTC control modes
3. MTC-based collaborative handover execution
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import tf2_geometry_msgs
from action_msgs.msg import GoalStatus

# Custom Interfaces
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.action import MoveToPose, AlignToTarget
from dual_arms_msgs.msg import GraspPoint 
from dual_arms_msgs.srv import GetGrasp, DetectBricks, GetHandoverZone, ExecuteMTCHandover
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from dual_arms_msgs.action import ExecuteTask
from std_srvs.srv import SetBool

from tf2_ros import TransformException, Buffer, TransformListener
from scipy.spatial.transform import Rotation as R


class HybridAssemblySupervisor(Node):
    """
    Enhanced supervisor that switches between multithreaded and MTC control
    based on proximity to handover zones.
    """
    
    def __init__(self):
        super().__init__('hybrid_supervisor')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # --- PARAMETERS ---
        self.declare_parameter('use_sim', True)
        self.declare_parameter('enable_mtc_mode', True)
        self.declare_parameter('handover_trigger_distance', 0.3)  # Distance to trigger MTC
        
        use_sim_value = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.enable_mtc_mode = self.get_parameter('enable_mtc_mode').get_parameter_value().bool_value
        self.handover_trigger_distance = self.get_parameter('handover_trigger_distance').get_parameter_value().double_value

        # --- TF2 INITIALIZATION ---
        self.static_broadcaster = StaticTransformBroadcaster(self)
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'ar4_base_link'
        static_transform.child_frame_id = 'ar4_camera_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- SERVICE CLIENTS ---
        self.gui_client = self.create_client(GetAssemblyPlan, 'get_assembly_plan', callback_group=self.cb_group)
        self.camera_client = self.create_client(DetectBricks, 'detect_bricks', callback_group=self.cb_group)
        self.grasp_pipeline_client = self.create_client(GetGrasp, 'grasp/get_grasp_point', callback_group=self.cb_group)
        self.gripper_client = self.create_client(SetBool, 'ar4_gripper/set', callback_group=self.cb_group)
        
        # MTC-specific service
        self.zone_client = self.create_client(GetHandoverZone, 'zone_detection/get_handover_zone', callback_group=self.cb_group)
        self.mtc_handover_client = self.create_client(ExecuteMTCHandover, 'mtc_controller/execute_handover', callback_group=self.cb_group)

        # --- ACTION CLIENTS ---
        self.ar4_point_client = ActionClient(self, MoveToPose, 'ar4_point_control', callback_group=self.cb_group)
        self.ar4_vs_client = ActionClient(self, AlignToTarget, 'ar4_visual_servo', callback_group=self.cb_group)
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)

        # --- INTERNAL STATE ---
        self.state = "INIT"
        self.current_brick = None
        self.assembly_queue = []
        self.detected_bricks = []
        self.current_grasp_point = None
        self.handover_pose = None
        
        # MTC state tracking
        self.control_mode = "MULTITHREADED"  # MULTITHREADED or MTC_HANDOVER
        self.in_handover_zone = False
        self.mtc_task_id = None

        self.get_logger().info('Hybrid Assembly Supervisor Initialized')
        self.timer = self.create_timer(1.0, self.state_machine_loop, callback_group=self.cb_group)

    # ========================================================================
    # ZONE DETECTION AND MODE SWITCHING
    # ========================================================================

    def detect_handover_proximity(self, pose):
        """Check if pose is approaching handover zone"""
        if self.handover_pose is None:
            return False
            
        dx = pose.position.x - self.handover_pose.position.x
        dy = pose.position.y - self.handover_pose.position.y
        dz = pose.position.z - self.handover_pose.position.z
        
        distance = (dx**2 + dy**2 + dz**2)**0.5
        return distance <= self.handover_trigger_distance

    async def switch_control_mode(self, new_mode):
        """Switch between multithreaded and MTC control modes"""
        if self.control_mode != new_mode:
            old_mode = self.control_mode
            self.control_mode = new_mode
            self.get_logger().warn(f'Control mode switched: {old_mode} -> {new_mode}')
            
            if new_mode == "MTC_HANDOVER":
                self.get_logger().info('MTC handover mode activated for collaborative task')
            else:
                self.get_logger().info('Switched back to standard multithreaded control')

    # ========================================================================
    # SUPERVISOR STATE MACHINE WITH MTC INTEGRATION
    # ========================================================================

    async def state_machine_loop(self):
        self.timer.cancel()
        try:
            if self.state == "INIT":
                self.get_logger().info('Requesting Assembly Plan from GUI...')
                
                if not self.gui_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Waiting for GUI node...')
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

                req = GetAssemblyPlan.Request()
                result = await self.gui_client.call_async(req)
                
                if result is not None and len(result.plan) > 0:
                    self.assembly_queue = result.plan
                    self.get_logger().info(f'Plan received! {len(self.assembly_queue)} bricks to process.')
                    self.state = "DETECT"
                else:
                    self.get_logger().warn('Assembly plan is empty. Retrying...')
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

            elif self.state == "DETECT":
                self.get_logger().info('Requesting Camera Detection...')
                
                if not self.camera_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().error('Camera Detection service not available!')
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

                req = DetectBricks.Request()
                result = await self.camera_client.call_async(req)
                
                for brick in result.bricks:
                    brick.pose = self.transform_pose_to_abb(brick.pose)
                
                self.detected_bricks = result.bricks
                self.handover_pose = self.transform_pose_to_abb(result.handover_pose)
                
                self.state = "PROCESS_NEXT"

            elif self.state == "PROCESS_NEXT":
                if not self.assembly_queue:
                    self.get_logger().info('All tasks complete!')
                    self.state = "DONE"
                    return

                self.current_brick = self.assembly_queue.pop(0)
                self.state = "GRASP_PIPELINE"

            elif self.state == "GRASP_PIPELINE":
                self.get_logger().info(f'Getting grasp for Brick {self.current_brick.id}')

                if not self.grasp_pipeline_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error('Grasp Pipeline service not available!')
                    self.state = "PROCESS_NEXT"
                    return

                grasp_req = GetGrasp.Request()
                grasp_req.brick_index = str(self.current_brick.id)
                grasp_result = await self.grasp_pipeline_client.call_async(grasp_req)

                if grasp_result.success:
                    raw_grasp = grasp_result.grasp_point
                    raw_grasp.pose = self.transform_pose_to_abb(raw_grasp.pose)
                    raw_grasp.pose.position.z = 0.22
                    self.current_grasp_point = raw_grasp
                    
                    self.get_logger().info(f'Grasp retrieved. Quality: {self.current_grasp_point.quality}')
                    
                    # Route to correct handler based on start_side
                    if self.current_brick.start_side == "ABB":
                        self.state = "EXECUTE_ABB_PICK"
                    elif self.current_brick.start_side == "HANDOVER":
                        # Check if we should use MTC mode
                        if self.enable_mtc_mode and self.detect_handover_proximity(self.current_grasp_point.pose):
                            await self.switch_control_mode("MTC_HANDOVER")
                            self.state = "MTC_HANDOVER_EXECUTION"
                        else:
                            self.state = "HANDOVER_SEQUENCE"
                    elif self.current_brick.start_side == "AR4":
                        self.state = "EXECUTE_AR4_DIRECT"
                    else:
                        self.get_logger().error(f"Unknown start_side: {self.current_brick.start_side}")
                else:
                    self.get_logger().error(f'Failed to get grasp for brick {self.current_brick.id}')
                    self.state = "PROCESS_NEXT"

            # ================================================================
            # MTC-BASED HANDOVER STATE (NEW)
            # ================================================================
            elif self.state == "MTC_HANDOVER_EXECUTION":
                self.get_logger().info('Executing MTC-based collaborative handover...')

                if not self.mtc_handover_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error('MTC handover service not available! Falling back to standard mode.')
                    await self.switch_control_mode("MULTITHREADED")
                    self.state = "HANDOVER_SEQUENCE"
                    return

                # Create MTC handover request
                mtc_req = ExecuteMTCHandover.Request()
                mtc_req.ar4_start_pose = self.current_grasp_point.pose
                mtc_req.abb_start_pose = self.current_brick.pickup_pose
                mtc_req.handover_pose = self.handover_pose
                mtc_req.object_id = self.current_brick.type

                mtc_result = await self.mtc_handover_client.call_async(mtc_req)

                if mtc_result.success:
                    self.get_logger().info(f'MTC handover completed. Execution ID: {mtc_result.execution_id}')
                    self.mtc_task_id = mtc_result.execution_id
                    self.state = "PROCESS_NEXT"
                else:
                    self.get_logger().error(f'MTC handover failed: {mtc_result.status_message}')
                    # Fall back to standard handover
                    await self.switch_control_mode("MULTITHREADED")
                    self.state = "HANDOVER_SEQUENCE"

            # ================================================================
            # STANDARD MULTITHREADED HANDOVER STATE (FALLBACK)
            # ================================================================
            elif self.state == "HANDOVER_SEQUENCE":
                if self.control_mode != "MULTITHREADED":
                    await self.switch_control_mode("MULTITHREADED")
                
                self.state = "AR4_PICK_FOR_HANDOVER"

            # ... (rest of existing states: EXECUTE_AR4_DIRECT, AR4_PLACE_ON_GRID, EXECUTE_ABB_PICK, etc.)
            # Keep all existing state handlers here

            elif self.state == "AR4_PICK_FOR_HANDOVER":
                self.get_logger().info(f'Starting AR4 Pick for handover')
                await self.set_ar4_gripper(True)
                
                goal_msg = MoveToPose.Goal()
                goal_msg.target_pose = self.current_grasp_point.pose
                goal_msg.strategy = "APPROACH_OFFSET"
                action_result = await self.send_action_goal(self.ar4_point_client, goal_msg)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                await self.set_ar4_gripper(False)

                self.get_logger().info('Switching to Visual Servoing...')
                vs_goal = AlignToTarget.Goal()
                vs_goal.object_id = self.current_brick.type
                action_result = await self.send_action_goal(self.ar4_vs_client, vs_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                self.get_logger().info('Lowering and Grasping...')
                grasp_goal = MoveToPose.Goal()
                grasp_goal.target_pose = self.current_grasp_point.pose
                grasp_goal.strategy = "GRASP"
                action_result = await self.send_action_goal(self.ar4_point_client, grasp_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                self.state = "HANDOVER_EXECUTION"

            elif self.state == "HANDOVER_EXECUTION":
                self.get_logger().info('Starting handover sequence')

                goal = MoveToPose.Goal()
                goal.strategy = "GOTO_HANDOVER"
                action_result = await self.send_action_goal(self.ar4_point_client, goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                abb_goal = ExecuteTask.Goal()
                abb_goal.task_type = "PICK_FROM_HANDOVER"
                abb_goal.target_pose = self.handover_pose
                action_result = await self.send_action_goal(self.abb_client, abb_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                release_goal = MoveToPose.Goal()
                release_goal.strategy = "RELEASE"
                action_result = await self.send_action_goal(self.ar4_point_client, release_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                self.state = "PROCESS_NEXT"

            elif self.state == "EXECUTE_AR4_DIRECT":
                self.get_logger().info(f'AR4 Direct Pick for {self.current_brick.id}')
                await self.set_ar4_gripper(True)

                goal_msg = MoveToPose.Goal()
                goal_msg.target_pose = self.current_grasp_point.pose
                goal_msg.strategy = "APPROACH_OFFSET"
                action_result = await self.send_action_goal(self.ar4_point_client, goal_msg)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                await self.set_ar4_gripper(False)

                vs_goal = AlignToTarget.Goal()
                vs_goal.object_id = self.current_brick.type
                action_result = await self.send_action_goal(self.ar4_vs_client, vs_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                grasp_goal = MoveToPose.Goal()
                grasp_goal.target_pose = self.current_grasp_point.pose
                grasp_goal.strategy = "GRASP"
                action_result = await self.send_action_goal(self.ar4_point_client, grasp_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                self.state = "AR4_PLACE_ON_GRID"

            elif self.state == "AR4_PLACE_ON_GRID":
                place_goal = MoveToPose.Goal()
                place_goal.target_pose = self.current_brick.place_pose
                place_goal.strategy = "PLACE"
                action_result = await self.send_action_goal(self.ar4_point_client, place_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return
                self.state = "PROCESS_NEXT"

            elif self.state == "EXECUTE_ABB_PICK":
                self.get_logger().info(f'ABB Pick for {self.current_brick.id}')

                abb_pick_goal = ExecuteTask.Goal()
                abb_pick_goal.task_type = "PICK"
                abb_pick_goal.target_pose = self.current_grasp_point.pose if self.current_grasp_point else self.current_brick.pickup_pose

                action_result = await self.send_action_goal(self.abb_client, abb_pick_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return
                self.state = "EXECUTE_ABB_PLACE"

            elif self.state == "EXECUTE_ABB_PLACE":
                abb_place_goal = ExecuteTask.Goal()
                abb_place_goal.task_type = "PLACE"
                abb_place_goal.target_pose = self.current_brick.place_pose
                abb_place_goal.target_pose.position.z = 0.24

                action_result = await self.send_action_goal(self.abb_client, abb_place_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return
                self.state = "PROCESS_NEXT"

            elif self.state == "RECOVERY":
                self.get_logger().warn('Entering Recovery Mode')
                recovery_goal = MoveToPose.Goal()
                recovery_goal.strategy = "HOME"
                action_result = await self.send_action_goal(self.ar4_point_client, recovery_goal)

                if action_result and action_result.success:
                    self.get_logger().info('Recovery successful')
                    self.assembly_queue.insert(0, self.current_brick)
                    self.state = "PROCESS_NEXT"
                else:
                    self.get_logger().error("Recovery failed! Emergency stop.")
                    self.state = "EMERGENCY_STOP"

            elif self.state == "EMERGENCY_STOP":
                self.get_logger().error("SYSTEM HALTED")
                return

        except Exception as e:
            self.get_logger().error(f'State machine error: {e}')

        if self.state != "DONE":
            self.timer = self.create_timer(0.1, self.state_machine_loop, callback_group=self.cb_group)

    # ========================================================================
    # HELPER METHODS
    # ========================================================================

    def transform_pose_to_abb(self, input_pose):
        """Transform pose from camera frame to ABB base frame"""
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose, t)
            return transformed_pose
        except TransformException as ex:
            self.get_logger().error(f'TF2 Error: {ex}')
            return input_pose

    async def send_action_goal(self, client, goal_msg):
        """Send action goal and wait for result"""
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server not available')
            return None

        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected')
            return None

        result_future = goal_handle.get_result_async()
        result = await result_future

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return result.result
        else:
            self.get_logger().error(f'Action failed with status: {result.status}')
            return None

    async def set_ar4_gripper(self, open_gripper: bool):
        """Control AR4 gripper"""
        if not self.gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Gripper service not available')
            return False

        req = SetBool.Request()
        req.data = open_gripper
        result = await self.gripper_client.call_async(req)
        return result.success


def main(args=None):
    rclpy.init(args=args)
    node = HybridAssemblySupervisor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
