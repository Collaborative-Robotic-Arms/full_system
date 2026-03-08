#!/usr/bin/env python3
"""
Hybrid Supervisor with MTC Integration

This enhanced supervisor adds:
1. Zone detection for handover areas
2. Dynamic switching between multithreaded and MTC control modes
3. MTC-based collaborative handover execution
4. True Parallel Execution using native rclpy timers
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
from std_msgs.msg import String


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
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)

        # --- INTERNAL STATE ---
        self.state = "INIT"
        self.current_brick = None
        self.assembly_queue = []
        self.detected_bricks = []
        self.current_grasp_point = None
        self.handover_pose = None
        self.ar4_busy = False
        self.abb_busy = False
        
        # Handover-specific state
        self.operation_type = None
        self.intermediate_pose = None
        self.abb_grasp_point_for_handover = None
        self.ar4_current_pose = None
        self.abb_current_pose = None
        
        # Parallel Execution Trackers
        self.ar4_parallel_done = False
        self.abb_parallel_done = False
        self.ar4_timer = None
        self.abb_timer = None

        # MTC state tracking
        self.control_mode = "MULTITHREADED"  # MULTITHREADED or MTC_HANDOVER
        self.in_handover_zone = False
        self.mtc_task_id = None

        # --- E-STOP CONFIG ---
        self.emergency_stop = False
        self.ar4_active_goal_handle = None
        self.abb_active_goal_handle = None
        self.zone_sub = self.create_subscription(
            String,
            '/zone_status',
            self.zone_status_callback,
            10,
            callback_group=self.cb_group
        )

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
    # HELPER METHODS FOR HANDOVER DECISIONS
    # ========================================================================
    
    def is_handover_operation(self, brick):
        if brick.start_side == brick.target_side:
            return False
        if brick.start_side not in ["AR4", "ABB"] or brick.target_side not in ["AR4", "ABB"]:
            return False
        return True

    def calculate_intermediate_pose(self, ar4_position, abb_target, height_offset=0.1):
        intermediate = Pose()
        intermediate.position.x = (ar4_position.position.x * 0.3 + abb_target.position.x * 0.7)
        intermediate.position.y = (ar4_position.position.y + abb_target.position.y) / 2.0
        intermediate.position.z = max(ar4_position.position.z, abb_target.position.z) + height_offset
        intermediate.orientation = ar4_position.orientation
        
        self.get_logger().info(
            f'Intermediate pose calculated: x={intermediate.position.x:.3f}, '
            f'y={intermediate.position.y:.3f}, z={intermediate.position.z:.3f}'
        )
        return intermediate

    def transform_stamped_to_pose(self, transform_stamped):
        pose = Pose()
        pose.position.x = transform_stamped.transform.translation.x
        pose.position.y = transform_stamped.transform.translation.y
        pose.position.z = transform_stamped.transform.translation.z
        pose.orientation = transform_stamped.transform.rotation
        return pose

    async def get_current_arm_poses(self):
        try:
            t_ar4 = self.tf_buffer.lookup_transform('world', 'ar4_ee_link', rclpy.time.Time())
            self.ar4_current_pose = self.transform_stamped_to_pose(t_ar4)
            
            t_abb = self.tf_buffer.lookup_transform('world', 'tool0', rclpy.time.Time())
            self.abb_current_pose = self.transform_stamped_to_pose(t_abb)
            return True
        except Exception as e:
            self.get_logger().warn(f'Could not get arm poses: {e}')
            return False

    # ========================================================================
    # PARALLEL EXECUTION HELPER SEQUENCES
    # ========================================================================

    async def execute_ar4_full_sequence(self, brick):
        """Standalone async sequence for true AR4 parallel execution"""
        self.get_logger().info(f'[PARALLEL] AR4 starting sequence for brick {brick.id}')
        
        req = GetGrasp.Request()
        req.brick_index = str(brick.id)
        res = await self.grasp_pipeline_client.call_async(req)
        if not res.success: return False
        
        grasp = res.grasp_point
        grasp.pose = self.transform_pose_to_abb(grasp.pose)
        grasp.pose.position.z = 0.22 
        
        await self.set_ar4_gripper(True)
        app_goal = MoveToPose.Goal()
        app_goal.target_pose = grasp.pose
        app_goal.strategy = "APPROACH_OFFSET"
        await self.send_action_goal(self.ar4_point_client, app_goal)
        
        await self.set_ar4_gripper(False)
        grs_goal = MoveToPose.Goal()
        grs_goal.target_pose = grasp.pose
        grs_goal.strategy = "GRASP"
        await self.send_action_goal(self.ar4_point_client, grs_goal)
        
        plc_goal = MoveToPose.Goal()
        plc_goal.target_pose = brick.place_pose
        plc_goal.target_pose.position.z = 0.26 
        plc_goal.strategy = "PLACE"
        await self.send_action_goal(self.ar4_point_client, plc_goal)
        
        retract_goal = MoveToPose.Goal()
        retract_goal.strategy = "HOME"
        await self.send_action_goal(self.ar4_point_client, retract_goal)

        self.get_logger().info(f'[PARALLEL] AR4 finished brick {brick.id}')
        return True

    async def execute_abb_full_sequence(self, brick):
        """Standalone async sequence for true ABB parallel execution"""
        self.get_logger().info(f'[PARALLEL] ABB starting sequence for brick {brick.id}')
        
        pick_goal = ExecuteTask.Goal()
        pick_goal.task_type = "PICK"
        pick_goal.target_pose = brick.pickup_pose 
        await self.send_action_goal(self.abb_client, pick_goal)
        
        plc_goal = ExecuteTask.Goal()
        plc_goal.task_type = "PLACE"
        plc_goal.target_pose = brick.place_pose
        plc_goal.target_pose.position.z = 0.24 
        await self.send_action_goal(self.abb_client, plc_goal)
        
        self.get_logger().info(f'[PARALLEL] ABB finished brick {brick.id}')
        return True
    async def execute_ar4_worker(self, brick):
        """Dedicated sequence for AR4 arm - runs independently of ABB"""
        self.get_logger().info(f'[AR4 WORKER] Starting sequence for brick {brick.id}')
        
        # Grasp Pipeline
        req = GetGrasp.Request()
        req.brick_index = str(brick.id)
        res = await self.grasp_pipeline_client.call_async(req)
        if not res or not res.success:
            self.ar4_busy = False
            return
        
        grasp = res.grasp_point
        grasp.pose = self.transform_pose_to_abb(grasp.pose)
        grasp.pose.position.z = 0.22 
        
        # Pick & Place
        await self.set_ar4_gripper(True)
        goal = MoveToPose.Goal(target_pose=grasp.pose, strategy="APPROACH_OFFSET")
        await self.send_action_goal(self.ar4_point_client, goal)
        
        await self.set_ar4_gripper(False)
        goal.strategy = "GRASP"
        await self.send_action_goal(self.ar4_point_client, goal)
        
        plc_goal = MoveToPose.Goal(target_pose=brick.place_pose, strategy="PLACE")
        plc_goal.target_pose.position.z = 0.22 
        await self.send_action_goal(self.ar4_point_client, plc_goal)
        
        # Retract
        await self.send_action_goal(self.ar4_point_client, MoveToPose.Goal(strategy="HOME"))

        self.ar4_busy = False # Signal that AR4 is free
        self.get_logger().info(f'[AR4 WORKER] Brick {brick.id} complete.')

    async def execute_abb_worker(self, brick):
        """Dedicated sequence for ABB arm - runs independently of AR4"""
        self.get_logger().info(f'[ABB WORKER] Starting sequence for brick {brick.id}')
        
        await self.send_action_goal(self.abb_client, ExecuteTask.Goal(task_type="PICK", target_pose=brick.pickup_pose))
        
        plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=brick.place_pose)
        plc_goal.target_pose.position.z = 0.24 
        await self.send_action_goal(self.abb_client, plc_goal)
        
        self.abb_busy = False # Signal that ABB is free
        self.get_logger().info(f'[ABB WORKER] Brick {brick.id} complete.')

    # ========================================================================
    # SUPERVISOR STATE MACHINE WITH MTC INTEGRATION
    # ========================================================================

    def zone_status_callback(self, msg):
        """Listens to the C++ Zone Manager and triggers emergency stop"""
        if "COLLISION_WARNING" in msg.data and not self.emergency_stop:
            self.get_logger().error("🚨 ZONE MANAGER DETECTED COLLISION RISK! TRIGGERING E-STOP! 🚨")
            self.emergency_stop = True
            self.state = "EMERGENCY_STOP"
            
            # Immediately lock both workers
            self.ar4_busy = True
            self.abb_busy = True

            # --- 🛑 INSTANTLY CANCEL ACTIVE MOTIONS 🛑 ---
            if self.ar4_active_goal_handle is not None:
                self.get_logger().error("🛑 Sending CANCEL request to AR4 Action Server!")
                # Call directly, do not use create_task
                self.ar4_active_goal_handle.cancel_goal_async() 
            
            if self.abb_active_goal_handle is not None:
                self.get_logger().error("🛑 Sending CANCEL request to ABB Action Server!")
                # Call directly, do not use create_task
                self.abb_active_goal_handle.cancel_goal_async()
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
                if not self.assembly_queue and not self.ar4_busy and not self.abb_busy:
                    self.get_logger().info('✅ All assembly tasks complete!')
                    self.state = "DONE"
                    return

                # --- AR4 DISPATCHER ---
                if not self.ar4_busy:
                    ar4_brick = next((b for b in self.assembly_queue if b.start_side == "AR4"), None)
                    if ar4_brick:
                        if self.is_handover_operation(ar4_brick):
                            self.assembly_queue.remove(ar4_brick)
                            self.current_brick = ar4_brick
                            self.state = "GRASP_PIPELINE"
                            return
                        else:
                            self.assembly_queue.remove(ar4_brick)
                            self.ar4_busy = True
                            # Use a non-lambda wrapper to ensure it is awaited
                            def ar4_callback():
                                self.ar4_timer.cancel()
                                self.executor.create_task(self.execute_ar4_worker(ar4_brick))
                            self.ar4_timer = self.create_timer(0.01, ar4_callback, callback_group=self.cb_group)

                # --- DISPATCH TO ABB ---
                if not self.abb_busy:
                    abb_brick = next((b for b in self.assembly_queue if b.start_side == "ABB"), None)
                    if abb_brick:
                        if self.is_handover_operation(abb_brick):
                            if not self.ar4_busy:
                                self.assembly_queue.remove(abb_brick)
                                self.current_brick = abb_brick
                                self.state = "GRASP_PIPELINE"
                                return
                        else:
                            self.assembly_queue.remove(abb_brick)
                            self.abb_busy = True
                            # Use a non-lambda wrapper to ensure it is awaited
                            def abb_callback():
                                self.abb_timer.cancel()
                                self.executor.create_task(self.execute_abb_worker(abb_brick))
                            self.abb_timer = self.create_timer(0.01, abb_callback, callback_group=self.cb_group)

                self.state = "PROCESS_NEXT"

                # self.current_brick = self.assembly_queue.pop(0)
                # self.state = "GRASP_PIPELINE"

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
                    
                    if self.is_handover_operation(self.current_brick):
                        self.get_logger().info(
                            f'🔄 HANDOVER detected: {self.current_brick.start_side} → {self.current_brick.target_side}'
                        )
                        self.operation_type = "HANDOVER"
                        self.state = "AR4_PICK_FOR_HANDOVER"
                        
                    elif len(self.assembly_queue) > 0:
                        self.get_logger().info(f'⚡ Multiple bricks detected - checking for parallel execution')
                        
                        ar4_bricks = [b for b in self.assembly_queue if b.start_side == "AR4"]
                        abb_bricks = [b for b in self.assembly_queue if b.start_side == "ABB"]
                        
                        if ar4_bricks and abb_bricks:
                            self.operation_type = "PARALLEL"
                            self.state = "INITIALIZE_PARALLEL_EXECUTION"
                        else:
                            self.operation_type = "SEQUENTIAL"
                            self.state = self.current_brick.start_side  
                    else:
                        self.get_logger().info(f'📦 Sequential pick/place operation')
                        self.operation_type = "SEQUENTIAL"
                        self.state = self.current_brick.start_side  
                else:
                    self.get_logger().error(f'Failed to get grasp for brick {self.current_brick.id}')
                    self.state = "PROCESS_NEXT"

            # ================================================================
            # PARALLEL EXECUTION STATE (ROS 2 NATIVE FIX)
            # ================================================================
            elif self.state == "INITIALIZE_PARALLEL_EXECUTION":
                self.get_logger().info('🔄 Initializing TRUE parallel execution...')
                
                brick1 = self.current_brick
                partner_side = "ABB" if brick1.start_side == "AR4" else "AR4"
                brick2 = next((b for b in self.assembly_queue if b.start_side == partner_side), None)
                
                if brick1 and brick2:
                    self.assembly_queue.remove(brick2) 
                    
                    ar4_brick = brick1 if brick1.start_side == "AR4" else brick2
                    abb_brick = brick1 if brick1.start_side == "ABB" else brick2
                    
                    self.get_logger().info(f'⚡ TRUE PARALLEL: AR4 -> Brick {ar4_brick.id}, ABB -> Brick {abb_brick.id}')
                    
                    # Reset Done Flags
                    self.ar4_parallel_done = False
                    self.abb_parallel_done = False

                    # Create wrappers to handle the async execution and set flags
                    async def run_ar4():
                        self.ar4_timer.cancel() # Stop the timer from repeating
                        await self.execute_ar4_full_sequence(ar4_brick)
                        self.ar4_parallel_done = True
                        
                    async def run_abb():
                        self.abb_timer.cancel() # Stop the timer from repeating
                        await self.execute_abb_full_sequence(abb_brick)
                        self.abb_parallel_done = True
                        
                    # Spawn both tasks instantly as ROS 2 timers
                    self.ar4_timer = self.create_timer(0.01, run_ar4, callback_group=self.cb_group)
                    self.abb_timer = self.create_timer(0.01, run_abb, callback_group=self.cb_group)
                    
                    # Transition to a waiting state
                    self.state = "WAIT_FOR_PARALLEL"
                else:
                    self.get_logger().warn('Partner brick not found. Falling back to sequential.')
                    if self.current_brick.start_side == "AR4":
                        self.state = "EXECUTE_AR4_DIRECT"
                    else:
                        self.state = "EXECUTE_ABB_PICK"

            elif self.state == "WAIT_FOR_PARALLEL":
                # Constantly check if both arms have finished their tasks
                if self.ar4_parallel_done and self.abb_parallel_done:
                    self.get_logger().info('✅ TRUE PARALLEL EXECUTION COMPLETE!')
                    self.state = "PROCESS_NEXT"

            # ================================================================
            # MTC-BASED HANDOVER STATE
            # ================================================================
            elif self.state == "MTC_HANDOVER_EXECUTION":
                self.get_logger().info('Executing MTC-based collaborative handover...')

                if not self.mtc_handover_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error('MTC handover service not available! Falling back to standard mode.')
                    await self.switch_control_mode("MULTITHREADED")
                    self.state = "HANDOVER_SEQUENCE"
                    return

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
                    await self.switch_control_mode("MULTITHREADED")
                    self.state = "HANDOVER_SEQUENCE"

            # ================================================================
            # HANDOVER SEQUENCE STATES
            # ================================================================

            elif self.state == "AR4_PICK_FOR_HANDOVER":
                self.get_logger().info(f'[HANDOVER Phase 1] AR4 picking brick {self.current_brick.id}')
                if not await self.set_ar4_gripper(True): return
                
                approach_goal = MoveToPose.Goal()
                approach_goal.target_pose = self.current_grasp_point.pose
                approach_goal.strategy = "APPROACH_OFFSET"
                action_result = await self.send_action_goal(self.ar4_point_client, approach_goal)
                if not action_result or not action_result.success: return
                
                grasp_goal = MoveToPose.Goal()
                grasp_goal.target_pose = self.current_grasp_point.pose
                grasp_goal.strategy = "GRASP"
                action_result = await self.send_action_goal(self.ar4_point_client, grasp_goal)
                if not action_result or not action_result.success: return
                
                if not await self.set_ar4_gripper(False): return
                
                self.get_logger().info('✓ AR4 picked successfully')
                self.state = "AR4_MOVE_TO_INTERMEDIATE"

            elif self.state == "AR4_MOVE_TO_INTERMEDIATE":
                self.get_logger().info(f'[HANDOVER Phase 2] AR4 moving to intermediate position')
                if not await self.get_current_arm_poses(): return
                
                self.intermediate_pose = self.calculate_intermediate_pose(
                    self.current_grasp_point.pose,
                    self.current_brick.place_pose
                )
                
                move_goal = MoveToPose.Goal()
                move_goal.target_pose = self.intermediate_pose
                move_goal.strategy = "MOVE"
                action_result = await self.send_action_goal(self.ar4_point_client, move_goal)
                if not action_result or not action_result.success: return
                
                self.get_logger().info('✓ AR4 at intermediate position')
                self.state = "REQUEST_ABB_GRASP_POINT"

            elif self.state == "REQUEST_ABB_GRASP_POINT":
                self.get_logger().info(f'[HANDOVER Phase 3] Requesting ABB grasp point from intermediate')
                if not self.grasp_pipeline_client.wait_for_service(timeout_sec=2.0): return
                
                grasp_req = GetGrasp.Request()
                grasp_req.brick_index = str(self.current_brick.id)
                grasp_result = await self.grasp_pipeline_client.call_async(grasp_req)
                
                if grasp_result.success:
                    abb_grasp_point = grasp_result.grasp_point
                    abb_grasp_point.pose = self.transform_pose_to_abb(abb_grasp_point.pose)
                    self.abb_grasp_point_for_handover = abb_grasp_point
                    self.state = "ABB_PICK_FROM_HANDOVER"
                else: return

            elif self.state == "ABB_PICK_FROM_HANDOVER":
                self.get_logger().info(f'[HANDOVER Phase 4] ABB picking from intermediate')
                abb_pick_goal = ExecuteTask.Goal()
                abb_pick_goal.task_type = "PICK"
                abb_pick_goal.target_pose = self.abb_grasp_point_for_handover.pose
                action_result = await self.send_action_goal(self.abb_client, abb_pick_goal)
                if not action_result or not action_result.success: return
                
                self.get_logger().info('✓ ABB picked successfully')
                self.state = "AR4_RELEASE_AT_INTERMEDIATE"

            elif self.state == "AR4_RELEASE_AT_INTERMEDIATE":
                self.get_logger().info(f'[HANDOVER Phase 5] AR4 releasing brick')
                if not await self.set_ar4_gripper(True): return
                
                retract_goal = MoveToPose.Goal()
                retract_goal.strategy = "HOME"
                await self.send_action_goal(self.ar4_point_client, retract_goal)
                
                self.get_logger().info('✓ AR4 released and retracted')
                self.state = "ABB_PLACE_FROM_HANDOVER"

            elif self.state == "ABB_PLACE_FROM_HANDOVER":
                self.get_logger().info(f'[HANDOVER Phase 6] ABB placing brick at target')
                abb_place_goal = ExecuteTask.Goal()
                abb_place_goal.task_type = "PLACE"
                abb_place_goal.target_pose = self.current_brick.place_pose
                action_result = await self.send_action_goal(self.abb_client, abb_place_goal)
                if not action_result or not action_result.success: return
                
                self.get_logger().info(f'✅ HANDOVER COMPLETE: Brick {self.current_brick.id} successfully transferred!')
                self.state = "PROCESS_NEXT"

            # ================================================================
            # SEQUENTIAL FALLBACK STATES
            # ================================================================

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
                place_goal.target_pose.position.z = 0.22 
                place_goal.strategy = "PLACE"
                action_result = await self.send_action_goal(self.ar4_point_client, place_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return
                
                self.get_logger().info('Retracting AR4 to safe position...')
                retract_goal = MoveToPose.Goal()
                retract_goal.strategy = "HOME"
                await self.send_action_goal(self.ar4_point_client, retract_goal)

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

        # --- STORE GOAL HANDLE FOR CANCELLATION ---
        if client == self.ar4_point_client:
            self.ar4_active_goal_handle = goal_handle
        elif client == self.abb_client:
            self.abb_active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result = await result_future

        # --- CLEAR GOAL HANDLE ONCE FINISHED ---
        if client == self.ar4_point_client and self.ar4_active_goal_handle == goal_handle:
            self.ar4_active_goal_handle = None
        elif client == self.abb_client and self.abb_active_goal_handle == goal_handle:
            self.abb_active_goal_handle = None

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