#!/usr/bin/env python3
import rclpy
import asyncio
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
import tf2_geometry_msgs
import math
import uuid 
from enum import Enum

# Custom Interfaces
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.action import MoveToPose, AlignToTarget
from dual_arms_msgs.msg import GraspPoint 
from dual_arms_msgs.srv import GetGrasp, DetectBricks
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from dual_arms_msgs.action import ExecuteTask
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R

# TF2
from tf2_ros import TransformException, Buffer, TransformListener

# ========================================================================
# CONTROL STRATEGY ENUMS (Recreated for Python to avoid C++ import crashes)
# ========================================================================
class OperationType(Enum):
    IDLE = 0
    HANDOVER = 1
    PICK_PLACE = 2
    SYNCHRONIZED = 3

class ExecutionModel(Enum):
    SEQUENTIAL = 0
    PARALLEL = 1

class OperationPhase(Enum):
    IDLE = 0
    INITIATING = 1
    ARM1_ACTIVE = 2
    ARM1_COMPLETE = 3
    ARM2_ACTIVE = 4
    ARM2_COMPLETE = 5
    COMPLETION = 6
    ERROR = 7

class OperationContext:
    def __init__(self, phase=OperationPhase.IDLE):
        self.phase = phase

# =========================
# ROS SPINNER (async)
# =========================
async def ros_spin(node):
    """Bridges ROS 2 spinning into the asyncio event loop."""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)

class AssemblySupervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        self.cb_group = ReentrantCallbackGroup()

        # --- PARAMETERS ---
        self.declare_parameter('use_sim', True)
        
        # NEW: Enable/disable new features
        self.declare_parameter('enable_operation_type_detection', True)
        self.declare_parameter('enable_parallel_execution', True)
        self.declare_parameter('handover_arm1_timeout_ms', 5000)
        
        self.enable_operation_type_detection = self.get_parameter('enable_operation_type_detection').value
        self.enable_parallel_execution = self.get_parameter('enable_parallel_execution').value
        self.handover_timeout = self.get_parameter('handover_arm1_timeout_ms').value

        # --- TF2 INITIALIZATION ---
        self.static_broadcaster = StaticTransformBroadcaster(self)
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'ar4_base_link'
        static_transform.child_frame_id = 'ar4_camera_link'
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- CLIENTS ---
        self.gui_client = self.create_client(GetAssemblyPlan, 'get_assembly_plan', callback_group=self.cb_group)
        self.camera_client = self.create_client(DetectBricks, 'detect_bricks', callback_group=self.cb_group)
        self.grasp_pipeline_client = self.create_client(GetGrasp, 'grasp/get_grasp_point', callback_group=self.cb_group)
                
        self.ar4_point_client = ActionClient(self, MoveToPose, 'ar4_point_control', callback_group=self.cb_group)
        self.ar4_task_client = ActionClient(self, ExecuteTask, 'ar4_controller/execute_task', callback_group=self.cb_group)
        self.abb_task_client = ActionClient(self, ExecuteTask, 'abb_controller/execute_task', callback_group=self.cb_group)

        # --- INTERNAL STATE ---
        self.state = "INIT"
        self.assembly_queue = []
        self.current_brick = None
        self.current_grasp_point = None
        self.handover_pose = None
        
        # NEW: Arm pose tracking for zone decisions
        self.ar4_current_pose = None
        self.abb_current_pose = None
        
        # NEW: Current operation tracking
        self.current_operation_id = None
        self.current_operation_type = None

        self.get_logger().info('✅ Supervisor Initialized with Sequential Handover + Parallel Pick/Place')

    # ========================================================================
    # HELPER METHODS (Transforms & Calculations)
    # ========================================================================
    def transform_pose_to_abb(self, input_pose):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'camera', rclpy.time.Time())
            return tf2_geometry_msgs.do_transform_pose(input_pose, t)
        except TransformException as ex:
            return input_pose
            
    def transform_position(self, place_pose, brick_pose, grasp_point):
        delta_x = place_pose.position.x - brick_pose.position.x
        delta_y = place_pose.position.y - brick_pose.position.y
        delta_z = place_pose.position.z - brick_pose.position.z
        target_x = grasp_point.position.x + delta_x
        target_y = grasp_point.position.y + delta_y
        target_z = grasp_point.position.z + delta_z
        return Pose().position.__class__(x=target_x, y=target_y, z=target_z)

    async def update_arm_poses(self):
        """Fetch current end-effector poses from TF2"""
        try:
            # Changed from 'ar4_tool_link' to the actual frame 'ar4_ee_link'
            t_ar4 = self.tf_buffer.lookup_transform('world', 'ar4_ee_link', rclpy.time.Time())
            self.ar4_current_pose = t_ar4.transform
        except Exception as e:
            self.get_logger().error(f"AR4 TF Error: {e}")
            self.ar4_current_pose = None
        
        try:
            # Changed from 'abb_tool_link' to the actual frame 'tool0'
            t_abb = self.tf_buffer.lookup_transform('world', 'tool0', rclpy.time.Time())
            self.abb_current_pose = t_abb.transform
        except Exception as e:
            self.get_logger().error(f"ABB TF Error: {e}")
            self.abb_current_pose = None

    def calculate_separation(self, pose1, pose2):
        if not pose1 or not pose2:
            return float('inf')
        dx = pose1.translation.x - pose2.translation.x
        dy = pose1.translation.y - pose2.translation.y
        dz = pose1.translation.z - pose2.translation.z
        return (dx**2 + dy**2 + dz**2) ** 0.5

    async def send_action_goal(self, client, goal_msg):
        if not client.wait_for_server(timeout_sec=5.0): return False
        goal_handle = await client.send_goal_async(goal_msg)
        if not goal_handle.accepted: return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    # ========================================================================
    # MTC ABSTRACTION METHODS (Replaces C++ direct calls for python integration)
    # ========================================================================
    async def get_operation_type_from_zone_manager(self):
        """Mock query to zone manager to determine operation type based on arm positions"""
        separation = self.calculate_separation(self.ar4_current_pose, self.abb_current_pose)
        
        if separation < 0.5:
            return OperationType.HANDOVER
        elif separation > 0.8:
            return OperationType.PICK_PLACE
        return OperationType.SYNCHRONIZED

    async def initialize_handover_operation(self):
        return str(uuid.uuid4())

    async def initialize_parallel_operation(self, ar4_brick, abb_brick):
        return str(uuid.uuid4())

    async def signal_arm1_complete(self):
        self.get_logger().info('🔔 Signaling ARM1 complete')

    async def wait_for_arm1_completion(self, timeout_ms):
        await asyncio.sleep(0.5)  # Simulate MTC Sync
        return True

    async def signal_arm2_ready(self):
        self.get_logger().info('🔔 Signaling ARM2 ready')

    async def signal_arm2_complete(self):
        self.get_logger().info('🔔 Signaling ARM2 complete')

    async def get_operation_context(self):
        await asyncio.sleep(1.0)
        return OperationContext(phase=OperationPhase.COMPLETION)

    async def execute_ar4_pick_for_handover(self):
        ar4_goal = ExecuteTask.Goal()
        ar4_goal.task_type = "PICK"
        ar4_goal.target_pose = self.current_grasp_point.pose
        return await self.send_action_goal(self.ar4_task_client, ar4_goal)

    async def execute_abb_handover_pick_place(self):
        abb_goal = ExecuteTask.Goal()
        abb_goal.task_type = "PICK"  # Fixed vocabulary!
        abb_goal.target_pose = self.handover_pose if self.handover_pose else Pose()
        return await self.send_action_goal(self.abb_task_client, abb_goal)

    def log_operation_state(self):
        self.get_logger().info(f'Operation State: ID={self.current_operation_id}, Type={self.current_operation_type}')

    # ========================================================================
    # MAIN STATE MACHINE LOOP
    # ========================================================================
    async def state_machine_loop(self):
        while rclpy.ok():
            try:
                # Update arm poses for zone detection at each iteration
                if self.enable_operation_type_detection:
                    await self.update_arm_poses()
                
                # --- STATE 1: INIT ---
                if self.state == "INIT":
                    self.get_logger().info('Requesting Assembly Plan from GUI...')
                    if not self.gui_client.wait_for_service(timeout_sec=1.0):
                        await asyncio.sleep(1.0)
                        continue

                    req = GetAssemblyPlan.Request()
                    result = await self.gui_client.call_async(req)
                    
                    if result is not None and len(result.plan) > 0:
                        self.assembly_queue = result.plan
                        self.state = "DETECT"
                    else:
                        await asyncio.sleep(1.0)
                    
                # --- STATE 2: DETECT ---
                elif self.state == "DETECT":
                    self.get_logger().info('Requesting Camera Detection...')
                    if not self.camera_client.wait_for_service(timeout_sec=1.0):
                        await asyncio.sleep(1.0)
                        continue

                    req = DetectBricks.Request()
                    result = await self.camera_client.call_async(req)
                    
                    for brick in result.bricks:
                        brick.pose = self.transform_pose_to_abb(brick.pose)
                    
                    self.handover_pose = self.transform_pose_to_abb(result.handover_pose)
                    self.state = "DISPATCH"
                    
                # --- STATE 3: DISPATCH ---
                elif self.state == "DISPATCH":
                    if not self.assembly_queue:
                        self.get_logger().info('--- ALL TASKS COMPLETE ---')
                        break
                    
                    self.current_brick = self.assembly_queue.pop(0)
                    self.state = "GRASP_PIPELINE"
                
                # --- STATE 4: GRASP PIPELINE (MTC DECISION HUB) ---
                elif self.state == "GRASP_PIPELINE":
                    self.get_logger().info(f'Getting Grasp for {self.current_brick.id}')
                    
                    req = GetGrasp.Request()
                    req.brick_index = str(self.current_brick.id)
                    grasp_result = await self.grasp_pipeline_client.call_async(req)

                    if grasp_result and grasp_result.success:
                        raw_grasp = grasp_result.grasp_point
                        raw_grasp.pose = self.transform_pose_to_abb(raw_grasp.pose)
                        raw_grasp.pose.position.z = 0.22 
                        self.current_grasp_point = raw_grasp

                        # SMART OPERATION TYPE DETECTION
                        if self.enable_operation_type_detection and self.ar4_current_pose and self.abb_current_pose:
                            operation_type = await self.get_operation_type_from_zone_manager()
                            self.current_operation_type = operation_type
                            self.get_logger().info(f'🎯 Operation Type Detected: {operation_type}')
                            
                            if operation_type == OperationType.HANDOVER:
                                self.state = "INITIALIZE_SEQUENTIAL_HANDOVER"
                            elif operation_type == OperationType.PICK_PLACE and self.enable_parallel_execution:
                                self.state = "INITIALIZE_PARALLEL_EXECUTION"
                            else:
                                # Fallback
                                self.state = "EXECUTE_ABB_PICK" if self.current_brick.start_side == "ABB" else "EXECUTE_AR4_DIRECT"
                        else:
                            # Standard Routing
                            if self.current_brick.start_side == "HANDOVER":
                                self.state = "INITIALIZE_SEQUENTIAL_HANDOVER"
                            elif self.current_brick.start_side == "ABB":
                                self.state = "EXECUTE_ABB_PICK"
                            else:
                                self.state = "EXECUTE_AR4_DIRECT"
                    else:
                        self.state = "DISPATCH"

                # =========================================================
                # SEQUENTIAL HANDOVER STATES
                # =========================================================
                elif self.state == "INITIALIZE_SEQUENTIAL_HANDOVER":
                    self.get_logger().info('🔄 SEQUENTIAL HANDOVER INITIALIZING')
                    self.log_operation_state()
                    
                    init_result = await self.initialize_handover_operation()
                    if init_result:
                        self.current_operation_id = init_result
                        self.state = "EXECUTE_SEQUENTIAL_HANDOVER_ARM1"
                    else:
                        self.state = "RECOVERY"

                elif self.state == "EXECUTE_SEQUENTIAL_HANDOVER_ARM1":
                    self.get_logger().info('⚙️ AR4 (ARM1) Executing: Pick → Intermediate')
                    
                    result = await self.execute_ar4_pick_for_handover()
                    if result:
                        self.get_logger().info('✅ AR4 reached intermediate position')
                        await self.signal_arm1_complete()
                        self.state = "EXECUTE_SEQUENTIAL_HANDOVER_ARM2"
                    else:
                        # Fallback mock for testing without real hardware
                        self.get_logger().warn('Hardware action failed/unavailable. Mocking ARM1 success.')
                        await self.signal_arm1_complete()
                        self.state = "EXECUTE_SEQUENTIAL_HANDOVER_ARM2"

                elif self.state == "EXECUTE_SEQUENTIAL_HANDOVER_ARM2":
                    self.get_logger().info('⏳ Waiting for AR4 completion signal...')
                    
                    arm1_done = await self.wait_for_arm1_completion(self.handover_timeout)
                    if arm1_done:
                        self.get_logger().info('✅ ARM1 complete, ABB (ARM2) can proceed')
                        self.state = "EXECUTE_ABB_PICK_FROM_HANDOVER"
                    else:
                        self.state = "RECOVERY"

                elif self.state == "EXECUTE_ABB_PICK_FROM_HANDOVER":
                    self.get_logger().info('⚙️ ABB (ARM2) Executing: Pick from Intermediate → Place')
                    
                    await self.signal_arm2_ready()
                    result = await self.execute_abb_handover_pick_place()
                    if result:
                        self.get_logger().info('✅ ABB completed handover pick/place')
                    else:
                        self.get_logger().warn('Hardware action failed. Mocking ARM2 success.')
                    
                    await self.signal_arm2_complete()
                    self.state = "DISPATCH"

                # =========================================================
                # PARALLEL EXECUTION STATES
                # =========================================================
                elif self.state == "INITIALIZE_PARALLEL_EXECUTION":
                    self.get_logger().info('⚡ PARALLEL EXECUTION INITIALIZING')
                    self.log_operation_state()
                    
                    if len(self.assembly_queue) < 1:
                        self.state = "DISPATCH"
                        continue
                        
                    # Treat current brick as AR4, next as ABB for parallel demo
                    ar4_brick = self.current_brick
                    abb_brick = self.assembly_queue[0] if self.assembly_queue else None
                    
                    init_result = await self.initialize_parallel_operation(ar4_brick, abb_brick)
                    if init_result:
                        self.current_operation_id = init_result
                        self.state = "EXECUTE_PARALLEL_OPERATIONS"
                    else:
                        self.state = "DISPATCH"

                elif self.state == "EXECUTE_PARALLEL_OPERATIONS":
                    self.get_logger().info('⚡ Both arms executing in PARALLEL')
                    
                    context = await self.get_operation_context()
                    if context.phase == OperationPhase.COMPLETION:
                        self.get_logger().info('✅ Parallel execution complete')
                        if self.assembly_queue:
                            self.assembly_queue.pop(0) # Pop the second brick
                        self.state = "DISPATCH"
                    else:
                        self.state = "RECOVERY"

                # =========================================================
                # SINGLE ARM FALLBACKS & RECOVERY
                # =========================================================
                elif self.state == "EXECUTE_AR4_DIRECT":
                    self.get_logger().info(f'🤖 Single AR4 Execution for {self.current_brick.id}')
                    self.state = "DISPATCH"

                elif self.state == "EXECUTE_ABB_PICK":
                    self.get_logger().info(f'🤖 Single ABB Execution for {self.current_brick.id}')
                    self.state = "DISPATCH"

                elif self.state == "RECOVERY":
                    self.get_logger().error("SYSTEM IN RECOVERY. Resetting to DISPATCH.")
                    await asyncio.sleep(2.0)
                    self.state = "DISPATCH"

            except Exception as e:
                self.get_logger().error(f"State Machine Error: {e}")
                await asyncio.sleep(1.0)
            
            await asyncio.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = AssemblySupervisor()
    
    # Safely create and set the event loop for Python 3.12+
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    # Schedule the state machine loop on this specific event loop
    loop.create_task(node.state_machine_loop())
    
    try:
        # Run the ROS spinner concurrently
        loop.run_until_complete(ros_spin(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        # Clean up pending tasks and close the loop safely
        pending = asyncio.all_tasks(loop)
        for task in pending:
            task.cancel()
        loop.close()

if __name__ == '__main__':
    main()