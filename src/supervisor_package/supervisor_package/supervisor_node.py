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

# Custom Interfaces
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.action import MoveToPose, AlignToTarget
from dual_arms_msgs.msg import GraspPoint 
from dual_arms_msgs.srv import GetGrasp , DetectBricks
from geometry_msgs.msg import TransformStamped, Pose # Added Pose to imports
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from dual_arms_msgs.action import ExecuteTask
from supervisor_package.action import MoveToPose
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R

# TF2
from tf2_ros import TransformException, Buffer, TransformListener

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

        # --- GRIPPER INITIALIZATION ---
        self.declare_parameter('use_sim', True)
        use_sim_value = self.get_parameter('use_sim').get_parameter_value().bool_value
        # self.gripper = GripperManager(self, use_sim=use_sim_value)

        # --- TF2 INITIALIZATION ---
        # Broadcaster: Defines the static relationship between the robot base and camera lens
        self.static_broadcaster = StaticTransformBroadcaster(self)
        static_transform = TransformStamped()
        
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'ar4_base_link'      # Parent Frame
        static_transform.child_frame_id = 'ar4_camera_link' # Child Frame

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- CLIENTS ---
        self.gui_client = self.create_client(GetAssemblyPlan, 'get_assembly_plan', callback_group=self.cb_group)
        self.camera_client = self.create_client(DetectBricks, 'detect_bricks', callback_group=self.cb_group)
        self.grasp_client = ActionClient(self, MoveToPose, 'grasp_pipeline', callback_group=self.cb_group)
        self.grasp_pipeline_client = self.create_client(GetGrasp, 'grasp/get_grasp_point', callback_group=self.cb_group)
                
        self.ar4_task_client = ActionClient(self, ExecuteTask, 'ar4_controller/execute_task', callback_group=self.cb_group)
        self.abb_task_client = ActionClient(self, ExecuteTask, 'abb_controller/execute_task', callback_group=self.cb_group)

        # --- INTERNAL STATE ---
        self.state = "INIT"
        self.assembly_queue = []
        self.detected_bricks = [] # Array of dual_arms_msgs/Brick
        self.abb_busy = False
        self.ar4_busy = False
        self.active_tasks = 0
        self.current_brick = None
        self.current_grasp_point = None
        
        # --- NEW: Operation Type Detection & Tracking ---
        self.declare_parameter('enable_operation_type_detection', True)
        self.declare_parameter('enable_parallel_execution', True)
        self.declare_parameter('handover_timeout_ms', 5000)
        
        self.enable_operation_type_detection = \
            self.get_parameter('enable_operation_type_detection').value
        self.enable_parallel_execution = \
            self.get_parameter('enable_parallel_execution').value
        self.handover_timeout = \
            self.get_parameter('handover_timeout_ms').value
        
        # Arm pose tracking for zone decisions
        self.ar4_current_pose = None
        self.abb_current_pose = None
        
        # Current operation tracking
        self.current_operation_id = None
        self.current_operation_type = None
        self.arm1_completed = False
        self.arm2_ready = False

        self.get_logger().info('Supervisor Initialized with Parallel Mission Support.')

    # --- NEW HELPER METHOD ---
    def transform_pose_to_abb(self, input_pose):
        """Standard TF2 transformation from Camera frame to ABB base_link."""
        try:
            # Lookup the transformation broadcasted in __init__
            t = self.tf_buffer.lookup_transform(
                'base_link', 
                'camera', 
                rclpy.time.Time()) # Get the latest available transform

            # Use tf2_geometry_msgs to translate/rotate the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose, t)

            return transformed_pose

        except TransformException as ex:
            self.get_logger().error(f'TF2 Error: {ex}')
            return input_pose # Fallback to original pose
        
    def transform_quaternion(self, place_pose, brick_pose, grasp_point):

        # 1. Check for NoneType (Prevents the 'NoneType' error)
        if any(p is None for p in [place_pose, brick_pose, grasp_point]):
            self.get_logger().error("One of the input poses is None!")
            return None

        try:
            # Helper to ensure we don't have [0,0,0,0]
            def get_valid_quat(orient):
                if orient.z == 0.0 and orient.w == 0.0:
                    return [0, 0, 0, 1] # Return Identity (no rotation)
                return [0, 0, orient.z, orient.w]

            # 2. Convert to Euler using the helper
            r_place = R.from_quat(get_valid_quat(place_pose.orientation)).as_euler('xyz', degrees=True)[2]
            r_brick = R.from_quat(get_valid_quat(brick_pose.orientation)).as_euler('xyz', degrees=True)[2]
            r_grasp = R.from_quat(get_valid_quat(grasp_point.orientation)).as_euler('xyz', degrees=True)[2]
            self.get_logger().info(f"Euler Angles (deg) -> Place: {r_place:.2f}, Brick: {r_brick:.2f}, Grasp: {r_grasp:.2f}")
            # 3. Thesis logic
            target_yaw_deg = r_grasp + (r_place - r_brick)

            # 4. Convert back
            target_quat_array = R.from_euler('z', target_yaw_deg, degrees=True).as_quat()

            self.get_logger().info(f"Target Yaw (deg): {target_yaw_deg:.2f} => Quaternion(z={target_quat_array[2]:.3f}, w={target_quat_array[3]:.3f})")

            # 5. Map back to a ROS Quaternion object (Prevents attribute errors later)
            from geometry_msgs.msg import Quaternion
            target_quat = Quaternion()
            target_quat.x = 0.0
            target_quat.y = 0.0
            target_quat.z = target_quat_array[2]
            target_quat.w = target_quat_array[3]
            
            self.get_logger().info(f"Transformed Quaternion: x={target_quat.x:.3f}, y={target_quat.y:.3f}, z={target_quat.z:.3f}, w={target_quat.w:.3f}")
            return target_quat

        except ValueError as e:
            self.get_logger().error(f"Quaternion Math Error: {e}")
            return None
    
    def transform_position(self, place_pose, brick_pose, grasp_point):
      
        # Simple translation logic based on your thesis requirements
        delta_x = place_pose.position.x - brick_pose.position.x
        delta_y = place_pose.position.y - brick_pose.position.y
        delta_z = place_pose.position.z - brick_pose.position.z

        target_x = grasp_point.position.x + delta_x
        target_y = grasp_point.position.y + delta_y
        target_z = grasp_point.position.z + delta_z

        self.get_logger().info(f"Transformed Position: Δx({delta_x:.3f}), Δy({delta_y:.3f}), Δz({delta_z:.3f}) => Target(x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f})")
        return Pose().position.__class__(x=target_x, y=target_y, z=target_z)
   
    # -------------------------
    async def update_arm_poses(self):
        """
        Fetch current end-effector poses from TF2.
        Called at beginning of each state_machine_loop iteration.
        """
        try:
            # Get AR4 TCP pose
            t_ar4 = self.tf_buffer.lookup_transform(
                'world',
                'ar4_tool_link',
                rclpy.time.Time()
            )
            self.ar4_current_pose = t_ar4.transform
        except Exception as e:
            self.get_logger().debug(f'Could not get AR4 pose: {e}')
        
        try:
            # Get ABB TCP pose
            t_abb = self.tf_buffer.lookup_transform(
                'world',
                'abb_tool_link',
                rclpy.time.Time()
            )
            self.abb_current_pose = t_abb.transform
        except Exception as e:
            self.get_logger().debug(f'Could not get ABB pose: {e}')

    def calculate_separation(self, pose1, pose2):
        """Calculate distance between two poses"""
        if not pose1 or not pose2:
            return float('inf')
        
        dx = pose1.translation.x - pose2.translation.x
        dy = pose1.translation.y - pose2.translation.y
        dz = pose1.translation.z - pose2.translation.z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)

    async def determine_operation_type(self):
        """
        Determine operation type based on arm positions.
        Returns: 'HANDOVER', 'PARALLEL', or 'SEQUENTIAL'
        """
        if not self.ar4_current_pose or not self.abb_current_pose:
            return 'SEQUENTIAL'
        
        separation = self.calculate_separation(
            self.ar4_current_pose,
            self.abb_current_pose
        )
        
        # If in handover zone (arms close together)
        if separation < 0.5:
            self.get_logger().info(f'🔄 Handover zone detected (separation: {separation:.2f}m)')
            return 'HANDOVER'
        
        # If far apart (safe for parallel)
        if separation > 0.8:
            self.get_logger().info(f'⚡ Parallel safe zone (separation: {separation:.2f}m)')
            return 'PARALLEL'
        
        # Otherwise synchronized
        self.get_logger().info(f'📦 Synchronized zone (separation: {separation:.2f}m)')
        return 'SEQUENTIAL'

    # =========================
    # PARALLEL WORKER MISSIONS
    # =========================
    
    async def run_abb_mission(self, brick):
        """Sequential Pick & Place for ABB with complex transform logic."""
        self.abb_busy = True
        self.active_tasks += 1
        self.get_logger().info(f"ABB Worker: Starting mission for Brick {brick.id}")

        try:
            # 1. THINK: Get Grasp
            grasp_req = GetGrasp.Request()
            grasp_req.brick_index = str(brick.id)
            res = await self.grasp_pipeline_client.call_async(grasp_req)

            if res and res.success:
                # Extract the .pose 
                grasp_pose = self.transform_pose_to_abb(res.grasp_point.pose)
                grasp_pose.position.z = 0.22
                
                # --- STEP 1: PICK ---
                # Use ExecuteTask (Ensure you have this imported/aliased!)
                pick_goal = ExecuteTask.Goal() 
                pick_goal.task_type = "PICK"
                pick_goal.target_pose = grasp_pose
                
                if await self.send_action_goal(self.abb_task_client, pick_goal):
                    await asyncio.sleep(3.0)

                    # --- STEP 2: CALCULATE PLACE POSE ---
                    place_goal = ExecuteTask.Goal()
                    place_goal.task_type = "PLACE"

                    place_goal.target_pose.position = self.transform_position(
                        place_pose=brick.place_pose,
                        brick_pose=brick.pickup_pose,
                        grasp_point=grasp_pose  
                    )

                    # Orientation Transform
                    orientation = self.transform_quaternion(
                        place_pose=brick.place_pose,
                        brick_pose=brick.pickup_pose,
                        grasp_point=grasp_pose # <--- Changed here too
                    )
                    
                    if orientation:
                        # ABB-Specific Orientation Mapping
                        place_goal.target_pose.orientation.x = orientation.z
                        place_goal.target_pose.orientation.y = orientation.w
                        place_goal.target_pose.orientation.z = 0.0
                        place_goal.target_pose.orientation.w = 0.0
                    
                    # Height Adjustment
                    place_goal.target_pose.position.z = 0.23 

                    # --- STEP 3: PLACE ---
                    self.get_logger().info(f"ABB: Sending Place goal for {brick.id}")
                    action_result = await self.send_action_goal(self.abb_task_client, place_goal)
                    
                    if action_result:
                        self.get_logger().info(f"ABB Worker: Successfully placed Brick {brick.id}")
            else:
                self.get_logger().error(f"ABB: Grasp Service failed for {brick.id}")            
        except Exception as e:
            self.get_logger().error(f"ABB Worker Error during execution: {e}")
        finally:
            self.abb_busy = False
            self.active_tasks -= 1

    async def run_ar4_mission(self, brick):
        """Truly parallel mission for AR4."""
        self.ar4_busy = True
        self.active_tasks += 1
        self.get_logger().info(f"AR4: Starting mission for Brick {brick.id}")

        try:
            grasp_req_ar = GetGrasp.Request()
            grasp_req_ar.brick_index = str(brick.id)
            
            self.get_logger().info(f"AR4: Requesting Grasp for {brick.id}")
            grasp_result_ar = await self.grasp_pipeline_client.call_async(grasp_req_ar)

            if grasp_result_ar and grasp_result_ar.success:
                # Extract Pose
                grasp_pose_ar = self.transform_pose_to_abb(grasp_result_ar.grasp_point.pose)
                grasp_pose_ar.position.z = 0.14

                # 3. DO: PICK
                pick_goal_ar = ExecuteTask.Goal()
                pick_goal_ar.task_type = "PICK"
                pick_goal_ar.target_pose = grasp_pose_ar
                
                pick_success_ar = await self.send_action_goal(self.ar4_task_client, pick_goal_ar)
                await asyncio.sleep(3.0)
                
                # 4. DO: PLACE
                if pick_success_ar:
                    place_goal_ar = ExecuteTask.Goal()
                    place_goal_ar.task_type = "PLACE"
                    place_goal_ar.target_pose=brick.place_pose
                    
                    place_goal_ar.target_pose.position = self.transform_position(
                        place_pose=brick.place_pose,
                        brick_pose=brick.pickup_pose,
                        grasp_point=grasp_pose_ar  
                    )
                    # # Orientation Transform
                    # orientation = self.transform_quaternion(
                    #     place_pose=brick.place_pose,
                    #     brick_pose=brick.pickup_pose,
                    #     grasp_point=grasp_pose # <--- Changed here too
                    # )
                    
                    # if orientation:
                    #     place_goal_ar.target_pose.orientation = orientation
                    
                    # Height Adjustment
                    place_goal_ar.target_pose.position.z = 0.14
                    
                    await self.send_action_goal(self.ar4_task_client, place_goal_ar)
                    self.get_logger().info(f"AR4: Successfully completed Brick {brick.id}")
            else:
                self.get_logger().error(f"AR4: Grasp Service failed for {brick.id}")

        except Exception as e:
            self.get_logger().error(f"AR4 Mission Error: {e}")
        finally:
            self.ar4_busy = False
            self.active_tasks -= 1
            
    # =========================
    # MAIN LOOP (YOUR LOGIC)
    # =========================

    async def state_machine_loop(self):
        while rclpy.ok():
            try:
                # NEW: Update arm poses for zone detection
                if self.enable_operation_type_detection:
                    await self.update_arm_poses()
                
                # =========================
                # STATE 1: INIT → DETECTION
                # =========================
                if self.state == "INIT":
                    self.get_logger().info('Requesting Assembly Plan from GUI...')
                    
                    # Check service availability without blocking the executor indefinitely
                    if not self.gui_client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('Waiting for GUI node...')
                        self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                        return

                    req = GetAssemblyPlan.Request()
                    result = await self.gui_client.call_async(req)
                    
                    # --- FIX: Only proceed if the plan actually contains bricks ---
                    if result is not None and len(result.plan) > 0:
                        self.assembly_queue = result.plan
                        self.get_logger().info(f'Plan received! {len(self.assembly_queue)} bricks to process.')
                        self.state = "DETECT"
                    else:
                        self.get_logger().warn('Assembly plan is empty or service failed. Retrying in 2 seconds...')
                        # Keep state as "INIT" and retry after a delay
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
                    
                    # Transform each brick to the ABB base frame
                    for brick in result.bricks:
                        brick.pose = self.transform_pose_to_abb(brick.pose)
                    self.detected_bricks = result.bricks
                    
                    # Transform handover pose to the ABB base frame
                    self.handover_pose = self.transform_pose_to_abb(result.handover_pose)

                    self.state = "DISPATCH"
                    
                elif self.state == "DISPATCH":
                    if not self.assembly_queue and self.active_tasks == 0:
                        self.get_logger().info('--- ALL TASKS COMPLETE ---')
                        break

                    # NEW: Determine operation type
                    if self.enable_operation_type_detection and \
                       self.ar4_current_pose and self.abb_current_pose:
                        operation_type = await self.determine_operation_type()
                        self.current_operation_type = operation_type
                        
                        if operation_type == 'HANDOVER':
                            self.get_logger().info('🔄 SEQUENTIAL HANDOVER DETECTED')
                            self.state = "SEQUENTIAL_HANDOVER"
                            continue
                        elif operation_type == 'PARALLEL' and self.enable_parallel_execution:
                            # Check if we have bricks for both arms
                            if len(self.assembly_queue) >= 2:
                                self.get_logger().info('⚡ PARALLEL EXECUTION POSSIBLE')
                                self.state = "PARALLEL_EXECUTION"
                                continue
                    
                    # Default: dispatch as before
                    if not self.abb_busy:
                        for brick in list(self.assembly_queue):
                            if brick.start_side == "ABB":
                                self.abb_busy = True
                                self.assembly_queue.remove(brick)
                                asyncio.create_task(self.run_abb_mission(brick))
                                self.get_logger().info(f"Dispatched ABB for Brick {brick.id}")
                                break

                    if not self.ar4_busy:
                        for brick in list(self.assembly_queue):
                            if brick.start_side == "AR4":
                                self.ar4_busy = True
                                self.assembly_queue.remove(brick)
                                asyncio.create_task(self.run_ar4_mission(brick))
                                self.get_logger().info(f"Dispatched AR4 for Brick {brick.id}")
                                break
                
                # NEW: Sequential Handover State
                elif self.state == "SEQUENTIAL_HANDOVER":
                    self.get_logger().info('🔄 SEQUENTIAL HANDOVER: AR4 picks → intermediate')
                    
                    # For now, dispatch to traditional handover flow
                    # In future, would call MTC controller
                    if not self.ar4_busy:
                        for brick in list(self.assembly_queue):
                            if brick.start_side == "AR4":
                                self.ar4_busy = True
                                self.assembly_queue.remove(brick)
                                self.current_brick = brick
                                
                                # Get grasp first
                                grasp_req = GetGrasp.Request()
                                grasp_req.brick_index = str(brick.id)
                                grasp_res = await self.grasp_pipeline_client.call_async(grasp_req)
                                
                                if grasp_res and grasp_res.success:
                                    grasp_pose = self.transform_pose_to_abb(grasp_res.grasp_point.pose)
                                    grasp_pose.position.z = 0.14
                                    self.current_grasp_point = grasp_pose
                                    
                                    # AR4 picks
                                    asyncio.create_task(self.run_ar4_mission(brick))
                                    self.arm1_completed = True
                                    self.state = "HANDOVER_ABB_PICK"
                                break
                    else:
                        await asyncio.sleep(0.5)
                
                # NEW: Handover - ABB picks
                elif self.state == "HANDOVER_ABB_PICK":
                    if self.arm1_completed and not self.abb_busy:
                        self.get_logger().info('🔄 SEQUENTIAL HANDOVER: ABB picks from intermediate')
                        
                        if not self.assembly_queue and self.current_brick:
                            self.abb_busy = True
                            brick = self.current_brick
                            
                            # ABB picks from intermediate
                            abb_pick_goal = ExecuteTask.Goal()
                            abb_pick_goal.task_type = "PICK_FROM_HANDOVER"
                            abb_pick_goal.target_pose = self.current_grasp_point
                            
                            if await self.send_action_goal(self.abb_task_client, abb_pick_goal):
                                await asyncio.sleep(2.0)
                                
                                # ABB places
                                place_goal = ExecuteTask.Goal()
                                place_goal.task_type = "PLACE"
                                place_goal.target_pose = brick.place_pose
                                place_goal.target_pose.position.z = 0.23
                                
                                await self.send_action_goal(self.abb_task_client, place_goal)
                                self.get_logger().info('✅ Handover complete')
                            
                            self.abb_busy = False
                            self.arm1_completed = False
                            self.current_brick = None
                            self.state = "DISPATCH"
                    else:
                        await asyncio.sleep(0.5)
                
                # NEW: Parallel Execution State
                elif self.state == "PARALLEL_EXECUTION":
                    if len(self.assembly_queue) >= 2:
                        self.get_logger().info('⚡ PARALLEL EXECUTION: Both arms executing simultaneously')
                        
                        # Get first two bricks
                        ar4_brick = None
                        abb_brick = None
                        
                        for brick in list(self.assembly_queue):
                            if not ar4_brick and brick.start_side == "AR4":
                                ar4_brick = brick
                                self.assembly_queue.remove(brick)
                            elif not abb_brick and brick.start_side == "ABB":
                                abb_brick = brick
                                self.assembly_queue.remove(brick)
                        
                        # If we have both, dispatch simultaneously
                        if ar4_brick and abb_brick:
                            self.ar4_busy = True
                            self.abb_busy = True
                            
                            asyncio.create_task(self.run_ar4_mission(ar4_brick))
                            asyncio.create_task(self.run_abb_mission(abb_brick))
                            
                            self.state = "DISPATCH"
                        else:
                            # Not enough compatible bricks, go back to dispatch
                            self.state = "DISPATCH"
                    else:
                        self.state = "DISPATCH"
                
                # NEW: Parallel Pick & Place State
                elif self.state == "PARALLEL_PICK_PLACE":
                    self.get_logger().info(f'⚡ Starting Parallel Pick for {self.current_brick.id}')
                    
                    if not self.ar4_busy and not self.abb_busy and self.current_brick:
                        self.ar4_busy = True
                        self.abb_busy = True
                        
                        brick = self.current_brick
                        
                        # Create concurrent pick tasks
                        async def parallel_pick():
                            ar4_pick_goal = MoveToPose.Goal()
                            ar4_pick_goal.strategy = "PICK"
                            ar4_pick_goal.target_pose = brick.pickup_pose
                            
                            abb_pick_goal = ExecuteTask.Goal()
                            abb_pick_goal.task_type = "PICK"
                            if self.current_grasp_point:
                                abb_pick_goal.target_pose = self.current_grasp_point.pose
                            else:
                                abb_pick_goal.target_pose = brick.pickup_pose
                            
                            # Run both picks concurrently
                            ar4_result, abb_result = await asyncio.gather(
                                self.send_action_goal(self.ar4_point_client, ar4_pick_goal),
                                self.send_action_goal(self.abb_task_client, abb_pick_goal),
                                return_exceptions=True
                            )
                            return ar4_result, abb_result
                        
                        # Dispatch parallel picks
                        asyncio.create_task(self._handle_parallel_picks(brick, parallel_pick()))
                    else:
                        await asyncio.sleep(0.5)

            except Exception as e:
                self.get_logger().error(f"Dispatcher Error: {e}")
            await asyncio.sleep(0.1)

    # =========================
    # HANDLER HELPERS
    # =========================

    async def handle_init(self):
        self.get_logger().info('Fetching Plan...')
        if not self.gui_client.wait_for_service(timeout_sec=1.0): return False
        res = await self.gui_client.call_async(GetAssemblyPlan.Request())
        if res and res.plan:
            self.assembly_queue = res.plan
            return True
        return False

    async def handle_detect(self):
        self.get_logger().info('Detecting Bricks...')
        if not self.camera_client.wait_for_service(timeout_sec=1.0): return False
        res = await self.camera_client.call_async(DetectBricks.Request())
        if res:
            # Transform as per your original logic
            for brick in res.bricks:
                brick.pose = self.transform_pose_to_abb(brick.pose)
            return True
        return False

    async def handle_grasp_pipeline(self):
        self.get_logger().info(f'Getting Grasp for {self.current_brick.id}')
        if not self.grasp_pipeline_client.wait_for_service(timeout_sec=2.0): return False
        
        req = GetGrasp.Request()
        req.brick_index = str(self.current_brick.id)
        res = await self.grasp_pipeline_client.call_async(req)

        if res and res.success:
            raw_grasp = res.grasp_point
            raw_grasp.pose = self.transform_pose_to_abb(raw_grasp.pose)
            raw_grasp.pose.position.z = 0.22 
            self.current_grasp_point = raw_grasp
            return True
        return False

    async def _handle_parallel_picks(self, brick, picks_coro):
        """Handle parallel pick execution for both arms."""
        try:
            ar4_result, abb_result = await picks_coro
            
            # Check results
            if isinstance(ar4_result, Exception) or ar4_result is None:
                self.get_logger().error(f"AR4 parallel pick failed")
                self.ar4_busy = False
                self.abb_busy = False
                self.state = "DISPATCH"
                return
            
            if isinstance(abb_result, Exception) or abb_result is None:
                self.get_logger().error(f"ABB parallel pick failed")
                self.ar4_busy = False
                self.abb_busy = False
                self.state = "DISPATCH"
                return
            
            # Both picks succeeded - proceed to parallel place
            self.get_logger().info('✅ Parallel picks complete - proceeding to parallel place')
            await asyncio.sleep(0.5)
            
            # Create concurrent place tasks
            ar4_place_goal = MoveToPose.Goal()
            ar4_place_goal.strategy = "PLACE"
            ar4_place_goal.target_pose = brick.place_pose
            
            abb_place_goal = ExecuteTask.Goal()
            abb_place_goal.task_type = "PLACE"
            abb_place_goal.target_pose = brick.place_pose
            abb_place_goal.target_pose.pose.position.z = 0.24
            
            # Run both places concurrently
            ar4_place_result, abb_place_result = await asyncio.gather(
                self.send_action_goal(self.ar4_point_client, ar4_place_goal),
                self.send_action_goal(self.abb_task_client, abb_place_goal),
                return_exceptions=True
            )
            
            # Check place results
            if isinstance(ar4_place_result, Exception) or ar4_place_result is None:
                self.get_logger().error(f"AR4 parallel place failed")
            elif isinstance(abb_place_result, Exception) or abb_place_result is None:
                self.get_logger().error(f"ABB parallel place failed")
            else:
                self.get_logger().info('✅ Parallel Pick & Place complete')
            
            self.ar4_busy = False
            self.abb_busy = False
            self.state = "DISPATCH"
            
        except Exception as e:
            self.get_logger().error(f"Parallel pick handler failed: {e}")
            self.ar4_busy = False
            self.abb_busy = False
            self.state = "DISPATCH"

    async def send_action_goal(self, client, goal_msg):
        """Action helper that awaits the physical movement result."""
        if not client.wait_for_server(timeout_sec=5.0): return False
        goal_handle = await client.send_goal_async(goal_msg)
        if not goal_handle.accepted: return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

# =========================
# MAIN
# =========================

async def main(args=None):
    rclpy.init(args=args)
    node = AssemblySupervisor()
    asyncio.create_task(node.state_machine_loop())
    await ros_spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())