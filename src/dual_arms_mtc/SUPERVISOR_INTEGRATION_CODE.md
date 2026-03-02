# Copy-Paste Integration Code for supervisor.py

## 1. ADD THESE IMPORTS AT THE TOP

```python
# Add to existing imports section in supervisor.py

# NEW: Control Strategy Types
from dual_arms_mtc.control_strategy import (
    OperationType, ExecutionModel, OperationPhase, OperationContext
)

# Note: Zone Manager and MTC Controller are C++ nodes,
# so they communicate via ROS services/actions, not direct imports
```

---

## 2. ADD THESE TO __init__ METHOD

```python
class AssemblySupervisor(Node):
    def __init__(self):
        # ... existing code ...
        
        # NEW: Arm pose tracking for zone decisions
        self.ar4_current_pose = None
        self.abb_current_pose = None
        
        # NEW: Current operation tracking
        self.current_operation_id = None
        self.current_operation_type = None
        
        # NEW: Enable/disable new features
        self.declare_parameter('enable_operation_type_detection', True)
        self.declare_parameter('enable_parallel_execution', True)
        self.declare_parameter('handover_arm1_timeout_ms', 5000)
        
        self.enable_operation_type_detection = \
            self.get_parameter('enable_operation_type_detection').value
        self.enable_parallel_execution = \
            self.get_parameter('enable_parallel_execution').value
        self.handover_timeout = \
            self.get_parameter('handover_arm1_timeout_ms').value
        
        # NEW: Clients to communicate with C++ nodes
        # Zone Detection Manager service client
        self.zone_client = self.create_client(
            GetOperationType,  # Custom service type
            'zone_manager/get_operation_type',
            callback_group=self.cb_group
        )
        
        # MTC Controller action clients
        # (if using action servers)
        # For now, assume MTC controller has service methods
        
        self.get_logger().info('✅ Sequential Handover + Parallel Pick/Place enabled')
```

---

## 3. ADD THIS HELPER METHOD (Copy in full)

```python
    async def update_arm_poses(self):
        """
        Fetch current end-effector poses from TF2.
        Called at beginning of each state_machine_loop iteration.
        """
        try:
            # Get AR4 TCP pose (adjust frame names as needed)
            t_ar4 = self.tf_buffer.lookup_transform(
                'world',           # Target frame
                'ar4_tool_link',   # Source frame
                rclpy.time.Time()  # Latest
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
```

---

## 4. MODIFY state_machine_loop AT TOP

```python
    async def state_machine_loop(self):
        self.timer.cancel()
        
        try:
            # NEW: Update arm poses for zone detection at each iteration
            if self.enable_operation_type_detection:
                await self.update_arm_poses()
            
            # ... rest of existing code ...
```

---

## 5. MODIFY THE GRASP_PIPELINE STATE

Replace this section in GRASP_PIPELINE (around line 200):

**BEFORE:**
```python
            elif self.state == "GRASP_PIPELINE":
                # ... existing grasp code ...
                if grasp_result.success:
                    # ... grasp setup ...
                    
                    # OLD: Simple branching
                    if self.current_brick.start_side == "ABB":
                        self.state = "EXECUTE_ABB_PICK"
                    elif self.current_brick.start_side == "HANDOVER":
                        self.state = "HANDOVER_SEQUENCE"
                    elif self.current_brick.start_side == "AR4":
                        self.state = "EXECUTE_AR4_DIRECT"
```

**AFTER:**
```python
            elif self.state == "GRASP_PIPELINE":
                # ... existing grasp code ...
                if grasp_result.success:
                    # ... grasp setup ...
                    
                    # NEW: Smart operation type detection
                    if self.enable_operation_type_detection and \
                       self.ar4_current_pose and self.abb_current_pose:
                        
                        # Call zone manager to determine operation type
                        operation_type = await self.get_operation_type_from_zone_manager()
                        
                        self.current_operation_type = operation_type
                        self.get_logger().info(f'🎯 Operation Type: {operation_type}')
                        
                        if operation_type == OperationType.HANDOVER:
                            self.state = "INITIALIZE_SEQUENTIAL_HANDOVER"
                        elif operation_type == OperationType.PICK_PLACE and \
                             self.enable_parallel_execution:
                            self.state = "INITIALIZE_PARALLEL_EXECUTION"
                        else:  # SYNCHRONIZED or fallback
                            # OLD: Default to brick.start_side
                            if self.current_brick.start_side == "ABB":
                                self.state = "EXECUTE_ABB_PICK"
                            elif self.current_brick.start_side == "AR4":
                                self.state = "EXECUTE_AR4_DIRECT"
                    else:
                        # Fallback to old logic if detection disabled
                        if self.current_brick.start_side == "ABB":
                            self.state = "EXECUTE_ABB_PICK"
                        elif self.current_brick.start_side == "HANDOVER":
                            self.state = "HANDOVER_SEQUENCE"
                        elif self.current_brick.start_side == "AR4":
                            self.state = "EXECUTE_AR4_DIRECT"
```

---

## 6. ADD THESE NEW STATES (Insert before "STATE: RECOVERY")

```python
            # =========================
            # STATE: SEQUENTIAL HANDOVER (NEW)
            # =========================
            elif self.state == "INITIALIZE_SEQUENTIAL_HANDOVER":
                self.get_logger().info('🔄 SEQUENTIAL HANDOVER INITIALIZING')
                
                # Call MTC controller to initialize handover operation
                try:
                    init_result = await self.initialize_handover_operation()
                    if init_result:
                        self.current_operation_id = init_result
                        self.state = "EXECUTE_SEQUENTIAL_HANDOVER_ARM1"
                    else:
                        self.get_logger().error('Failed to initialize handover')
                        self.state = "RECOVERY"
                except Exception as e:
                    self.get_logger().error(f'Handover init failed: {e}')
                    self.state = "RECOVERY"

            elif self.state == "EXECUTE_SEQUENTIAL_HANDOVER_ARM1":
                self.get_logger().info('⚙️  AR4 (ARM1) Executing: Pick → Intermediate')
                
                # AR4 picks and moves to intermediate position
                # This is handled by MTC task, but supervision happens here
                try:
                    result = await self.execute_ar4_pick_for_handover()
                    if result:
                        self.get_logger().info('✅ AR4 reached intermediate position')
                        # Signal that ARM1 completed
                        await self.signal_arm1_complete()
                        self.state = "EXECUTE_SEQUENTIAL_HANDOVER_ARM2"
                    else:
                        self.get_logger().error('AR4 failed to reach intermediate')
                        self.state = "RECOVERY"
                except Exception as e:
                    self.get_logger().error(f'ARM1 execution failed: {e}')
                    self.state = "RECOVERY"

            elif self.state == "EXECUTE_SEQUENTIAL_HANDOVER_ARM2":
                self.get_logger().info('⏳ Waiting for AR4 completion signal...')
                
                # Wait for ARM1 to complete
                try:
                    arm1_done = await self.wait_for_arm1_completion(
                        self.handover_timeout
                    )
                    if arm1_done:
                        self.get_logger().info('✅ ARM1 complete, ABB (ARM2) can proceed')
                        self.state = "EXECUTE_ABB_PICK_FROM_HANDOVER"
                    else:
                        self.get_logger().error('ARM1 timeout!')
                        self.state = "RECOVERY"
                except Exception as e:
                    self.get_logger().error(f'ARM1 wait failed: {e}')
                    self.state = "RECOVERY"

            elif self.state == "EXECUTE_ABB_PICK_FROM_HANDOVER":
                self.get_logger().info('⚙️  ABB (ARM2) Executing: Pick from Intermediate → Place')
                
                # ABB picks from intermediate and places
                try:
                    # Signal to MTC that ARM2 is ready
                    await self.signal_arm2_ready()
                    
                    # Execute ABB pick/place from handover
                    result = await self.execute_abb_handover_pick_place()
                    if result:
                        self.get_logger().info('✅ ABB completed handover pick/place')
                        # Signal that ARM2 completed
                        await self.signal_arm2_complete()
                        self.state = "PROCESS_NEXT"
                    else:
                        self.get_logger().error('ABB failed handover pick/place')
                        self.state = "RECOVERY"
                except Exception as e:
                    self.get_logger().error(f'ARM2 execution failed: {e}')
                    self.state = "RECOVERY"

            # =========================
            # STATE: PARALLEL EXECUTION (NEW)
            # =========================
            elif self.state == "INITIALIZE_PARALLEL_EXECUTION":
                self.get_logger().info('⚡ PARALLEL EXECUTION INITIALIZING')
                
                # Check we have bricks for both arms
                if len(self.assembly_queue) < 2:
                    self.get_logger().warn('Not enough bricks for parallel execution')
                    self.state = "PROCESS_NEXT"
                    return
                
                # Get AR4 and ABB targets
                ar4_brick = self.assembly_queue[0]
                abb_brick = self.assembly_queue[1]
                
                try:
                    init_result = await self.initialize_parallel_operation(
                        ar4_brick, abb_brick
                    )
                    if init_result:
                        self.current_operation_id = init_result
                        self.state = "EXECUTE_PARALLEL_OPERATIONS"
                    else:
                        self.get_logger().error('Failed to initialize parallel operation')
                        self.state = "PROCESS_NEXT"
                except Exception as e:
                    self.get_logger().error(f'Parallel init failed: {e}')
                    self.state = "PROCESS_NEXT"

            elif self.state == "EXECUTE_PARALLEL_OPERATIONS":
                self.get_logger().info('⚡ Both arms executing in PARALLEL')
                
                # Both arms are now executing simultaneously
                # Monitor for completion
                try:
                    context = await self.get_operation_context()
                    
                    if context.phase == OperationPhase.COMPLETION:
                        self.get_logger().info('✅ Parallel execution complete')
                        # Remove both processed bricks
                        self.assembly_queue.pop(0)  # AR4 brick
                        if self.assembly_queue:
                            self.assembly_queue.pop(0)  # ABB brick
                        self.state = "PROCESS_NEXT"
                    elif context.phase == OperationPhase.ERROR:
                        self.get_logger().error('❌ Parallel execution error')
                        self.state = "RECOVERY"
                except Exception as e:
                    self.get_logger().error(f'Parallel monitoring failed: {e}')
                    self.state = "RECOVERY"
```

---

## 7. ADD THESE HELPER METHODS (Copy in full)

```python
    async def get_operation_type_from_zone_manager(self):
        """
        Query zone manager to determine operation type based on arm positions.
        Returns: OperationType enum value
        """
        try:
            # Wait for service
            if not self.zone_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Zone manager service not available')
                return None
            
            # Call service (would need custom service definition)
            # For now, return based on positions
            # In real implementation, call the C++ service
            
            separation = self.calculate_separation(
                self.ar4_current_pose,
                self.abb_current_pose
            )
            
            # If in handover zone
            if separation < 0.5:  # Example threshold
                return OperationType.HANDOVER
            
            # If far apart
            if separation > 0.8:  # Example threshold
                return OperationType.PICK_PLACE
            
            # Otherwise synchronized
            return OperationType.SYNCHRONIZED
            
        except Exception as e:
            self.get_logger().error(f'Error getting operation type: {e}')
            return None

    def calculate_separation(self, pose1, pose2):
        """Calculate distance between two poses"""
        if not pose1 or not pose2:
            return float('inf')
        
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        
        return (dx**2 + dy**2 + dz**2) ** 0.5

    async def initialize_handover_operation(self):
        """Initialize sequential handover with MTC controller"""
        # This would call the MTC controller service/action
        # For now, return operation ID
        import uuid
        return str(uuid.uuid4())

    async def signal_arm1_complete(self):
        """Signal that ARM1 (AR4) has completed its task"""
        # Call MTC controller to signal ARM1 completion
        self.get_logger().info('🔔 Signaling ARM1 complete')

    async def wait_for_arm1_completion(self, timeout_ms):
        """Wait for ARM1 to complete with timeout"""
        # This would call MTC controller's wait method
        # Simulated here
        import asyncio
        await asyncio.sleep(0.1)  # Simulate waiting
        return True

    async def signal_arm2_ready(self):
        """Signal that ARM2 (ABB) is ready to proceed"""
        self.get_logger().info('🔔 Signaling ARM2 ready')

    async def signal_arm2_complete(self):
        """Signal that ARM2 (ABB) has completed its task"""
        self.get_logger().info('🔔 Signaling ARM2 complete')

    async def execute_ar4_pick_for_handover(self):
        """Execute AR4 pick and move to intermediate position"""
        # Similar to existing AR4 pick, but moves to intermediate
        goal = MoveToPose.Goal()
        goal.target_pose = self.current_grasp_point.pose
        goal.strategy = "APPROACH_FOR_HANDOVER"
        
        action_result = await self.send_action_goal(self.ar4_point_client, goal)
        return action_result is not None and action_result.success

    async def execute_abb_handover_pick_place(self):
        """Execute ABB pick from intermediate and place"""
        # ABB picks from intermediate position
        abb_goal = ExecuteTask.Goal()
        abb_goal.task_type = "PICK_FROM_HANDOVER"
        abb_goal.target_pose = self.handover_pose
        
        result = await self.send_action_goal(self.abb_client, abb_goal)
        return result is not None and result.success

    async def initialize_parallel_operation(self, ar4_brick, abb_brick):
        """Initialize parallel pick/place operation"""
        import uuid
        return str(uuid.uuid4())

    async def get_operation_context(self):
        """Get current operation context from MTC controller"""
        # Would query MTC controller for operation phase
        # Return mock for now
        return OperationContext(phase=OperationPhase.COMPLETION)
```

---

## 8. OPTIONAL: ADD DIAGNOSTIC LOGGING

Add this method for debugging:

```python
    def log_operation_state(self):
        """Log current operation state for debugging"""
        self.get_logger().info(
            f'Operation State: ID={self.current_operation_id}, '
            f'Type={self.current_operation_type}, '
            f'AR4_Pose={self.ar4_current_pose}, '
            f'ABB_Pose={self.abb_current_pose}'
        )
```

Call it periodically:
```python
    if self.state in ["INITIALIZE_SEQUENTIAL_HANDOVER", 
                      "INITIALIZE_PARALLEL_EXECUTION"]:
        self.log_operation_state()
```

---

## CONFIGURATION FILE

Create/update `supervisor_config.yaml`:

```yaml
supervisor:
  # NEW: Enable sequential/parallel features
  enable_operation_type_detection: true
  enable_parallel_execution: true
  
  # NEW: Operation timeouts
  handover_arm1_timeout_ms: 5000
  handover_arm2_timeout_ms: 5000
  parallel_execution_timeout_ms: 10000
  
  # NEW: Safety thresholds
  min_parallel_separation: 0.3  # meters
  handover_detection_distance: 0.5  # meters
  
  # Existing parameters
  use_sim: true
  arm_pose_update_hz: 10
```

Launch with:
```bash
ros2 launch supervisor_package supervisor_launch.py \
  config:=src/dual_arms_mtc/config/supervisor_config.yaml
```

---

## QUICK START CHECKLIST

- [ ] Add imports at top of supervisor.py
- [ ] Add new attributes to `__init__`
- [ ] Add `update_arm_poses()` method
- [ ] Call `update_arm_poses()` in state_machine_loop
- [ ] Modify GRASP_PIPELINE state with operation type detection
- [ ] Add 3 new states (INIT, EXECUTE_ARM1, EXECUTE_ARM2 for handover)
- [ ] Add 2 new states for parallel execution
- [ ] Add all helper methods
- [ ] Create supervisor_config.yaml
- [ ] Test with dry-run first (enable_operation_type_detection: false)
- [ ] Enable features one at a time

---

## TESTING

Test the integration step by step:

### Test 1: Pose Updates
```python
# In state_machine loop, verify poses are being updated
if self.ar4_current_pose:
    print(f"AR4: {self.ar4_current_pose}")
if self.abb_current_pose:
    print(f"ABB: {self.abb_current_pose}")
```

### Test 2: Operation Type Detection
```python
# Verify operation types are being detected correctly
op_type = await self.get_operation_type_from_zone_manager()
print(f"Operation Type: {op_type}")
```

### Test 3: State Transitions
```python
# Run with single brick, verify it goes through correct states
# Watch supervisor logs for state progression
```

### Test 4: Full Parallel
```python
# Run with 2 bricks in queue
# Verify both arms execute simultaneously
```

---

## DEBUGGING TIPS

1. **Enable verbose logging**:
   ```python
   self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
   ```

2. **Check TF transformations**:
   ```bash
   ros2 run tf2_tools view_frames.py
   rviz2 -d <YOUR_CONFIG>
   ```

3. **Monitor MTC Controller**:
   ```bash
   ros2 topic echo /hybrid_mtc_controller/operation_state
   ```

4. **Slow-motion execution** (add delays):
   ```python
   import asyncio
   await asyncio.sleep(1.0)  # Delay for debugging
   ```
