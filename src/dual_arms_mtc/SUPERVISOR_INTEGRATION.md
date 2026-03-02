# Supervisor Integration with Sequential Handover + Parallel Pick/Place

## Current Supervisor Architecture

The supervisor is a **state machine orchestrator** that controls the overall assembly workflow:

```
GUI → Assembly Plan
       ↓
Supervisor ← Camera Detection
   ├─→ AR4 Controller (Point Control / Visual Servo)
   └─→ ABB Controller (Pick/Place Actions)
```

### Current State Flow (without new implementation)
```
INIT → DETECT → PROCESS_NEXT
                    ↓
         ┌──────────┼──────────┐
         ↓          ↓          ↓
    EXECUTE_   EXECUTE_    HANDOVER_
    AR4_      ABB_PICK    SEQUENCE
    DIRECT    ↓            ↓
         (sequential)   (hardcoded
                         sequential)
```

---

## New Supervisor Integration Points

The supervisor needs to integrate the **Zone Detection Manager** and **Hybrid MTC Controller** at crucial decision points:

### 1. **Operation Type Detection** (New)
```python
# In GRASP_PIPELINE state, before branching:

# NEW: Get operation type recommendation
operation_type = self.zone_manager.determine_required_operation_type(
    ar4_current_pose,  # From TF
    abb_current_pose   # From TF
)

if operation_type == OperationType.HANDOVER:
    self.state = "HANDOVER_SEQUENCE"
elif operation_type == OperationType.PICK_PLACE:
    # Check if both arms are ready to pick independently
    if self.zone_manager.can_operate_in_parallel(ar4_pose, abb_pose):
        self.state = "PARALLEL_EXECUTION"
    else:
        self.state = self.current_brick.target_side  # Default to sequential
else:  # SYNCHRONIZED
    self.state = "SYNCHRONIZED_EXECUTION"
```

---

## Modified State Machine with New Implementation

```
┌─────────────────────────────────────────────────────────────────┐
│                SUPERVISOR STATE MACHINE                         │
└─────────────────────────────────────────────────────────────────┘

                    INIT
                     ↓
                  DETECT
                     ↓
               PROCESS_NEXT
                     ↓
              GRASP_PIPELINE
                     ↓
         ┌───────────┼───────────┐
         │           │           │
    [AR4]        [ABB]       [HANDOVER]
         ↓           ↓           ↓
    ┌──────────┐ ┌──────────┐ ┌──────────────────────┐
    │ Execute  │ │ Execute  │ │ Operation Type       │
    │ AR4      │ │ ABB      │ │ Detection:           │
    │ Pick     │ │ Pick     │ │ - Can_Parallel?      │
    │          │ │          │ │ - Should_Handover?   │
    └─────┬────┘ └─────┬────┘ └──────────┬───────────┘
          │            │                  │
          ▼            ▼                  ↓
    [AR4_PLACE]  [ABB_PLACE]  ┌──────────────────────┐
          │            │       │ PARALLEL_EXECUTION  │
          │            │       │ (NEW)               │
          │            │       │ - ARM1 & ARM2       │
          │            │       │   simultaneous      │
          │            │       └──────────┬──────────┘
          │            │                  │
          └────┬───────┘                  │
               │                          │
               ↓                          ↓
         ┌──────────────────────────────────────┐
         │       SEQUENTIAL_HANDOVER (NEW)      │
         │ - Phase 1: AR4 Pick → Intermediate   │
         │ - Phase 2: ABB Pick from Intermediate│
         │ - Phase 3: ABB Place                 │
         └───────────┬────────────────────┬─────┘
                     │                    │
              [Signal ARM1    [Signal ARM2
               Complete]      Complete]
                     │                    │
              [Wait for ARM1  [Continue
               Completion]     to Place]
                     │                    │
                     └────────┬──────────┘
                              ↓
                         PROCESS_NEXT
```

---

## Key Integration Changes

### A. New Imports in Supervisor
```python
# Add to supervisor.py imports:
from dual_arms_mtc.control_strategy import OperationType, ExecutionModel, OperationPhase
from dual_arms_mtc.hybrid_mtc_controller import HybridMTCController
from dual_arms_mtc.zone_detection_manager import ZoneDetectionManager
```

### B. New Clients in `__init__`
```python
def __init__(self):
    # ... existing code ...
    
    # NEW: Zone Detection Manager (C++ node)
    self.zone_manager = ZoneDetectionManager(self)  # Or as external node
    
    # NEW: Hybrid MTC Controller (C++ node)
    self.mtc_controller = HybridMTCController(self)  # Or as external node
    
    # NEW: Clients for zone detection decisions
    self.zone_client = self.create_client(
        GetOperationType,           # Custom service
        'zone_manager/get_operation_type'
    )
```

### C. Current Arm Pose Tracking
```python
def __init__(self):
    # ... existing code ...
    
    # NEW: Track current arm poses for zone decisions
    self.ar4_current_pose = None
    self.abb_current_pose = None
    
    # NEW: TF2 listener for arm poses
    # (Already have self.tf_buffer from existing code)

async def update_arm_poses(self):
    """Get current end-effector poses from TF"""
    try:
        # AR4 EE pose
        t_ar4 = self.tf_buffer.lookup_transform(
            'world', 'ar4_tool_link', rclpy.time.Time()
        )
        self.ar4_current_pose = self.transform_stamped_to_pose(t_ar4)
        
        # ABB EE pose
        t_abb = self.tf_buffer.lookup_transform(
            'world', 'abb_tool_link', rclpy.time.Time()
        )
        self.abb_current_pose = self.transform_stamped_to_pose(t_abb)
        
    except TransformException as e:
        self.get_logger().warn(f'Could not get arm poses: {e}')
```

### D. Modified GRASP_PIPELINE State
```python
elif self.state == "GRASP_PIPELINE":
    self.get_logger().info(f'Calling GetGrasp Service for Brick ID: {self.current_brick.id}')
    
    # ... existing grasp pipeline code ...
    
    if grasp_result.success:
        # ... existing grasp setup code ...
        
        # NEW: Determine operation type based on current positions
        await self.update_arm_poses()
        
        operation_type = self.zone_manager.determine_required_operation_type(
            self.ar4_current_pose,
            self.abb_current_pose
        )
        
        self.get_logger().info(f'🎯 Operation Type Detected: {operation_type}')
        
        # NEW: Route to appropriate state
        if operation_type == OperationType.HANDOVER:
            self.state = "INITIALIZE_SEQUENTIAL_HANDOVER"
        elif operation_type == OperationType.PICK_PLACE:
            if self.zone_manager.can_operate_in_parallel(
                self.ar4_current_pose,
                self.abb_current_pose
            ):
                self.state = "INITIALIZE_PARALLEL_EXECUTION"
            else:
                # Fall back to sequential by arm
                self.state = self.current_brick.start_side  # AR4 or ABB
        else:
            self.state = self.current_brick.start_side
```

### E. New State: INITIALIZE_SEQUENTIAL_HANDOVER
```python
elif self.state == "INITIALIZE_SEQUENTIAL_HANDOVER":
    self.get_logger().info('🔄 SEQUENTIAL HANDOVER STARTING')
    
    # NEW: Initialize operation with MTC controller
    operation_id = self.mtc_controller.initialize_operation(
        OperationType.HANDOVER
    )
    
    self.current_operation_id = operation_id
    self.get_logger().info(f'Operation ID: {operation_id}')
    
    # Get intermediate pose (between AR4 pick and ABB pick)
    intermediate_pose = self.calculate_intermediate_pose(
        self.current_grasp_point.pose,
        self.current_brick.place_pose
    )
    
    # NEW: Begin sequential handover with MTC
    self.mtc_controller.begin_sequential_handover(
        ar4_start=self.ar4_current_pose,
        abb_start=self.abb_current_pose,
        handover_point=self.current_grasp_point.pose,
        intermediate_pose=intermediate_pose
    )
    
    self.state = "WAIT_HANDOVER_ARM1_COMPLETE"

elif self.state == "WAIT_HANDOVER_ARM1_COMPLETE":
    self.get_logger().info('⏳ Waiting for AR4 (ARM1) to reach intermediate position...')
    
    # NEW: Signal that ARM1 (AR4) has completed its task
    self.mtc_controller.signal_arm1_complete()
    
    # NEW: Wait for arm1 completion (blocking with timeout)
    arm1_complete = self.mtc_controller.wait_for_arm1_completion(timeout_ms=5000)
    
    if arm1_complete:
        self.get_logger().info('✅ AR4 complete, ABB can now proceed')
        self.state = "WAIT_HANDOVER_ARM2_COMPLETE"
    else:
        self.get_logger().error('❌ ARM1 timeout during handover')
        self.state = "RECOVERY"

elif self.state == "WAIT_HANDOVER_ARM2_COMPLETE":
    self.get_logger().info('⏳ Waiting for ABB (ARM2) to complete handover...')
    
    # NEW: Signal that ARM2 (ABB) is ready
    self.mtc_controller.signal_arm2_ready()
    
    # Wait for MTC task to complete
    context = self.mtc_controller.get_operation_context()
    
    if context.phase == OperationPhase.COMPLETION:
        self.get_logger().info('✅ Handover complete')
        self.state = "PROCESS_NEXT"
    elif context.phase == OperationPhase.ERROR:
        self.get_logger().error('❌ Handover failed')
        self.state = "RECOVERY"
```

### F. New State: INITIALIZE_PARALLEL_EXECUTION
```python
elif self.state == "INITIALIZE_PARALLEL_EXECUTION":
    self.get_logger().info('⚡ PARALLEL PICK/PLACE STARTING')
    
    # NEW: Determine which arm picks which brick
    # Could have multiple bricks in queue for both arms
    
    # Get AR4 target
    ar4_brick = self.assembly_queue[0] if self.assembly_queue else None
    # Get ABB target (next in queue or from separate stack)
    abb_brick = self.assembly_queue[1] if len(self.assembly_queue) > 1 else None
    
    if ar4_brick and abb_brick:
        # NEW: Initialize parallel operation
        operation_id = self.mtc_controller.initialize_operation(
            OperationType.PICK_PLACE
        )
        
        # NEW: Begin parallel execution
        self.mtc_controller.begin_parallel_pick_place(
            ar4_target=ar4_brick.pickup_pose,
            abb_target=abb_brick.pickup_pose
        )
        
        self.state = "WAIT_PARALLEL_EXECUTION"
    else:
        self.get_logger().info('Not enough bricks for parallel execution')
        self.state = "PROCESS_NEXT"

elif self.state == "WAIT_PARALLEL_EXECUTION":
    self.get_logger().info('⏳ Both arms executing in parallel...')
    
    # Both arms execute simultaneously
    # Check if operation is complete
    context = self.mtc_controller.get_operation_context()
    
    if context.phase == OperationPhase.COMPLETION:
        self.get_logger().info('✅ Parallel execution complete')
        # Pop both processed bricks
        self.assembly_queue.pop(0)  # AR4 brick
        if self.assembly_queue:
            self.assembly_queue.pop(0)  # ABB brick
        self.state = "PROCESS_NEXT"
    elif context.phase == OperationPhase.ERROR:
        self.get_logger().error('❌ Parallel execution failed')
        self.state = "RECOVERY"
```

---

## Data Flow Diagram

```
       ┌────────────────────────────────┐
       │  Current Arm Poses (TF)         │
       │  - AR4 TCP position/orientation │
       │  - ABB TCP position/orientation │
       └────────────┬────────────────────┘
                    │
                    ▼
       ┌────────────────────────────────┐
       │  Zone Detection Manager        │
       │  - Calculate separation        │
       │  - Determine zones             │
       │  - Return: OperationType       │
       └────────────┬────────────────────┘
                    │
           ╔════════╩════════╗
           ║                 ║
        HANDOVER         PICK_PLACE
           ║                 ║
           ▼                 ▼
    ┌──────────────┐  ┌──────────────┐
    │MTC Sequential│  │MTC Parallel  │
    │Handover Task │  │Pick/Place    │
    │              │  │              │
    │ ARM1→ARM2    │  │ ARM1 || ARM2 │
    │ Sequential   │  │ Parallel     │
    └──────────────┘  └──────────────┘
           │                 │
           └────────┬────────┘
                    │
                    ▼
           ┌──────────────────┐
           │ Monitor Phase    │
           │ Update State     │
           │ Signal           │
           │ Completion       │
           └──────────────────┘
```

---

## Critical Integration Points

### 1. **Arm Pose Updates**
Must be called frequently to keep zone detection accurate:
```python
async def state_machine_loop(self):
    # At the beginning of each loop iteration
    await self.update_arm_poses()
    
    # Then proceed with state logic
    if self.state == "GRASP_PIPELINE":
        # ... will use current poses
```

### 2. **Zone-Based Branching**
Must happen BEFORE committing to a sequence:
```python
# GOOD: Get operation type recommendation
op_type = self.zone_manager.determine_required_operation_type(current_poses)

# Branch based on recommendation
if op_type == HANDOVER:
    use_sequential_handover()
elif op_type == PICK_PLACE:
    use_parallel_execution()
```

### 3. **ARM1→ARM2 Synchronization (Sequential Only)**
Must properly enforce sequencing:
```python
# ARM1 (AR4) completes
self.mtc_controller.signal_arm1_complete()

# ARM2 (ABB) waits
success = self.mtc_controller.wait_for_arm1_completion(timeout=5000)

# Only proceed if ARM1 succeeded
if success:
    # ARM2 can proceed
    self.mtc_controller.signal_arm2_ready()
```

### 4. **Parallel Execution (No Synchronization)**
Both arms execute without blocking:
```python
# Start both simultaneously
self.mtc_controller.begin_parallel_pick_place(ar4_target, abb_target)

# Check completion occasionally but don't block
context = self.mtc_controller.get_operation_context()
if context.phase == OperationPhase.COMPLETION:
    # Both done
```

---

## Backward Compatibility

### What Stays the Same
- ✅ Existing AR4 direct control still works
- ✅ Existing ABB control still works
- ✅ MULTITHREADED mode still available
- ✅ Gripper control unchanged
- ✅ Camera detection unchanged

### What Changes
- ⚠️ HANDOVER_SEQUENCE state logic (adds new operation types)
- ⚠️ GRASP_PIPELINE state (adds operation type detection)
- ⚠️ State names/transitions (adds new states for parallel)

### Migration Path
```python
# OLD: Simple sequential routing
if brick.start_side == "AR4":
    state = "EXECUTE_AR4_DIRECT"
elif brick.start_side == "ABB":
    state = "EXECUTE_ABB_PICK"
elif brick.start_side == "HANDOVER":
    state = "HANDOVER_SEQUENCE"

# NEW: Smart routing with zone awareness
op_type = zone_manager.determine_required_operation_type(poses)
if op_type == HANDOVER:
    state = "INITIALIZE_SEQUENTIAL_HANDOVER"
elif op_type == PICK_PLACE and can_parallel:
    state = "INITIALIZE_PARALLEL_EXECUTION"
else:
    # Fall back to old logic
    if brick.start_side == "AR4":
        state = "EXECUTE_AR4_DIRECT"
    # ... etc
```

---

## Configuration in Supervisor

### ROS Parameters for Supervisor
```yaml
# supervisor_config.yaml
supervisor:
  # Operation type detection
  enable_operation_type_detection: true
  enable_parallel_execution: true
  
  # Parallel operation safety
  min_parallel_separation: 0.3  # meters
  
  # Synchronization timeouts
  handover_arm1_timeout_ms: 5000
  handover_arm2_timeout_ms: 5000
  
  # Arm pose update frequency
  arm_pose_update_hz: 10  # Hz
```

### Loading in Supervisor
```python
def __init__(self):
    # ... existing code ...
    
    # NEW: Load configuration
    self.declare_parameter('enable_operation_type_detection', True)
    self.declare_parameter('enable_parallel_execution', True)
    self.declare_parameter('min_parallel_separation', 0.3)
    
    self.enable_operation_type_detection = \
        self.get_parameter('enable_operation_type_detection').value
    self.enable_parallel_execution = \
        self.get_parameter('enable_parallel_execution').value
```

---

## Testing the Integration

### Unit Tests Needed
```python
# Test zone detection integration
def test_supervisor_detects_handover_zone():
    # Mock arm poses in handover zone
    # Verify supervisor calls sequential handover

# Test parallel detection
def test_supervisor_detects_parallel_safe():
    # Mock arm poses far apart in safe zones
    # Verify supervisor enables parallel

# Test state transitions
def test_supervisor_handover_state_transitions():
    # Verify: GRASP → INIT_HANDOVER → WAIT_ARM1 → WAIT_ARM2 → PROCESS_NEXT

# Test synchronization
def test_supervisor_arm1_arm2_sync():
    # Verify ARM2 blocks until ARM1 signals completion
```

### Integration Tests Needed
```
1. End-to-end handover with new sequential task
2. Parallel pick/place with real TF updates
3. Mode switching under changing conditions
4. Timeout handling and recovery
5. Multiple bricks in parallel
```

---

## Summary

The supervisor now acts as the **high-level orchestrator** for the new implementation:

| Role | Component | Responsibility |
|------|-----------|-----------------|
| **Decision Maker** | Supervisor | Get arm poses → Ask zone manager for operation type → Branch logic |
| **Zone Expert** | Zone Manager | Analyze arm separation → Determine if parallel/handover/synchronized |
| **Executor** | MTC Controller | Execute one or both arms → Handle synchronization → Track phases |
| **State Machine** | Supervisor | Route through states → Monitor completion → Coordinate transitions |

The new implementation **empowers the supervisor** to make smart decisions based on real-time arm positions rather than just brick properties.
