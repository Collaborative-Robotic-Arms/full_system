# Sequential Handover + Parallel Pick/Place Implementation Guide

## Overview

This guide explains the modifications made to implement **sequential handover control** and **parallel pick/place operations** for the dual-arm robotic system. The key architectural change ensures that:

- **Handover Process**: Sequential and exclusive - AR4 picks → moves to intermediate → ABB picks → places
- **Pick/Place Operations**: Parallel and independent - both arms can work simultaneously when in safe zones
- **Safety**: Automatic mode switching based on arm positions and collision detection

## Architecture Modification

### Previous Architecture
```
Mode: MULTITHREADED vs MTC_HANDOVER
Execution: Either parallel OR synchronized (no distinction)
Problem: Handover could be treated as parallel, risking collision during transfer
```

### New Architecture
```
Mode: MULTITHREADED | SEQUENTIAL_HANDOVER | PARALLEL_PICK_PLACE
Execution: Operation-type specific
- HANDOVER → SEQUENTIAL (forced)
- PICK_PLACE → PARALLEL (allowed when safe)
```

## New Files Created

### 1. `control_strategy.hpp`
**Location**: `/home/mariamelsebaey/full_system/src/dual_arms_mtc/include/dual_arms_mtc/control_strategy.hpp`

**Purpose**: Defines control strategy enums and operation context tracking

**Key Components**:

#### Enums
```cpp
enum class OperationType {
    IDLE,
    HANDOVER,       // Sequential: AR4 picks → intermediate → ABB picks
    PICK_PLACE,     // Parallel: Both arms independent
    SYNCHRONIZED    // Both arms move together but not handover
};

enum class ExecutionModel {
    SEQUENTIAL,     // One arm waits for the other
    PARALLEL        // Both arms operate independently
};

enum class OperationPhase {
    IDLE,
    INITIATING,
    ARM1_ACTIVE,    // AR4 executing
    ARM1_COMPLETE,  // AR4 done, waiting to signal ABB
    ARM2_ACTIVE,    // ABB executing
    ARM2_COMPLETE,
    COMPLETION,
    ERROR
};
```

#### OperationContext Structure
Tracks:
- Current operation type and execution model
- Operation phase and timing
- ARM1/ARM2 readiness flags
- Operation ID and phase timing for debugging

## Modified Files

### 2. `hybrid_mtc_controller.hpp`
**Modifications**:

```cpp
// New includes
#include <dual_arms_mtc/control_strategy.hpp>
#include <condition_variable>
#include <mutex>

// Updated enum
enum class ControlMode {
    MULTITHREADED,        // Fast point control mode
    SEQUENTIAL_HANDOVER,  // MTC-based sequential handover
    PARALLEL_PICK_PLACE   // MTC-based parallel execution
};

// New public methods for operation management
std::string initialize_operation(OperationType operation_type);
OperationContext get_operation_context() const;
ExecutionModel get_execution_model(OperationType op_type) const;
void update_operation_phase(OperationPhase new_phase);
void signal_arm1_complete();
bool wait_for_arm1_completion(uint64_t timeout_ms = 5000);
void signal_arm2_ready();
void begin_sequential_handover(...);
void begin_parallel_pick_place(...);

// New private members for synchronization
OperationContext current_operation_;
std::mutex operation_mutex_;
std::condition_variable arm1_completion_cv_;
std::condition_variable arm2_ready_cv_;
std::atomic<bool> arm1_completed_;
std::atomic<bool> arm2_ready_;
```

### 3. `mtc_node.cpp`
**New Methods Added**:

#### Sequential Handover Task
```cpp
Task create_sequential_handover_task(
    const geometry_msgs::msg::Pose& ar4_start,
    const geometry_msgs::msg::Pose& abb_start,
    const geometry_msgs::msg::Pose& intermediate_pose,
    const geometry_msgs::msg::Pose& abb_target)
```

**Phases**:
1. AR4 picks brick and moves to intermediate position
2. ABB waits for AR4 completion, then picks from intermediate
3. AR4 retracts while ABB holds object
4. ABB moves to final placement position

#### Parallel Pick/Place Task
```cpp
Task create_parallel_pick_place_task(
    const geometry_msgs::msg::Pose& ar4_target,
    const geometry_msgs::msg::Pose& abb_target)
```

**Features**:
- Both arms use Connect stage for simultaneous planning
- Allows collision between arms (planned to avoid)
- Execution time: ~500ms for planning + execution

#### Operation Management Methods
```cpp
std::string generate_operation_id()
void initialize_operation(OperationType operation_type)
void signal_arm1_complete()
bool wait_for_arm1_completion(uint64_t timeout_ms)
void signal_arm2_ready()
void enforce_execution_model()
bool is_operation_phase_valid()
```

### 4. `zone_detection_manager.hpp`
**Additions**:

```cpp
// New enum
enum class OperationZone {
    AR4_SAFE_ZONE,        // AR4 can pick/place independently
    ABB_SAFE_ZONE,        // ABB can pick/place independently
    HANDOVER_AREA,        // Sequential handover required
    COLLISION_RISK_ZONE   // Too close - potential collision
};

// New configuration parameter
double parallel_safe_separation = 0.3;  // Safe separation for parallel ops

// New public methods
OperationZone get_operation_zone(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose);

bool can_operate_in_parallel(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose);

bool should_use_sequential_handover(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose);

double get_required_parallel_separation() const;

OperationType determine_required_operation_type(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose);
```

### 5. `zone_detection_manager.cpp`
**Implementations**:

#### get_operation_zone()
Returns which zone the arms are in based on their poses and separation

#### can_operate_in_parallel()
Returns true if:
- Both arms in SAFE_ZONE (not approaching handover)
- Separation ≥ parallel_safe_separation (default 0.3m)
- No collision risk detected

#### should_use_sequential_handover()
Returns true if:
- At least one arm in handover zone
- Both arms ready (validated minimum separation)

#### determine_required_operation_type()
Automatically determines which operation mode to use:
- HANDOVER if in handover zone
- PICK_PLACE if safe for parallel
- SYNCHRONIZED otherwise

## Usage Flow

### For Supervisor / High-Level Controller

#### Scenario 1: Parallel Pick/Place
```python
# Both arms in safe zones, far apart
if zone_manager.can_operate_in_parallel(ar4_pose, abb_pose):
    op_id = mtc_controller.initialize_operation(OperationType.PICK_PLACE)
    mtc_controller.begin_parallel_pick_place(ar4_target, abb_target)
    # Both arms execute movements simultaneously
```

#### Scenario 2: Sequential Handover
```python
# AR4 approaching handover zone
if zone_manager.should_use_sequential_handover(ar4_pose, abb_pose):
    op_id = mtc_controller.initialize_operation(OperationType.HANDOVER)
    mtc_controller.begin_sequential_handover(
        ar4_start, abb_start, 
        intermediate_pose, abb_target)
    
    # AR4 picks and moves to intermediate
    mtc_controller.signal_arm1_complete()
    
    # ABB waits for AR4, then picks from intermediate
    if mtc_controller.wait_for_arm1_completion(5000):
        # ABB can now proceed
        pass
```

## Key Design Decisions

### 1. Sequential Forcing for Handover
- Handover operations always use sequential execution
- Prevents collision during object transfer
- AR4 must finish before ABB picks

### 2. Parallel Separation Threshold
- Default: 0.3m separation required for parallel operations
- Configurable via zone_config.parallel_safe_separation
- Based on workspace dimensions and gripper sizes

### 3. Automatic Mode Detection
- Zone manager continuously monitors arm positions
- Supervisor can call `determine_required_operation_type()` to get recommendation
- Allows reactive mode switching

### 4. Synchronization Primitives
- Condition variables for ARM1→ARM2 synchronization
- Prevents race conditions during sequential handover
- Timeout protection (default 5 seconds per phase)

### 5. Operation Context Tracking
- Every operation gets unique ID (for logging/debugging)
- Phase tracking with timestamps
- Timeout validation (default 10 seconds per phase)

## Configuration Parameters

### ROS Parameters (in config/hybrid_mtc_config.yaml)

```yaml
handover_zone:
  center:
    x: 0.5
    y: 0.0
    z: 0.3
  radius_x: 0.2
  radius_y: 0.2
  radius_z: 0.2
  approach_margin: 0.15
  min_arm_separation: 0.1
  parallel_safe_separation: 0.3  # NEW: For parallel operations
```

## Execution Flow Diagrams

### Sequential Handover Flow
```
┌─────────────────────────────────────────────────────┐
│ Supervisor: initialize_operation(HANDOVER)          │
└────────────────┬────────────────────────────────────┘
                 │
    ┌────────────▼────────────┐
    │ MTC Controller:          │
    │ PHASE = INITIATING       │
    │ MODE = SEQUENTIAL        │
    └────────────┬─────────────┘
                 │
    ┌────────────▼──────────────────────┐
    │ PHASE = ARM1_ACTIVE (AR4)          │
    │ AR4: Pick → Intermediate Position  │
    └────────────┬──────────────────────┘
                 │
    ┌────────────▼──────────────────────┐
    │ PHASE = ARM1_COMPLETE              │
    │ signal_arm1_complete()             │
    │ arm1_completion_cv_.notify_all()   │
    └────────────┬──────────────────────┘
                 │
    ┌────────────▼──────────────────────┐
    │ PHASE = ARM2_ACTIVE (ABB)          │
    │ wait_for_arm1_completion()         │
    │ ABB: Pick from Intermediate → Place│
    └────────────┬──────────────────────┘
                 │
    ┌────────────▼──────────────────────┐
    │ PHASE = ARM2_COMPLETE              │
    │ PHASE = COMPLETION                 │
    │ on_task_complete()                 │
    └────────────────────────────────────┘
```

### Parallel Pick/Place Flow
```
┌──────────────────────────────────────────┐
│ Supervisor: initialize_operation(PICK_PLACE)
└────────────┬─────────────────────────────┘
             │
┌────────────▼─────────────────┐
│ MTC Controller:              │
│ PHASE = INITIATING           │
│ MODE = PARALLEL              │
└────────────┬─────────────────┘
             │
  ┌──────────┴──────────┐
  │                     │
┌─▼────────────┐  ┌────▼──────────┐
│ ARM1 (AR4)   │  │ ARM2 (ABB)     │
│ Simultaneous │  │ Simultaneous   │
│ Execution    │  │ Execution      │
└─┬────────────┘  └────┬───────────┘
  │                    │
  │                    │
  └──────────┬─────────┘
             │
┌────────────▼────────────────┐
│ PHASE = COMPLETION          │
│ on_task_complete()          │
└─────────────────────────────┘
```

## Testing Checklist

- [ ] Handover zones properly detected
- [ ] Sequential handover enforced in handover zone
- [ ] ARM1 completion signal blocks ARM2 until ARM1 ready
- [ ] Parallel operations execute simultaneously in safe zones
- [ ] Collision risk detected (separation < threshold)
- [ ] Mode switching smooth (no deadlocks)
- [ ] Timeout protection working
- [ ] Operation IDs unique and logged correctly
- [ ] Zone transitions logged

## Troubleshooting

### Issue: Sequential handover timeout
**Solution**: Increase timeout value or check if AR4 is stuck
```cpp
if (!mtc_controller.wait_for_arm1_completion(10000)) {  // 10 seconds
    RCLCPP_ERROR(..., "ARM1 timeout!");
}
```

### Issue: Both arms trying to operate in parallel when too close
**Solution**: Increase parallel_safe_separation threshold or check arm positions

### Issue: Mode switching delays
**Solution**: Zone transition callbacks are working, but operation initialization may be slow

## Future Enhancements

1. Add force feedback integration during handover
2. Machine learning for dynamic collision prediction
3. Adaptive separation threshold based on gripper size
4. Multi-object handover coordination
5. Visual confirmation of object transfer during handover

## References

- [control_strategy.hpp](../include/dual_arms_mtc/control_strategy.hpp)
- [hybrid_mtc_controller.hpp](../include/dual_arms_mtc/hybrid_mtc_controller.hpp)
- [mtc_node.cpp](../src/mtc_node.cpp)
- [zone_detection_manager.hpp](../include/dual_arms_mtc/zone_detection_manager.hpp)
- [zone_detection_manager.cpp](../src/zone_detection_manager.cpp)
