# Hybrid MTC Implementation Guide

## Quick Start

### 1. Build the System
```bash
cd ~/full_system
colcon build --packages-select dual_arms_mtc dual_arms_msgs supervisor_package
```

### 2. Source the Workspace
```bash
source install/setup.bash
```

### 3. Launch the Hybrid MTC System
```bash
ros2 launch dual_arms_mtc hybrid_mtc_launch.py
```

## Component Description

### A. HybridMTCController (C++)
**Located:** `src/dual_arms_mtc/src/mtc_node.cpp`
**Header:** `include/dual_arms_mtc/hybrid_mtc_controller.hpp`

**Responsibilities:**
- Creates MoveIt Task Constructor tasks for collaborative handover
- Manages solvers and motion planning pipelines
- Switches between control modes based on zone detection
- Provides gripper control interfaces
- Handles task status and failure recovery

**Key Methods:**
```cpp
// Zone detection
bool is_in_handover_zone(const geometry_msgs::msg::Pose& pose);
void switch_to_mtc_mode();
void switch_to_multithreaded_mode();

// Task creation
Task create_collaborative_handover_task(...);
Task create_synchronized_approach_task(...);
Task create_handover_transfer_task(...);

// Gripper control
bool set_ar4_gripper(bool open);
bool set_abb_gripper(bool open);
```

**Configuration Parameters:**
```yaml
handover_zone:
  center: {x: 0.5, y: 0.0, z: 0.3}
  radius: {x: 0.2, y: 0.2, z: 0.2}
  approach_margin: 0.15

motion_planning:
  planner_id: "RRTstar"
  planning_time: 5.0
```

### B. ZoneDetectionManager (C++)
**Located:** `src/dual_arms_mtc/src/zone_detection_manager.cpp`
**Header:** `include/dual_arms_mtc/zone_detection_manager.hpp`

**Responsibilities:**
- Continuously monitors robot poses
- Detects entry/exit of handover zones
- Checks collision risks between arms
- Publishes zone status information
- Triggers callbacks on zone transitions

**Key Methods:**
```cpp
// Zone queries
ZoneType get_zone_type(const geometry_msgs::msg::Pose& pose);
bool is_in_handover_zone(const geometry_msgs::msg::Pose& pose);
bool is_approaching_handover(const geometry_msgs::msg::Pose& pose);

// Dual arm coordination
bool are_both_arms_ready_for_handover(...);
bool check_collision_risk(...);

// Callbacks
void register_zone_transition_callback(ZoneTransitionCallback callback);
```

**Zone States:**
```
SAFE_ZONE       → Normal multithreaded operation
APPROACH_ZONE   → Transition phase, prepare MTC
HANDOVER_ZONE   → Active MTC collaborative control
RETRACT_ZONE    → Transition out of MTC mode
```

### C. HybridAssemblySupervisor (Python)
**Located:** `src/supervisor_package/supervisor_package/hybrid_supervisor_node.py`

**Responsibilities:**
- Orchestrates assembly tasks using state machine
- Detects when bricks require handover (start_side="HANDOVER")
- Triggers automatic mode switching based on zone proximity
- Routes tasks to appropriate controllers
- Handles fallback to standard multithreading if MTC fails

**Key States:**
```
INIT → DETECT → PROCESS_NEXT → [GRASP_PIPELINE]
                                      ↓
                ┌─────────────────────┼─────────────────┐
                ↓                     ↓                 ↓
        EXECUTE_ABB_PICK    MTC_HANDOVER_EXECUTION  EXECUTE_AR4_DIRECT
                ↓                     ↓                 ↓
        EXECUTE_ABB_PLACE    (fast completion)  AR4_PLACE_ON_GRID
                ↓                     ↓                 ↓
                └─────────────────────┼─────────────────┘
                        PROCESS_NEXT (repeat)
```

**Mode Switching Logic:**
```python
# In GRASP_PIPELINE state:
if brick.start_side == "HANDOVER":
    if enable_mtc_mode and detect_handover_proximity(pose):
        await switch_control_mode("MTC_HANDOVER")
        state = "MTC_HANDOVER_EXECUTION"
    else:
        state = "HANDOVER_SEQUENCE"  # fallback
```

## Integration Points

### ROS 2 Services Created

#### 1. GetHandoverZone
**Service:** `/zone_detection/get_handover_zone`
```
Request: (empty)
Response:
  - zone_center: geometry_msgs/Pose
  - radius_x/y/z: float64
  - approach_margin: float64
  - zone_active: bool
```

#### 2. ExecuteMTCHandover
**Service:** `/mtc_controller/execute_handover`
```
Request:
  - ar4_start_pose: geometry_msgs/Pose
  - abb_start_pose: geometry_msgs/Pose
  - handover_pose: geometry_msgs/Pose
  - object_id: string
  
Response:
  - success: bool
  - status_message: string
  - execution_id: string
```

### Published Topics

- `/zone_status` (std_msgs/String)
  - Real-time zone status for monitoring
  - Format: "AR4: HANDOVER | ABB: APPROACH"

### Subscribed Topics

- Joint states from both arm drivers
- Gripper state feedback
- Object detection results

## Configuration Files

### hybrid_mtc_config.yaml
**Location:** `src/dual_arms_mtc/config/hybrid_mtc_config.yaml`

Contains all tunable parameters for:
- Handover zone dimensions and location
- Motion planning parameters
- Synchronization thresholds
- Safety constraints
- Task routing logic
- Diagnostics settings

**How to Edit:**
1. Adjust zone center to your handover point
2. Set zone radius based on robot workspace
3. Configure planning parameters for your hardware
4. Enable/disable diagnostics as needed

```bash
# Edit before building
nano src/dual_arms_mtc/config/hybrid_mtc_config.yaml

# Or override at launch time
ros2 launch dual_arms_mtc hybrid_mtc_launch.py \
  handover_zone.center.x:=0.45 \
  handover_zone.center.y:=0.05
```

## Testing the System

### Test 1: Zone Detection
```bash
# Terminal 1: Start MTC system
ros2 launch dual_arms_mtc hybrid_mtc_launch.py

# Terminal 2: Monitor zones
ros2 topic echo /zone_status

# Terminal 3: Move AR4 near handover zone
# (via separate control commands)
# Watch zone_status change: SAFE → APPROACH → HANDOVER
```

### Test 2: Mode Switching
```bash
# Monitor control mode in supervisor logs
ros2 run rqt_console rqt_console
# Filter to "hybrid_supervisor"
# Watch for "Control mode switched" messages
```

### Test 3: MTC Handover Execution
```bash
# Trigger a handover task through the assembly plan
ros2 service call /assembly_plan_service ...

# Monitor hybrid_supervisor output:
# "MTC-based collaborative handover..."
# "Object handed off successfully"
```

### Test 4: Fallback Behavior
```bash
# Kill MTC controller mid-task
pkill -f hybrid_mtc_controller

# Supervisor should:
# 1. Detect service unavailable
# 2. Switch to standard multithreaded mode
# 3. Continue with basic handover
```

## Troubleshooting Guide

### Issue: "MTC handover service not available"
**Diagnosis:**
```bash
ros2 service list | grep handover
# Should show: /mtc_controller/execute_handover
```

**Solution:**
1. Verify hybrid_mtc_controller is running:
   ```bash
   ros2 node list | grep mtc
   ```
2. Check for compilation errors:
   ```bash
   colcon build --packages-select dual_arms_mtc
   ```
3. Rebuild without cache:
   ```bash
   colcon build --packages-select dual_arms_mtc --no-cache-dir
   ```

### Issue: "Handover zone dimensions incorrect"
**Solution:**
1. Verify current config:
   ```bash
   ros2 param get /zone_detection_manager handover_zone.center.x
   ```
2. Update in hybrid_mtc_config.yaml
3. Restart nodes:
   ```bash
   ros2 service call /zone_detection_manager/set_handover_zone_config
   ```

### Issue: MTC planning timeout
**Solution:**
1. Increase planning time in config:
   ```yaml
   motion_planning:
     planning_time: 10.0  # Increase from 5.0
   ```
2. Reduce zone dimensions (easier to plan)
3. Add more approach margin for better planning

### Issue: "Mode keeps switching in/out"
**Solution:**
1. Increase approach_margin in config
2. Add pose filtering to supervisor
3. Increase distance_threshold before mode switch

## Advanced Configuration

### Custom Motion Planners

To use a different motion planner (e.g., OMPL):

```yaml
motion_planning:
  planner_id: "RRTConnect"  # Instead of RRTstar
  planning_time: 3.0
```

Available planners depend on your MoveIt installation.

### Custom Gripper Timing

Adjust gripper synchronization delays:

```yaml
handover_sequence:
  gripper_sync:
    ar4_open_delay_ms: 150    # Increase if gripper slow to open
    abb_close_delay_ms: 50    # Adjust for ABB gripper response
```

### Performance Tuning

For faster execution:
```yaml
motion_planning:
  max_velocity_scaling: 0.5   # Increase from 0.3 (faster)
  max_acceleration_scaling: 0.4  # Increase from 0.2 (faster)
```

For safer execution:
```yaml
motion_planning:
  max_velocity_scaling: 0.1   # Decrease for smoother motions
  max_acceleration_scaling: 0.1
```

## Deployment Checklist

- [ ] All packages compile without errors
- [ ] Hardware interfaces configured (AR4 and ABB drivers running)
- [ ] MoveIt planning scene properly set up
- [ ] Handover zone center measured and configured
- [ ] Zone dimensions tested with manual operations
- [ ] Safety limits verified in config
- [ ] Gripper control services working
- [ ] Gripper timing tuned for smooth handover
- [ ] Zone status topic monitored during operation
- [ ] Fallback procedures tested and working
- [ ] Emergency stop tested
- [ ] Documentation reviewed by team
- [ ] First run done under supervision

## Support and Debugging

### Enable Debug Logging
```bash
# Edit supervisor launch command
ros2 run supervisor_package hybrid_supervisor_node \
  --ros-args --log-level DEBUG
```

### RViz Visualization
```bash
# Show MTC planning results and zone visualization
ros2 run rviz2 rviz2 -d src/dual_arms_mtc/rviz/hybrid_mtc.rviz
```

### Performance Profiling
```bash
# Record timing statistics
ros2 bag record \
  /mtc_controller/task_status \
  /zone_status \
  /hybrid_supervisor/mode_switch
```

## References

- [MoveIt Task Constructor Documentation](https://moveit.ros.org/docs/concepts/task_constructor/)
- [ROS 2 Services and Clients](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service.html)
- [TF2 Transforms](http://wiki.ros.org/tf2)
