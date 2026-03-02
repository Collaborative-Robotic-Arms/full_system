# Simulation Testing Guide

## Overview

This guide walks you through testing the dual-arm control system in Gazebo simulation before deploying to real hardware.

---

## Prerequisites

### Install Required Packages

```bash
# Gazebo and ROS2 integration
sudo apt install gazebo ros-jazzy-gazebo-* ros-jazzy-gazebo-ros-*

# Build tools
sudo apt install python3-colcon-common-extensions

# Optional: Visualization tools
sudo apt install ros-jazzy-tf2-tools ros-jazzy-rviz2
```

### Build Packages

```bash
cd /home/mariamelsebaey/full_system
colcon build --packages-select supervisor_package dual_arms_mtc assembly_environment
source install/setup.bash
```

---

## Quick Start (5 Minutes)

### Terminal 1: Launch Gazebo
```bash
cd /home/mariamelsebaey/full_system
source install/setup.bash
ros2 launch assembly_environment gazebo.launch.py
```

You should see:
- Gazebo window opens
- Two robot arms visible (AR4 and ABB)
- World with grid and collision objects

### Terminal 2: Launch Supervisor
```bash
cd /home/mariamelsebaey/full_system
source install/setup.bash
ros2 run supervisor_package supervisor_node
```

You should see:
- Supervisor initialization messages
- TF2 buffer created
- Waiting for assembly plans

### Terminal 3: Monitor State
```bash
cd /home/mariamelsebaey/full_system
source install/setup.bash
ros2 topic echo /supervisor_state
```

### Terminal 4: Run Tests
```bash
cd /home/mariamelsebaey/full_system
chmod +x test_simulation.sh
./test_simulation.sh
```

---

## Manual Testing

### Test 1: Handover Operation (Separation < 0.5m)

**Scenario:** Both arms close together, should trigger HANDOVER mode

```bash
# Terminal - Publish AR4 pose (left position)
ros2 topic pub --once /target_pose_ar4 geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: -0.2, z: 0.3}, \
    orientation: {x: 0, y: 0, z: 0, w: 1}}}"

# Terminal - Publish ABB pose (right position, close)
ros2 topic pub --once /target_pose_abb geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.2, z: 0.3}, \
    orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

**Expected Result:**
- Separation: ~0.4m
- Operation Type: **HANDOVER**
- State: DISPATCH → SEQUENTIAL_HANDOVER

**Verify:**
```bash
# Check supervisor logs
grep "Operation Type" ~/.ros/log/*/supervisor*.log

# Watch topic in real-time
ros2 topic echo /supervisor_state | grep -i "handover"
```

---

### Test 2: Sequential Operation (0.5m < Separation < 0.8m)

**Scenario:** Arms at medium distance, should trigger SEQUENTIAL mode

```bash
# Terminal - Publish AR4 pose (left)
ros2 topic pub --once /target_pose_ar4 geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.4, y: -0.25, z: 0.3}, \
    orientation: {x: 0, y: 0, z: 0, w: 1}}}"

# Terminal - Publish ABB pose (right)
ros2 topic pub --once /target_pose_abb geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.6, y: 0.25, z: 0.3}, \
    orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

**Expected Result:**
- Separation: ~0.54m
- Operation Type: **SEQUENTIAL**
- State: DISPATCH → [Based on brick.start_side]

---

### Test 3: Parallel Operation (Separation >= 0.8m)

**Scenario:** Arms far apart, should trigger PARALLEL mode

```bash
# Terminal - Publish AR4 pose (far left)
ros2 topic pub --once /target_pose_ar4 geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.2, y: -0.5, z: 0.3}, \
    orientation: {x: 0, y: 0, z: 0, w: 1}}}"

# Terminal - Publish ABB pose (far right)
ros2 topic pub --once /target_pose_abb geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.8, y: 0.5, z: 0.3}, \
    orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

**Expected Result:**
- Separation: ~1.0m
- Operation Type: **PARALLEL**
- State: DISPATCH → PARALLEL_PICK_PLACE → PARALLEL_PLACE → PROCESS_NEXT

**Verify:**
```bash
# Should see parallel execution messages
grep "Parallel" ~/.ros/log/*/supervisor*.log
```

---

## Monitoring & Debugging

### View Available Topics
```bash
ros2 topic list
```

### Monitor Supervisor State
```bash
ros2 topic echo /supervisor_state
```

### Monitor Operation Type
```bash
ros2 topic echo /current_operation_type
```

### Check TF2 Frames
```bash
# View frame tree
ros2 run tf2_tools view_frames

# Check specific frames
ros2 run tf2_ros tf2_echo world ar4_tool_link
ros2 run tf2_ros tf2_echo world abb_tool_link
```

### View Supervisor Logs
```bash
# Follow real-time logs
tail -f ~/.ros/log/latest/supervisor*.log

# Search for specific operation types
grep "Operation Type:\|State transition:" ~/.ros/log/*/supervisor*.log

# Search for errors
grep "ERROR\|WARN" ~/.ros/log/*/supervisor*.log
```

### Monitor Arm Poses
```bash
# Create a separate terminal to check arm poses
while true; do
  echo "AR4 Position:"
  ros2 topic echo /ar4/tf --no-arr | head -5
  echo ""
  echo "ABB Position:"
  ros2 topic echo /abb/tf --no-arr | head -5
  sleep 2
done
```

---

## Test Scenarios Comparison

| Scenario | AR4 Position | ABB Position | Separation | Expected Mode | States |
|----------|---|---|---|---|---|
| **Handover** | (0.5, -0.2) | (0.5, 0.2) | 0.4m | HANDOVER | 6 |
| **Sequential** | (0.4, -0.3) | (0.6, 0.3) | 0.54m | SEQUENTIAL | Varies |
| **Parallel** | (0.2, -0.5) | (0.8, 0.5) | 1.0m | PARALLEL | 4 |
| **Threshold H** | (0.0, 0.0) | (0.5, 0.0) | 0.5m | HANDOVER | 6 |
| **Threshold P** | (0.0, 0.0) | (0.8, 0.0) | 0.8m | PARALLEL | 4 |

---

## Expected Outputs

### Successful Handover Test
```
supervisor_node: 🔄 SEQUENTIAL HANDOVER detected
supervisor_node: AR4 separation < 0.5m
supervisor_node: State transition: DISPATCH → SEQUENTIAL_HANDOVER
supervisor_node: Starting AR4 Pick...
supervisor_node: Waiting for AR4 completion...
supervisor_node: ABB Pick from intermediate...
supervisor_node: ✅ Handover complete
```

### Successful Parallel Test
```
supervisor_node: ⚡ PARALLEL PICK/PLACE detected
supervisor_node: Arm separation 1.0m (>= 0.8m threshold)
supervisor_node: State transition: DISPATCH → PARALLEL_PICK_PLACE
supervisor_node: ⚡ Starting concurrent pick operations
supervisor_node: AR4 pick in progress...
supervisor_node: ABB pick in progress...
supervisor_node: ✅ Parallel picks complete
supervisor_node: Starting concurrent place operations...
supervisor_node: ✅ Parallel Pick & Place complete
```

---

## Troubleshooting

### Problem: Gazebo Won't Start
```bash
# Check if gazebo is installed
gazebo --version

# Try launching manually
gazebo empty_world.sdf

# Or reinstall
sudo apt remove gazebo
sudo apt install gazebo ros-jazzy-gazebo-*
```

### Problem: Supervisor Not Receiving Poses
```bash
# Check if topics are publishing
ros2 topic list | grep pose

# Monitor incoming poses
ros2 topic echo /target_pose_ar4 --no-arr | head -5

# Check supervisor node status
ros2 node list
ros2 node info /supervisor_node
```

### Problem: Operation Type Not Detected
```bash
# Verify operation type detection is enabled
grep "enable_operation_type_detection" ~/.ros/log/*/supervisor*.log

# Check if it's set to True
# Edit supervisor.py or supervisor_node.py if needed

# Verify TF2 frames exist
ros2 topic list | grep tf

# Test manual frame lookup
ros2 run tf2_ros tf2_echo world ar4_tool_link
```

### Problem: State Transitions Not Working
```bash
# Check for state transition logs
grep "State transition" ~/.ros/log/*/supervisor*.log

# Verify state machine is running
ros2 topic echo /supervisor_state

# Check for exceptions in logs
grep "Exception\|Traceback" ~/.ros/log/*/supervisor*.log
```

---

## Performance Monitoring

### Measure Cycle Times
```bash
# Extract timestamps for each state
grep -E "DISPATCH|SEQUENTIAL_HANDOVER|PARALLEL|PROCESS_NEXT" ~/.ros/log/*/supervisor*.log \
  | while read line; do echo "$(date +%s.%N): $line"; done
```

### Compare Operation Types
```bash
# Count operations
grep "Operation Type:" ~/.ros/log/*/supervisor*.log | sort | uniq -c

# Show distribution
grep "Operation Type: HANDOVER" ~/.ros/log/*/supervisor*.log | wc -l
grep "Operation Type: PARALLEL" ~/.ros/log/*/supervisor*.log | wc -l
grep "Operation Type: SEQUENTIAL" ~/.ros/log/*/supervisor*.log | wc -l
```

---

## Next Steps After Simulation

### 1. Validate Results
- ✅ All three operation modes detected correctly
- ✅ State transitions working as expected
- ✅ Handover is sequential (safety requirement met)
- ✅ Parallel is only used when safe (>= 0.8m)
- ✅ Fallback logic works when parallel disabled

### 2. Adjust Thresholds (if needed)
Edit `supervisor.py` lines 40-50:
```python
# Zone Thresholds
HANDOVER_ZONE_THRESHOLD = 0.5      # Adjust for your workspace
PARALLEL_SAFE_THRESHOLD = 0.8      # Adjust for your workspace
```

### 3. Test with Multiple Bricks
```bash
# Create multiple brick entries in queue
# Verify supervisor processes mixed operation types correctly
```

### 4. Hardware Integration
Once simulation validates:
1. Connect AR4 driver
2. Connect ABB controller (RWS/EGM)
3. Verify TF2 frames publish from real hardware
4. Run same supervisor code (should work unchanged)

---

## Quick Reference

### Common Commands
```bash
# Launch everything
source install/setup.bash
ros2 launch assembly_environment gazebo.launch.py

# Run supervisor
ros2 run supervisor_package supervisor_node

# Test scenarios
./test_simulation.sh

# Monitor operation type
ros2 topic echo /supervisor_state

# Check logs
tail -f ~/.ros/log/latest/supervisor*.log

# View frames
ros2 run tf2_tools view_frames
```

### ROS2 Debugging
```bash
# List nodes
ros2 node list

# List topics
ros2 topic list

# Check topic rate
ros2 topic hz /supervisor_state

# Inspect topic message
ros2 topic echo /supervisor_state --once

# Check TF tree
ros2 run tf2_tools view_frames
```

---

## Success Criteria

✅ **Simulation testing complete when:**

1. Gazebo launches with dual arm setup
2. Supervisor node starts without errors
3. Handover detected when arms < 0.5m apart
4. Sequential detected when arms 0.5-0.8m apart
5. Parallel detected when arms >= 0.8m apart
6. State transitions match expected sequences
7. Operation type logs show correct classifications
8. No exceptions or errors in supervisor logs
9. TF2 frames properly publish
10. Topic messages received and processed correctly

Once all criteria met → **Ready for hardware deployment**

---

## Support

For issues:
1. Check `/home/mariamelsebaey/full_system/QUICK_REFERENCE.md`
2. Review supervisor logs: `grep "Operation Type" ~/.ros/log/*/supervisor*.log`
3. Verify all prerequisites installed
4. Restart nodes and try again
5. Check GitHub/documentation for known issues

---

**Last Updated:** March 2, 2026  
**Status:** Ready for simulation testing
