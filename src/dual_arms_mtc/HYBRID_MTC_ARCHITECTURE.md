# Hybrid MTC Controller Architecture

## Overview

The **Hybrid MTC Controller** is a sophisticated system that dynamically switches between multithreaded control and MoveIt Task Constructor (MTC) based collaborative control depending on operational context. This design enables efficient handling of both general manipulation tasks (using fast multithreaded execution) and complex collaborative handover tasks (using coordinated MTC-based planning).

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Hybrid Assembly Supervisor                       │
│  - State machine for task orchestration                             │
│  - Zone detection and mode switching                                │
│  - Task routing (AR4, ABB, or Collaborative)                       │
└──────────────────────────┬──────────────────────────────────────┬───┘
                           │                                      │
         ┌─────────────────▼──────────────┐     ┌───────────────▼────────┐
         │   Standard Multithreaded Mode   │     │  MTC Collaborative Mode │
         ├─────────────────────────────────┤     ├──────────────────────────┤
         │ • Point control (AR4)           │     │ • Zone Detection Manager │
         │ • Visual servoing               │     │ • Hybrid MTC Controller  │
         │ • Pick/Place on single arm      │     │ • Synchronized planning  │
         │ • Fast independent movements    │     │ • Gripper coordination   │
         │ • Communication overhead: LOW   │     │ • Collision avoidance    │
         │ • Processing time: ~100ms/op    │     │ • Communication: MEDIUM  │
         │                                 │     │ • Processing: ~500ms/plan│
         └─────────────────────────────────┘     └──────────────────────────┘
                           │                                      │
                           │                                      │
         ┌─────────────────▼──────────────┐     ┌───────────────▼────────┐
         │    AR4 Controller Node           │     │    MoveIt Task Executor │
         │    ABB Controller Node           │     │    (Task Constructor)   │
         │                                 │     │                        │
         │    Sends to robot drivers       │     │  Generates motion plans │
         │    via direct IK/trajectory     │     │  for both arms together │
         └─────────────────────────────────┘     └──────────────────────────┘
                           │                                      │
                           └───────────────┬─────────────────────┘
                                          │
                    ┌─────────────────────▼──────────────────┐
                    │      Robot Hardware                    │
                    │  • ABB IRB1200                        │
                    │  • AR4 Industrial Robot               │
                    │  • Grippers with sensors              │
                    └──────────────────────────────────────┘
```

## Control Modes

### 1. Multithreaded Mode (Default)
**Used for:** Single-arm tasks, general manipulation
- **Characteristics:**
  - Fast, responsive control (~100ms cycle time)
  - Independent arm control via separate threads
  - Low computational overhead
  - Direct trajectory generation
  
- **When Active:**
  - Standard pick/place operations
  - Single arm assembly tasks
  - AR4 or ABB operating independently

### 2. MTC Collaborative Mode
**Used for:** Dual-arm handover, synchronized assembly
- **Characteristics:**
  - Coordinated motion planning for both arms
  - Complete collision avoidance
  - Gripper synchronization
  - Higher planning time (~500ms) but safer execution
  
- **When Active:**
  - Automatic when entering handover zone
  - Both arms must be synchronized
  - Transfer of objects between grippers
  - Complex collaborative assembly

## Zone-Based Switching Logic

```
                    ┌──────────────────────┐
                    │   SAFE ZONE          │
                    │ (Multithreaded Mode) │
                    │                      │
        ┌───────────┼──────────────────────┼────────────┐
        │           │                      │            │
        │    ┌──────▼──────┐              │            │
        │    │ APPROACH    │              │            │
        │    │ ZONE        │              │            │
        │    │ (Transition)│              │            │
        │    │             │              │            │
        │    │  Triggers   │              │            │
        │    │  MTC Mode   │              │            │
        │    │             │              │            │
        │    └──────┬──────┘              │            │
        │           │                     │            │
        │           ▼                     │            │
        │    ┌──────────────┐             │            │
        │    │ HANDOVER     │             │            │
        │    │ ZONE         │             │            │
        │    │ (Active MTC) │             │            │
        │    │              │             │            │
        │    │ Both arms    │             │            │
        │    │ synchronized │             │            │
        │    │ Grippers     │             │            │
        │    │ coordinated  │             │            │
        │    └──────┬───────┘             │            │
        │           │                     │            │
        │    ┌──────▼──────┐              │            │
        │    │ RETRACT     │              │            │
        │    │ ZONE        │              │            │
        │    │ (Transition)│              │            │
        │    └──────┬──────┘              │            │
        │           │                     │            │
        └───────────┴─────►SAFE ZONE──────┴────────────┘
        (Back to standard mode)
```

### Zone Configuration Parameters

```yaml
Handover Zone Center: (0.5, 0.0, 0.3)m  # Meeting point of both arms

Zone Dimensions:
  - Radius X: ±0.20m
  - Radius Y: ±0.20m
  - Radius Z: ±0.20m

Approach Margin: 0.15m                  # Distance before critical zone
Min Arm Separation: 0.10m                # Safety distance between grippers
```

## Handover Sequence

The MTC-based handover executes in five stages:

### Stage 1: Synchronized Approach
```
    AR4 (Top)           ABB (Bottom)
      ▼                    ▲
      │                    │
      └──────▼──────◄──────┘
         Handover Point
         (0.5, 0.0, 0.3)
```
- Both arms plan trajectories to approach the handover point
- Planning accounts for collision avoidance
- Movements are synchronized (within 100ms tolerance)

### Stage 2: Pre-gripper Adjustment
```
      AR4          ABB
      ▼             ▲
      │ (open)      │ (ready to close)
      └─────②──────┘
      Hand-off position
```
- AR4 gripper prepares to release
- ABB gripper positions for capture

### Stage 3: Object Hand-off
```
      ┌─────① ②─────┐
      │               │
      ▼ AR4  ABB ▲
      ├───●───┤     Physical contact
                    of grippers without
                    collision
```
- Object detached from AR4 gripper
- Object attached to ABB gripper
- Minimal spatial clearance required

### Stage 4: Gripper Synchronization
```
   Status Check:     Timing:
   AR4: OPENED       t0 → open AR4
   ABB: CLOSED       t0+100ms → close ABB
   Brick: STABLE     Verify attachment
```
- AR4 completely opens (release)
- ABB closes and grasps the object
- Force feedback confirms secure grip

### Stage 5: Retract and Separate
```
      AR4 ▲          ▼ ABB
      │              │
      └──────③───────┘
         Separation Phase
```
- AR4 moves away (upward)
- ABB holds object and moves to place pose
- Both arms return to ready states

## Cross-Component Communication

### Supervisor ↔ MTC Controller

**Service: `/mtc_controller/execute_handover`**
```
Request:
  - ar4_start_pose: Current AR4 TCP position
  - abb_start_pose: Current ABB TCP position
  - handover_pose: Meeting point coordinates
  - object_id: Type of object being transferred

Response:
  - success: Boolean (true = executed successfully)
  - status_message: Description of result
  - execution_id: Unique identifier for tracking
```

**Service: `/zone_detection/get_handover_zone`**
```
Request: (empty)

Response:
  - zone_center: Geometry pose
  - radius_x/y/z: Zone dimensions
  - approach_margin: Entry threshold
  - zone_active: Currently enabled/disabled
```

### MTC Controller ↔ Robot Controllers

**Action: `/ar4_controller/execute_task`**
- Sends trajectory goals to AR4 driver
- Receives joint state feedback
- Collision checking during execution

**Action: `/abb_controller/execute_task`**
- Sends trajectory goals to ABB driver
- Receives joint state feedback
- Force/torque monitoring

### Gripper Service Calls

**Service: `/ar4_gripper/set`**
```
Request: data (boolean)
  - true: OPEN gripper
  - false: CLOSE gripper

Service: `/abb_gripper/set`
  (identical interface)
```

## Decision Logic Flow

```
┌─ Supervisor detects brick location ─┐
│                                     │
├─ Is brick for HANDOVER? ───NO─┐     
│                              │     
YES                             │     
│                              │     
├─ Enable MTC? ────NO──┐        │     
│                      │        │     
YES                    │        │     
│                      │        │     
├─ Get current poses ──┤        │     
│                      │        │     
├─ Calculate distance  │        │     
│  to handover zone    │        │     
│                      │        │     
├─ Distance ≤ MARGIN? │        │     
│     │       │        │        │     
│    YES     NO        │        │     
│     │       │        │        │     
│     ├──────►│        │        │     
│     │  Query │        │        │     
│     │  zone  │        │        │     
│     │  config│        │        │     
│     │        │        │        │     
│     └────────┼────────┤        │     
│              │        │        │     
│         Switch to ────┼────────┤     
│         MTC Mode  ────┤        │     
│              │        │        │     
│         Call MTC   Std │       │     
│        Handover    Multi-      │     
│        Service    thread       │     
│              │    Mode ◄───────┤     
│              │        │        │     
│              └────┬───┘        │     
│                   │            │     
│              Continue ◄────────┘     
│              Assembly
└────────────────────────────────────┘
```

## Implementation Features

### 1. Seamless Mode Switching
- Supervisor automatically detects zone entry/exit
- MTC mode activated before handover begins
- Fallback to multithreaded mode if MTC fails
- No manual intervention required

### 2. Safety Mechanisms
- **Collision Avoidance:** Full planning with collision checking
- **Force Limits:** Gripper forces monitored during transfer
- **Separation Verification:** Ensures arms don't collide
- **Timeout Protection:** Operations abort if exceeding max duration
- **Emergency Stop:** Can halt execution immediately

### 3. Performance Optimization
- **Parallel Planning:** Both arm trajectories planned simultaneously
- **Caching:** Previous successful plans can be reused
- **Adaptive Timing:** Gripper sync delays adjust based on arm speeds
- **Trajectory Smoothing:** Reduces acceleration loads

### 4. Diagnostics & Monitoring
- Real-time zone status publishing
- Arm separation distance tracking
- Task execution timing statistics
- Failure reason logging
- Visualization of planning results

## Configuration and Tuning

### Critical Parameters

1. **Handover Zone Dimensions**
   - Too small: Frequent planning failures
   - Too large: Inefficient, over-constrained planning
   - Typical: 0.2m radius (40cm diameter)

2. **Approach Margin**
   - Too small: Late switching, less planning time
   - Too large: Unnecessary MTC overhead
   - Typical: 0.15m (15cm)

3. **Minimum Arm Separation**
   - Too small: Collision risk during handover
   - Too large: Less close manipulation
   - Typical: 0.10m (10cm)

4. **Planning Time**
   - Too short: Failed plans or suboptimal trajectories
   - Too long: Delays in task execution
   - Typical: 3-5 seconds

### Tuning Procedure

1. **Measure actual handover point** during manual operation
2. **Set zone center** to that point
3. **Start with conservative zone size** (0.2m)
4. **Increase margins** if planning fails
5. **Decrease margins** if execution is inefficient
6. **Enable collision checking** and verify no contacts
7. **Record successful parameters** for future runs

## Example Usage

### Launch the Hybrid System
```bash
ros2 launch dual_arms_mtc hybrid_mtc_launch.py
```

### Monitor Zone Status
```bash
ros2 topic echo /zone_status
```

### Watch Control Mode Switching
```bash
ros2 service call /hybrid_supervisor/get_control_mode
```

### Execute Assembly Task
```bash
ros2 action send_goal /hybrid_supervisor/execute_assembly_plan ...
```

## Troubleshooting

### Frequent Mode Switching
- **Problem:** Control mode keeps switching in/out of MTC
- **Cause:** Zone boundaries too small or pose noise
- **Solution:** Increase approach margin, add pose filtering

### MTC Planning Failures
- **Problem:** Handover fails with "planning failed"
- **Cause:** Insufficient workspace overlap or collision at handover point
- **Solution:** Adjust handover pose, increase zone dimensions

### Gripper Synchronization Issues
- **Problem:** Object drops during handover
- **Cause:** Timing delays, gripper speed differences
- **Solution:** Increase delays, verify gripper responses

### Collision Warnings
- **Problem:** Arms nearly collide during approach
- **Cause:** Zone too small or trajectory overlap
- **Solution:** Increase minimum separation distance, adjust zone center

## Future Enhancements

1. **Learning-based Zone Optimization**
   - Adaptive zone sizing based on success rates
   - ML-based optimal handover point prediction

2. **Adaptive Planning**
   - Different planning strategies for different object types
   - Force feedback-guided trajectory modification

3. **Multi-object Coordination**
   - Sequential handovers
   - Parallel assembly with constraint satisfaction

4. **Human Collaboration**
   - Shared autonomy in handover tasks
   - Haptic feedback for operator guidance
