# Mock Service Nodes Documentation

This directory contains mock service providers that allow testing the supervisor system without requiring access to the actual detection module, GUI, or grasping pipeline.

## Overview

The mock nodes provide three critical services needed by the supervisor:

### 1. **Mock Detection Node** (`mock_detection_node.py`)
- **Service**: `/detect_bricks`
- **Returns**: List of randomly generated bricks with poses
- **Use Case**: Test supervisor behavior without camera/vision system
- **Features**:
  - Generates 5-7 mock bricks
  - Randomizes brick types and starting positions
  - Alternates between AR4 and ABB as starting arms
  - Provides realistic handover pose

### 2. **Mock GUI Node** (`mock_gui_node.py`)
- **Service**: `/get_assembly_plan`
- **Returns**: Pre-configured assembly sequence (5 bricks)
- **Use Case**: Test supervisor state machine without GUI interface
- **Features**:
  - Default plan with alternating arm pickups
  - Configurable brick types and positions
  - Can be extended to support parameterized plans

### 3. **Mock Grasping Pipeline Node** (`mock_grasping_node.py`)
- **Service**: `/grasp/get_grasp_point`
- **Returns**: Grasp point with pose and quality score
- **Use Case**: Test gripper control without ML model
- **Features**:
  - Generates realistic grasp poses with randomization
  - Quality scores simulating model confidence (0.7-0.99)
  - Stacked brick height simulation

## Usage

### Option 1: Launch All Mock Nodes Together

```bash
cd /home/mariamelsebaey/full_system
source install/setup.bash

# Launch all mock services
ros2 launch supervisor_package mock_services.launch.py
```

This will start all three mock nodes in one command.

### Option 2: Launch Individual Mock Nodes

```bash
cd /home/mariamelsebaey/full_system
source install/setup.bash

# Terminal 1: Mock Detection
ros2 run supervisor_package mock_detection_node

# Terminal 2: Mock GUI
ros2 run supervisor_package mock_gui_node

# Terminal 3: Mock Grasping
ros2 run supervisor_package mock_grasping_node
```

### Option 3: Call Services Manually (Testing)

```bash
# In a new terminal after mocks are running
source install/setup.bash

# Test detection service
ros2 service call /detect_bricks dual_arms_msgs/srv/DetectBricks "{brick_type: 'test'}"

# Test GUI service
ros2 service call /get_assembly_plan supervisor_package/srv/GetAssemblyPlan "{assembly_id: 'test'}"

# Test grasp service
ros2 service call /grasp/get_grasp_point dual_arms_msgs/srv/GetGrasp "{brick_index: '0'}"
```

## Testing with Supervisor

### Complete Testing Flow

```bash
# Terminal 1: Launch mock services
ros2 launch supervisor_package mock_services.launch.py

# Terminal 2: Launch other required nodes (arm controllers, etc.)
# ... launch your other nodes here ...

# Terminal 3: Run supervisor
ros2 run supervisor_package hybrid_supervisor_node
```

The supervisor will:
1. Request assembly plan from mock GUI
2. Request brick detections from mock detection
3. Request grasp points from mock grasping
4. Execute arm movements based on mocked data

## Customizing Mock Behavior

### Modify Detection Output

Edit `mock_detection_node.py`:
- Change `num_bricks` range in `detect_bricks_callback()`
- Modify brick positions in the loop
- Adjust handover pose

### Modify Assembly Plan

Edit `mock_gui_node.py`:
- Modify `_create_default_plan()` to change brick sequence
- Adjust starting positions for AR4/ABB
- Change target stack heights

### Modify Grasp Points

Edit `mock_grasping_node.py`:
- Adjust quality score range in `_generate_grasp_for_brick()`
- Modify pose randomization ranges
- Add specific grasp patterns for testing

## Key Service Definitions

### GetAssemblyPlan
**Request**: `assembly_id` (string)
**Response**: 
- `plan` - array of SuperBrick objects
- `success` - boolean

### DetectBricks
**Request**: `brick_type` (string)
**Response**:
- `bricks` - array of SuperBrick objects
- `handover_pose` - Pose between arms
- `success` - boolean

### GetGrasp
**Request**: `brick_index` (string)
**Response**:
- `grasp_point` - GraspPoint object
- `success` - boolean

## Data Structures

### SuperBrick
```
id (int32)
type (string)
start_side (string) - "AR4" or "ABB"
target_side (string) - "TOP" or "BOTTOM"
pickup_pose (Pose)
place_pose (Pose)
```

### GraspPoint
```
header (Header)
brick_id (int32)
pose (Pose)
quality (float32) - 0.0 to 1.0
```

## Troubleshooting

### Services Not Found
- Ensure mock nodes are running: `ros2 node list`
- Check service availability: `ros2 service list`
- Verify service names match supervisor configuration

### Supervisor Hangs on Service Call
- Check mock node logs for errors
- Verify callback groups are properly configured
- Ensure nodes are using same domain ID

### Unrealistic Brick Poses
- Adjust hardcoded positions in mock files
- Verify frame_id consistency (e.g., "abb_base_link")
- Check transformation setup in supervisor

## Next Steps

Once testing with mocks is successful:
1. Replace mock detection with real vision system
2. Integrate actual GUI interface
3. Deploy ML grasping model
4. Run full system tests with real hardware

## Files

- `mock_detection_node.py` - Detection service provider
- `mock_gui_node.py` - GUI service provider
- `mock_grasping_node.py` - Grasping service provider
- `mock_services.launch.py` - Launch configuration
- `README_MOCKS.md` - This file
