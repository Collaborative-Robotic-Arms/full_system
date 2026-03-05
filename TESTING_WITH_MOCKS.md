# Testing System with Mock Services

This guide explains how to test your dual-arm control system using mock services for the detection module, GUI, and grasping pipeline.

## Quick Start

### 1. Build the Workspace

```bash
cd /home/mariamelsebaey/full_system
colcon build --packages-select supervisor_package
source install/setup.bash
```

### 2. Launch Mock Services

Open a terminal and run:

```bash
cd /home/mariamelsebaey/full_system
bash test_with_mocks.sh
```

Or manually:

```bash
ros2 launch supervisor_package mock_services.launch.py
```

### 3. Test Supervisor in Another Terminal

```bash
cd /home/mariamelsebaey/full_system
source install/setup.bash
ros2 run supervisor_package hybrid_supervisor_node
```

## What's Included

I've created three mock service nodes that replace missing components:

### Mock Detection Node (`mock_detection_node.py`)
- **Service**: `/detect_bricks`
- **What it does**: Generates 5-7 randomly positioned bricks
- **Replaces**: Your vision/detection system
- **Output**: Realistic brick positions with types and handover pose

### Mock GUI Node (`mock_gui_node.py`)
- **Service**: `/get_assembly_plan`
- **What it does**: Creates a pre-configured assembly sequence (5 bricks)
- **Replaces**: Your GUI interface
- **Output**: Assembly plan with pickup/placement poses for each brick

### Mock Grasping Node (`mock_grasping_node.py`)
- **Service**: `/grasp/get_grasp_point`
- **What it does**: Generates grasp points with quality scores
- **Replaces**: Your grasping ML model
- **Output**: Realistic grasp poses with 0.7-0.99 confidence scores

## Testing Scenarios

### Scenario 1: Quick Service Check

Verify all mocks are working:

```bash
# Terminal 1
ros2 launch supervisor_package mock_services.launch.py

# Terminal 2
ros2 service list | grep -E "detect_bricks|get_assembly_plan|get_grasp_point"
```

Expected output:
```
/detect_bricks
/get_assembly_plan
/grasp/get_grasp_point
```

### Scenario 2: Manual Service Testing

```bash
# Terminal 1
ros2 launch supervisor_package mock_services.launch.py

# Terminal 2
# Test detection
ros2 service call /detect_bricks dual_arms_msgs/srv/DetectBricks "{brick_type: 'test'}"

# Test GUI plan
ros2 service call /get_assembly_plan supervisor_package/srv/GetAssemblyPlan "{assembly_id: 'test'}"

# Test grasping
ros2 service call /grasp/get_grasp_point dual_arms_msgs/srv/GetGrasp "{brick_index: '0'}"
```

### Scenario 3: Full Supervisor Test

This tests the complete supervisor state machine with mocks:

```bash
# Terminal 1: Launch mocks
ros2 launch supervisor_package mock_services.launch.py

# Terminal 2: Other required nodes (if needed)
# ros2 launch [your_other_packages]

# Terminal 3: Run supervisor
ros2 run supervisor_package hybrid_supervisor_node
```

The supervisor should:
1. ✅ Request assembly plan → Get 5 bricks
2. ✅ Request brick detection → Get randomized bricks
3. ✅ For each brick, request grasp point
4. ✅ Execute arm movements based on mocked data

### Scenario 4: Run Existing Tests

```bash
cd /home/mariamelsebaey/full_system
bash run_tests.sh
```

Your unit tests should pass with mocked data.

## File Locations

- **Mock Nodes**: `/home/mariamelsebaey/full_system/src/supervisor_package/supervisor_package/`
  - `mock_detection_node.py`
  - `mock_gui_node.py`
  - `mock_grasping_node.py`

- **Launch File**: `/home/mariamelsebaey/full_system/src/supervisor_package/launch/mock_services.launch.py`

- **Documentation**: `/home/mariamelsebaey/full_system/src/supervisor_package/README_MOCKS.md`

- **Quick Start Script**: `/home/mariamelsebaey/full_system/test_with_mocks.sh`

## Customizing Mock Behavior

### Change Number of Bricks

Edit `mock_detection_node.py`, function `detect_bricks_callback()`:

```python
num_bricks = random.randint(5, 7)  # Change these numbers
```

### Modify Assembly Sequence

Edit `mock_gui_node.py`, function `_create_default_plan()`:

```python
for i in range(5):  # Change brick count
    brick.start_side = "AR4" if i % 2 == 0 else "ABB"  # Modify pattern
    brick.target_side = "TOP" if i >= 3 else "BOTTOM"  # Change positions
```

### Adjust Grasp Quality Range

Edit `mock_grasping_node.py`, function `_generate_grasp_for_brick()`:

```python
grasp.quality = random.uniform(0.7, 0.99)  # Change confidence range
```

## Troubleshooting

### "Service not available" error

1. Check nodes are running:
   ```bash
   ros2 node list | grep mock
   ```
   Expected output:
   ```
   /mock_detection_node
   /mock_gui_node
   /mock_grasping_node
   ```

2. Check services are advertised:
   ```bash
   ros2 service list | grep -E "detect|grasp|plan"
   ```

### ROS2 domain issues

Ensure all terminals use same domain:

```bash
# In all terminals
export ROS_DOMAIN_ID=0
source install/setup.bash
```

### Build errors

Rebuild the package:

```bash
cd /home/mariamelsebaey/full_system
colcon build --packages-select supervisor_package
source install/setup.bash
```

## Integration Path

Once you have this working with mocks:

1. **Phase 1** (Current): Test supervisor logic with mocks ✅
2. **Phase 2**: Replace mock_detection_node with real camera node
3. **Phase 3**: Replace mock_gui_node with actual GUI
4. **Phase 4**: Replace mock_grasping_node with ML model
5. **Phase 5**: Full system test with real hardware

## Next Steps

- [ ] Build the supervisor_package
- [ ] Launch mock services
- [ ] Verify service availability
- [ ] Run supervisor with mocks
- [ ] Test complete workflow
- [ ] Customize mock data for your scenarios
- [ ] Integrate real components as they become available

## Questions?

Refer to:
- `README_MOCKS.md` - Detailed mock documentation
- Individual mock node files - Code comments explain each section
- ROS2 documentation: https://docs.ros.org/en/humble/

