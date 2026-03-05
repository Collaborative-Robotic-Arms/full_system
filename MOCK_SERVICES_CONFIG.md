# Mock Services Configuration Guide

This guide shows how to manage mock vs. real service providers for flexible testing.

## Current Setup

The system is configured to work with mock services that replace:
- Vision/Detection System → `mock_detection_node`
- GUI Interface → `mock_gui_node`
- Grasping Pipeline → `mock_grasping_node`

## Service Mapping

```
Supervisor Request          →    Current Provider
─────────────────────────────────────────────────
/detect_bricks              ←    mock_detection_node
/get_assembly_plan          ←    mock_gui_node
/grasp/get_grasp_point      ←    mock_grasping_node
```

## Testing Workflow

### Current: With All Mocks

```bash
# Terminal 1: All mocks
ros2 launch supervisor_package mock_services.launch.py

# Terminal 2: Supervisor
ros2 run supervisor_package hybrid_supervisor_node
```

**Status**: ✅ Complete system available, no real hardware needed

---

### Hybrid 1: Real Detection + Mocked GUI/Grasping

When your detection system is ready:

```bash
# Terminal 1: Real detection + mock GUI/Grasping
ros2 run brick_detection detection_node &
ros2 run supervisor_package mock_gui_node &
ros2 run supervisor_package mock_grasping_node &

# Terminal 2: Supervisor
ros2 run supervisor_package hybrid_supervisor_node
```

This tests supervisor with real brick data but mocked planning/grasping.

---

### Hybrid 2: Real Detection + Real Grasping + Mocked GUI

```bash
# Terminal 1
ros2 run brick_detection detection_node &
ros2 run brick_grasping_model grasping_node &
ros2 run supervisor_package mock_gui_node &

# Terminal 2: Supervisor
ros2 run supervisor_package hybrid_supervisor_node
```

This tests real perception pipeline before integrating GUI.

---

### Final: All Real Services

When all components are available:

```bash
# Terminal 1: All real services
ros2 run brick_detection detection_node &
ros2 run gui_package gui_node &
ros2 run brick_grasping_model grasping_node &

# Terminal 2: Supervisor
ros2 run supervisor_package hybrid_supervisor_node
```

**Status**: ✅ Full system with real components

## Mock Service Details

### Detect Bricks Service

**Topic**: `/detect_bricks`
**Type**: `dual_arms_msgs/srv/DetectBricks`

**Mock Implementation**:
```python
# File: mock_detection_node.py
# Returns: 5-7 randomly positioned bricks
# Uses: Hardcoded poses with randomization
```

**Real Implementation** (when available):
```python
# File: brick_detection/detection_node.py
# Returns: Actual bricks detected by camera
# Uses: Vision system + YOLO
```

**No Changes Needed**: Supervisor calls same service, automatically uses available provider.

---

### Get Assembly Plan Service

**Topic**: `/get_assembly_plan`
**Type**: `supervisor_package/srv/GetAssemblyPlan`

**Mock Implementation**:
```python
# File: mock_gui_node.py
# Returns: Pre-configured 5-brick assembly sequence
# Uses: Hardcoded default plan
```

**Real Implementation** (when available):
```python
# File: gui_package/gui_node.py
# Returns: Plan created by user in GUI
# Uses: Web interface + database
```

**No Changes Needed**: Just replace mock node with real GUI node.

---

### Get Grasp Point Service

**Topic**: `/grasp/get_grasp_point`
**Type**: `dual_arms_msgs/srv/GetGrasp`

**Mock Implementation**:
```python
# File: mock_grasping_node.py
# Returns: Random grasp with 0.7-0.99 quality
# Uses: Simulated grasping model
```

**Real Implementation** (when available):
```python
# File: brick_grasping_model/grasping_node.py
# Returns: CNN-predicted grasp
# Uses: TensorFlow/PyTorch model
```

**No Changes Needed**: Same service interface, quality scores may vary.

## Performance Notes

### Mock Services (Current)

- **Response Time**: < 5ms per service call
- **Consistency**: Exactly reproducible (with seed control)
- **Reliability**: 100% - no hardware dependencies
- **Use Case**: Development, testing, CI/CD

### Real Services (Future)

- **Response Time**: Varies (vision: 50-200ms, grasping: 100-500ms)
- **Consistency**: Varies based on environmental conditions
- **Reliability**: Hardware-dependent (lighting, camera issues, etc.)
- **Use Case**: Deployment, production

## Migration Checklist

When integrating real components:

- [ ] Verify real service uses same topic name
- [ ] Verify same service type (messages must match)
- [ ] Check response time compatibility
- [ ] Add error handling for real hardware failures
- [ ] Test hybrid combinations first
- [ ] Validate data consistency
- [ ] Update launch files
- [ ] Remove mock nodes from production launch

## Service Interface Compatibility

**Critical**: Supervisor makes NO assumptions about implementation.

```python
# Supervisor code (unchanged)
result = await self.camera_client.call_async(req)

# Works with BOTH:
# - mock_detection_node (instant response)
# - real detection_node (50-200ms response)
```

This means you can:
1. Start development with all mocks
2. Gradually replace mocks with real components
3. No supervisor code changes needed
4. Easy rollback if real component has issues

## Validation Testing

When replacing a mock with real component:

```bash
# 1. Start real + other mocks
ros2 run real_component real_node &
ros2 launch supervisor_package mock_services.launch.py &

# 2. Manually test the service
ros2 service call /service_name package/srv/Service "{params}"

# 3. Run supervisor
ros2 run supervisor_package hybrid_supervisor_node

# 4. Verify output logs match expectations
```

## Rollback Procedure

If real component has issues:

```bash
# 1. Stop supervisor
pkill -f hybrid_supervisor_node

# 2. Stop real component
pkill -f real_node

# 3. Restart with mock
ros2 launch supervisor_package mock_services.launch.py &
ros2 run supervisor_package hybrid_supervisor_node
```

Takes ~5 seconds to rollback to stable mocks.

## Future Enhancements

Consider adding to mock services:

1. **Failure Injection**: Toggle service failures to test error handling
2. **Latency Simulation**: Artificial delays to match real hardware timing
3. **Data Recording**: Save/replay real data while developing
4. **Randomization Control**: Seed-based reproducibility
5. **Parameter Tuning**: Change brick count, pose distribution, etc. at runtime

---

## Questions?

- Mock Implementation: See individual `mock_*_node.py` files
- Launch Configuration: See `launch/mock_services.launch.py`
- Full Documentation: See `README_MOCKS.md`

