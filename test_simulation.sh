#!/bin/bash
# Simulation Testing Script
# Publishes test poses to verify operation type detection

WORKSPACE="/home/mariamelsebaey/full_system"
cd "$WORKSPACE"
source install/setup.bash

echo ""
echo "╔════════════════════════════════════════════════════════════════════════╗"
echo "║     DUAL-ARM CONTROL SYSTEM - SIMULATION TEST SCENARIOS               ║"
echo "╚════════════════════════════════════════════════════════════════════════╝"
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Function to run test scenario
run_scenario() {
    local test_num=$1
    local test_name=$2
    local ar4_x=$3
    local ar4_y=$4
    local abb_x=$5
    local abb_y=$6
    local expected=$7
    
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}TEST $test_num: $test_name${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "AR4 Position:  ($ar4_x, $ar4_y, 0.3)"
    echo "ABB Position:  ($abb_x, $abb_y, 0.3)"
    echo "Expected:      $expected"
    echo ""
    echo "Publishing poses..."
    
    # Publish AR4 pose
    ros2 topic pub --once /target_pose_ar4 geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'world'}, pose: {position: {x: $ar4_x, y: $ar4_y, z: 0.3}, orientation: {x: 0, y: 0, z: 0, w: 1}}}" &
    
    # Publish ABB pose
    ros2 topic pub --once /target_pose_abb geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'world'}, pose: {position: {x: $abb_x, y: $abb_y, z: 0.3}, orientation: {x: 0, y: 0, z: 0, w: 1}}}" &
    
    wait
    
    # Calculate separation
    dx=$(echo "$abb_x - $ar4_x" | bc)
    dy=$(echo "$abb_y - $ar4_y" | bc)
    sep=$(echo "sqrt($dx * $dx + $dy * $dy)" | bc -l)
    echo ""
    echo -e "Calculated separation: ${GREEN}${sep:0:5}m${NC}"
    echo ""
    echo "Monitor /supervisor_state topic to verify operation type:"
    echo "  $ ros2 topic echo /supervisor_state"
    echo ""
    read -p "Press ENTER after supervisor processes the poses..."
}

echo "Make sure:"
echo "  1. Gazebo is running with dual arms"
echo "  2. Supervisor node is running"
echo "  3. Topic monitor is open in another terminal: ros2 topic echo /supervisor_state"
echo ""
read -p "Continue with tests? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

# Test 1: Handover (close arms)
run_scenario 1 \
    "HANDOVER - Arms Close (0.4m)" \
    "0.5" "-0.2" \
    "0.5" "0.2" \
    "HANDOVER (separation < 0.5m)"

# Test 2: Sequential (middle distance)
run_scenario 2 \
    "SEQUENTIAL - Arms Medium Distance (0.6m)" \
    "0.4" "-0.3" \
    "0.6" "0.3" \
    "SEQUENTIAL (0.5m < separation < 0.8m)"

# Test 3: Parallel (far apart)
run_scenario 3 \
    "PARALLEL - Arms Far Apart (1.2m)" \
    "0.2" "-0.5" \
    "0.8" "0.5" \
    "PARALLEL (separation >= 0.8m)"

# Test 4: Handover at exact threshold
run_scenario 4 \
    "HANDOVER - At Threshold (exactly 0.5m)" \
    "0.0" "0.0" \
    "0.5" "0.0" \
    "HANDOVER (at 0.5m threshold)"

# Test 5: Parallel at exact threshold
run_scenario 5 \
    "PARALLEL - At Threshold (exactly 0.8m)" \
    "0.0" "0.0" \
    "0.8" "0.0" \
    "PARALLEL (at 0.8m threshold)"

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                    ALL TEST SCENARIOS COMPLETE                         ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "Results Interpretation:"
echo "  ✅ If operation type matches expected → TEST PASSED"
echo "  ❌ If operation type different → DEBUG AND FIX"
echo ""
echo "Next steps:"
echo "  1. Check supervisor logs: grep 'Operation Type' ~/.ros/log/*/supervisor*.log"
echo "  2. Verify TF2 frames: ros2 run tf2_tools view_frames"
echo "  3. Monitor arm controller output"
echo ""
