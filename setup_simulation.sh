#!/bin/bash
# Simulation Testing Setup Script
# Prepares and launches the dual-arm system in Gazebo

set -e

WORKSPACE="/home/mariamelsebaey/full_system"
cd "$WORKSPACE"

echo ""
echo "╔════════════════════════════════════════════════════════════════════════╗"
echo "║         DUAL-ARM CONTROL SYSTEM - SIMULATION SETUP                    ║"
echo "╚════════════════════════════════════════════════════════════════════════╝"
echo ""

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built. Building now..."
    colcon build --packages-select supervisor_package dual_arms_mtc assembly_environment
fi

# Source the setup
echo "📦 Sourcing ROS2 environment..."
source install/setup.bash

# Check for Gazebo
if ! command -v gazebo &> /dev/null; then
    echo "⚠️  Gazebo not installed. Install with:"
    echo "    sudo apt install gazebo ros-jazzy-gazebo-*"
    exit 1
fi

echo ""
echo "✅ Prerequisites verified"
echo ""
echo "Launch sequence (use separate terminals):"
echo ""
echo "TERMINAL 1: Launch Gazebo"
echo "────────────────────────────────────────"
echo "$ cd $WORKSPACE"
echo "$ source install/setup.bash"
echo "$ ros2 launch assembly_environment gazebo.launch.py"
echo ""
echo "TERMINAL 2: Launch Supervisor"
echo "────────────────────────────────────────"
echo "$ cd $WORKSPACE"
echo "$ source install/setup.bash"
echo "$ ros2 run supervisor_package supervisor_node"
echo ""
echo "TERMINAL 3: Monitor Topics"
echo "────────────────────────────────────────"
echo "$ cd $WORKSPACE"
echo "$ source install/setup.bash"
echo "$ ros2 topic echo /supervisor_state"
echo ""
echo "TERMINAL 4: Test Operation Detection"
echo "────────────────────────────────────────"
echo "See test_simulation.sh for example commands"
echo ""
echo "Press ENTER to continue with automated setup..."
read
