#!/bin/bash
# Quick test runner with mock services

set -e

WORKSPACE_DIR="/home/mariamelsebaey/full_system"

echo ""
echo "======================================================================="
echo "SETTING UP SUPERVISOR TESTING WITH MOCK SERVICES"
echo "======================================================================="
echo ""

# Change to workspace
cd "$WORKSPACE_DIR"

# Source the workspace setup
echo "Sourcing workspace setup..."
source install/setup.bash || {
    echo "⚠️  Build files not found. Building workspace..."
    colcon build --packages-select supervisor_package
    source install/setup.bash
}

echo ""
echo "======================================================================="
echo "Mock Services Available:"
echo "======================================================================="
echo ""
echo "✓ Mock Detection Node  - /detect_bricks service"
echo "✓ Mock GUI Node        - /get_assembly_plan service"
echo "✓ Mock Grasping Node   - /grasp/get_grasp_point service"
echo ""
echo "======================================================================="
echo "LAUNCHING MOCK SERVICES"
echo "======================================================================="
echo ""

# Launch all mock services
ros2 launch supervisor_package mock_services.launch.py

