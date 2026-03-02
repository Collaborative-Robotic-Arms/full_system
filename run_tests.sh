#!/bin/bash
# Test Execution Script for Dual-Arm Control System
# Runs all unit tests and integration tests

set -e  # Exit on error

WORKSPACE_DIR="/home/mariamelsebaey/full_system"
TEST_DIR="$WORKSPACE_DIR/src/supervisor_package"

echo ""
echo "======================================================================="
echo "DUAL-ARM CONTROL SYSTEM - TEST SUITE"
echo "======================================================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Initialize counters
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Function to run a test file
run_test() {
    local test_file=$1
    local test_name=$2
    
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}Running: $test_name${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    
    if cd "$WORKSPACE_DIR" && python3 "$TEST_DIR/$test_file" 2>&1; then
        echo ""
        echo -e "${GREEN}✅ $test_name PASSED${NC}"
        PASSED_TESTS=$((PASSED_TESTS + 1))
        return 0
    else
        echo ""
        echo -e "${RED}❌ $test_name FAILED${NC}"
        FAILED_TESTS=$((FAILED_TESTS + 1))
        return 1
    fi
}

# Run all tests
echo -e "${BLUE}┌─ UNIT TESTS${NC}"
run_test "test_dual_arm_control.py" "Zone Detection & Operation Type Logic"
TOTAL_TESTS=$((TOTAL_TESTS + 1))

echo ""
echo -e "${BLUE}┌─ INTEGRATION TESTS${NC}"
run_test "test_supervisor_integration.py" "Supervisor State Machine"
TOTAL_TESTS=$((TOTAL_TESTS + 1))

# Print final summary
echo ""
echo "======================================================================="
echo "TEST EXECUTION SUMMARY"
echo "======================================================================="
echo ""
echo "Total Test Suites:  $TOTAL_TESTS"
echo "Passed:             $((PASSED_TESTS)) / $TOTAL_TESTS"
echo "Failed:             $((FAILED_TESTS)) / $TOTAL_TESTS"
echo ""

if [ $FAILED_TESTS -eq 0 ]; then
    echo -e "${GREEN}✅ ALL TESTS PASSED!${NC}"
    echo ""
    echo "Next Steps:"
    echo "  1. Review test output above for any warnings"
    echo "  2. Build C++ packages: colcon build --packages-select dual_arms_mtc"
    echo "  3. Launch supervisor: ros2 run supervisor_package supervisor_node"
    echo "  4. Test with actual hardware or simulation"
    echo ""
    exit 0
else
    echo -e "${RED}❌ SOME TESTS FAILED${NC}"
    echo ""
    echo "Please review the test output above for details."
    exit 1
fi
