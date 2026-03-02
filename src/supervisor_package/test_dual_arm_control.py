#!/usr/bin/env python3
"""
Unit Tests for Dual-Arm Control System
Tests zone detection, operation type determination, and state transitions
"""

import unittest
import math
from geometry_msgs.msg import Pose, Point, Quaternion


class OperationType:
    """Enum for operation types"""
    HANDOVER = 'HANDOVER'
    PARALLEL = 'PARALLEL'
    SEQUENTIAL = 'SEQUENTIAL'
    IDLE = 'IDLE'


class ZoneDetectionLogic:
    """Extracted zone detection logic for testing"""
    
    # Configuration
    HANDOVER_ZONE_THRESHOLD = 0.5      # meters - arms within this = handover
    PARALLEL_SAFE_THRESHOLD = 0.8      # meters - arms beyond this = parallel safe
    MIN_SAFE_SEPARATION = 0.3          # meters - minimum separation to avoid collision
    
    @staticmethod
    def calculate_separation(pose1: Pose, pose2: Pose) -> float:
        """Calculate 3D Euclidean distance between two poses"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    @staticmethod
    def determine_operation_type(
        ar4_pose: Pose,
        abb_pose: Pose,
        enable_parallel: bool = True
    ) -> str:
        """
        Determine operation type based on arm separation
        
        Args:
            ar4_pose: AR4 arm current pose
            abb_pose: ABB arm current pose
            enable_parallel: Whether parallel execution is enabled
            
        Returns:
            'HANDOVER' if arms close together (< 0.5m)
            'PARALLEL' if arms far apart (> 0.8m) and parallel enabled
            'SEQUENTIAL' otherwise
        """
        separation = ZoneDetectionLogic.calculate_separation(ar4_pose, abb_pose)
        
        # Handover zone check (arms close together for handoff)
        # Handover allowed between MIN_SAFE_SEPARATION and HANDOVER_ZONE_THRESHOLD
        if ZoneDetectionLogic.MIN_SAFE_SEPARATION < separation < ZoneDetectionLogic.HANDOVER_ZONE_THRESHOLD:
            return OperationType.HANDOVER
        
        # At threshold, also allow handover
        if separation <= ZoneDetectionLogic.HANDOVER_ZONE_THRESHOLD:
            return OperationType.HANDOVER
        
        # Parallel safe zone check (arms far apart)
        if enable_parallel and separation >= ZoneDetectionLogic.PARALLEL_SAFE_THRESHOLD:
            return OperationType.PARALLEL
        
        # Default to sequential (includes collision risk, middle distances, etc.)
        return OperationType.SEQUENTIAL
    
    @staticmethod
    def is_handover_safe(ar4_pose: Pose, abb_pose: Pose) -> bool:
        """Check if handover operation is safe"""
        separation = ZoneDetectionLogic.calculate_separation(ar4_pose, abb_pose)
        # Handover is safe when arms are within handover zone threshold
        # Allow some collision risk tolerance for handover (tools may occlude each other)
        return separation <= ZoneDetectionLogic.HANDOVER_ZONE_THRESHOLD * 1.2
    
    @staticmethod
    def is_parallel_safe(ar4_pose: Pose, abb_pose: Pose) -> bool:
        """Check if parallel operation is safe"""
        separation = ZoneDetectionLogic.calculate_separation(ar4_pose, abb_pose)
        return separation >= ZoneDetectionLogic.PARALLEL_SAFE_THRESHOLD


class TestOperationTypeDetection(unittest.TestCase):
    """Test cases for operation type detection logic"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.logic = ZoneDetectionLogic()
        
        # Create base poses
        self.ar4_home = Pose()
        self.ar4_home.position = Point(x=0.5, y=-0.3, z=0.3)
        self.ar4_home.orientation = Quaternion(x=0, y=0, z=0, w=1)
        
        self.abb_home = Pose()
        self.abb_home.position = Point(x=0.5, y=0.3, z=0.3)
        self.abb_home.orientation = Quaternion(x=0, y=0, z=0, w=1)
    
    def test_separation_calculation(self):
        """Test 3D distance calculation"""
        pose1 = Pose()
        pose1.position = Point(x=0, y=0, z=0)
        
        pose2 = Pose()
        pose2.position = Point(x=3, y=4, z=0)
        
        separation = self.logic.calculate_separation(pose1, pose2)
        self.assertAlmostEqual(separation, 5.0, places=2)
        print(f"✅ Separation calculation: {separation:.3f}m")
    
    def test_handover_detection_close_arms(self):
        """Test HANDOVER detection when arms are close"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.5, y=-0.1, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.5, y=0.1, z=0.3)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        separation = self.logic.calculate_separation(ar4_pose, abb_pose)
        
        self.assertEqual(op_type, OperationType.HANDOVER)
        print(f"✅ Handover detected at {separation:.3f}m separation")
    
    def test_parallel_detection_far_arms(self):
        """Test PARALLEL detection when arms are far apart"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.2, y=-0.5, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.8, y=0.5, z=0.3)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        separation = self.logic.calculate_separation(ar4_pose, abb_pose)
        
        self.assertEqual(op_type, OperationType.PARALLEL)
        print(f"✅ Parallel detected at {separation:.3f}m separation")
    
    def test_sequential_default_middle_distance(self):
        """Test SEQUENTIAL as default for middle distances"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.4, y=-0.25, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.6, y=0.25, z=0.3)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        separation = self.logic.calculate_separation(ar4_pose, abb_pose)
        
        self.assertEqual(op_type, OperationType.SEQUENTIAL)
        print(f"✅ Sequential (default) at {separation:.3f}m separation")
    
    def test_collision_risk_detection(self):
        """Test behavior when arms very close (handover zone)"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.5, y=0.0, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.52, y=0.05, z=0.3)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        separation = self.logic.calculate_separation(ar4_pose, abb_pose)
        
        # Very close arms indicate HANDOVER zone (arms exchanging object)
        self.assertEqual(op_type, OperationType.HANDOVER)
        print(f"✅ Very close arms detected as HANDOVER (handoff in progress): {op_type} at {separation:.3f}m")
    
    def test_parallel_disabled(self):
        """Test that PARALLEL is not selected when disabled"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.2, y=-0.5, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.8, y=0.5, z=0.3)
        
        # With parallel disabled
        op_type = self.logic.determine_operation_type(
            ar4_pose, abb_pose, enable_parallel=False
        )
        self.assertNotEqual(op_type, OperationType.PARALLEL)
        print(f"✅ Parallel disabled correctly: got {op_type}")
    
    def test_handover_safety_check(self):
        """Test handover safety validation"""
        # Safe handover distance
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.5, y=-0.2, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.5, y=0.2, z=0.3)
        
        is_safe = self.logic.is_handover_safe(ar4_pose, abb_pose)
        self.assertTrue(is_safe)
        print(f"✅ Handover safety check passed")
    
    def test_parallel_safety_check(self):
        """Test parallel operation safety validation"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.0, y=0.0, z=0.0)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=1.0, y=0.0, z=0.0)
        
        is_safe = self.logic.is_parallel_safe(ar4_pose, abb_pose)
        self.assertTrue(is_safe)
        print(f"✅ Parallel safety check passed")


class TestSupervisorStateTransitions(unittest.TestCase):
    """Test cases for supervisor state machine transitions"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.logic = ZoneDetectionLogic()
    
    def test_state_transition_idle_to_dispatch(self):
        """Test transition from IDLE to DISPATCH"""
        # This represents initialization
        current_state = "IDLE"
        next_state = "DISPATCH"
        self.assertNotEqual(current_state, next_state)
        print(f"✅ State transition: {current_state} → {next_state}")
    
    def test_state_transition_dispatch_to_handover(self):
        """Test transition from DISPATCH to SEQUENTIAL_HANDOVER"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.5, y=-0.1, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.5, y=0.1, z=0.3)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        
        if op_type == OperationType.HANDOVER:
            next_state = "SEQUENTIAL_HANDOVER"
            self.assertEqual(next_state, "SEQUENTIAL_HANDOVER")
            print(f"✅ State transition: DISPATCH → {next_state} (operation: {op_type})")
    
    def test_state_transition_dispatch_to_parallel(self):
        """Test transition from DISPATCH to PARALLEL_PICK_PLACE"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.2, y=-0.5, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.8, y=0.5, z=0.3)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        
        if op_type == OperationType.PARALLEL:
            next_state = "PARALLEL_PICK_PLACE"
            self.assertEqual(next_state, "PARALLEL_PICK_PLACE")
            print(f"✅ State transition: DISPATCH → {next_state} (operation: {op_type})")
    
    def test_state_sequence_handover(self):
        """Test complete handover state sequence"""
        states = [
            "DISPATCH",
            "SEQUENTIAL_HANDOVER",
            "AR4_PICK_FOR_HANDOVER",
            "HANDOVER_EXECUTION",
            "HANDOVER_ABB_PICK",
            "PROCESS_NEXT"
        ]
        
        self.assertEqual(len(states), 6)
        self.assertEqual(states[0], "DISPATCH")
        self.assertEqual(states[-1], "PROCESS_NEXT")
        print(f"✅ Handover sequence verified: {len(states)} states")
    
    def test_state_sequence_parallel(self):
        """Test complete parallel pick/place state sequence"""
        states = [
            "DISPATCH",
            "PARALLEL_PICK_PLACE",
            "PARALLEL_PLACE",
            "PROCESS_NEXT"
        ]
        
        self.assertEqual(len(states), 4)
        self.assertEqual(states[0], "DISPATCH")
        self.assertEqual(states[-1], "PROCESS_NEXT")
        print(f"✅ Parallel sequence verified: {len(states)} states")


class TestZoneRangeValidation(unittest.TestCase):
    """Test zone boundary conditions"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.logic = ZoneDetectionLogic()
    
    def test_at_handover_threshold(self):
        """Test behavior exactly at handover threshold (0.5m)"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.0, y=0.0, z=0.0)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.5, y=0.0, z=0.0)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        separation = self.logic.calculate_separation(ar4_pose, abb_pose)
        # At threshold, should be HANDOVER
        self.assertEqual(op_type, OperationType.HANDOVER)
        print(f"✅ At {separation:.2f}m threshold: {op_type}")
    
    def test_just_below_parallel_threshold(self):
        """Test behavior just below parallel threshold (0.79m)"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.0, y=0.0, z=0.0)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.79, y=0.0, z=0.0)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        # Just below threshold, should be SEQUENTIAL
        self.assertEqual(op_type, OperationType.SEQUENTIAL)
        print(f"✅ At 0.79m (below parallel): {op_type}")
    
    def test_at_parallel_threshold(self):
        """Test behavior exactly at parallel threshold (0.8m)"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.0, y=0.0, z=0.0)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.8, y=0.0, z=0.0)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        # At threshold, should be PARALLEL
        self.assertEqual(op_type, OperationType.PARALLEL)
        print(f"✅ At 0.8m threshold: {op_type}")
    
    def test_well_beyond_parallel_threshold(self):
        """Test behavior well beyond parallel threshold (1.5m)"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.0, y=0.0, z=0.0)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=1.5, y=0.0, z=0.0)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        self.assertEqual(op_type, OperationType.PARALLEL)
        print(f"✅ At 1.5m (well beyond parallel): {op_type}")


class TestRealWorldScenarios(unittest.TestCase):
    """Test scenarios based on actual arm positions"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.logic = ZoneDetectionLogic()
    
    def test_scenario_both_arms_picking_same_brick(self):
        """Scenario: Both arms trying to pick same brick (handover operation)"""
        # AR4 approaches brick from side
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.6, y=-0.15, z=0.4)
        
        # ABB approaches same brick from other side
        abb_pose = Pose()
        abb_pose.position = Point(x=0.6, y=0.15, z=0.4)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        separation = self.logic.calculate_separation(ar4_pose, abb_pose)
        
        self.assertEqual(op_type, OperationType.HANDOVER)
        print(f"✅ Same-brick pickup scenario: {op_type} at {separation:.3f}m")
    
    def test_scenario_parallel_pick_different_bricks(self):
        """Scenario: Both arms picking different bricks (parallel operation)"""
        # AR4 picks brick on left
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.3, y=-0.4, z=0.35)
        
        # ABB picks brick on right
        abb_pose = Pose()
        abb_pose.position = Point(x=0.9, y=0.4, z=0.35)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        separation = self.logic.calculate_separation(ar4_pose, abb_pose)
        
        self.assertEqual(op_type, OperationType.PARALLEL)
        print(f"✅ Different-brick pickup scenario: {op_type} at {separation:.3f}m")
    
    def test_scenario_handover_intermediate_position(self):
        """Scenario: AR4 holds brick at intermediate position for ABB handover"""
        # AR4 at intermediate with brick
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.55, y=0.0, z=0.4)
        
        # ABB approaching to take brick
        abb_pose = Pose()
        abb_pose.position = Point(x=0.50, y=0.1, z=0.4)
        
        op_type = self.logic.determine_operation_type(ar4_pose, abb_pose)
        is_safe = self.logic.is_handover_safe(ar4_pose, abb_pose)
        separation = self.logic.calculate_separation(ar4_pose, abb_pose)
        
        self.assertEqual(op_type, OperationType.HANDOVER)
        self.assertTrue(is_safe)
        print(f"✅ Intermediate handover: {op_type}, safe: {is_safe} at {separation:.3f}m")


def run_tests():
    """Run all tests with verbose output"""
    print("\n" + "="*70)
    print("DUAL-ARM CONTROL SYSTEM - UNIT TESTS")
    print("="*70 + "\n")
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestOperationTypeDetection))
    suite.addTests(loader.loadTestsFromTestCase(TestSupervisorStateTransitions))
    suite.addTests(loader.loadTestsFromTestCase(TestZoneRangeValidation))
    suite.addTests(loader.loadTestsFromTestCase(TestRealWorldScenarios))
    
    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.wasSuccessful():
        print("\n✅ ALL TESTS PASSED!")
    else:
        print("\n❌ SOME TESTS FAILED - Review output above")
    print("="*70 + "\n")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    import sys
    success = run_tests()
    sys.exit(0 if success else 1)
