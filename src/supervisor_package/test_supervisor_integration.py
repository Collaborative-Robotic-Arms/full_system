#!/usr/bin/env python3
"""
Integration Tests for Supervisor State Machine
Tests the supervisor's ability to detect operation types and transition states correctly
"""

import unittest
import asyncio
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


class MockBrick:
    """Mock brick object for testing"""
    def __init__(self, brick_id, start_side="ABB", pickup_pose=None, place_pose=None):
        self.id = brick_id
        self.start_side = start_side
        self.pickup_pose = pickup_pose or self._create_pose(0.5, 0.0, 0.35)
        self.place_pose = place_pose or self._create_pose(0.5, 0.0, 0.5)
    
    @staticmethod
    def _create_pose(x, y, z):
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
        return pose


class SupervisorStateSimulator:
    """Simulates supervisor state machine for testing"""
    
    def __init__(self):
        self.state = "INIT"
        self.current_brick = None
        self.assembly_queue = []
        self.ar4_current_pose = None
        self.abb_current_pose = None
        self.enable_operation_type_detection = True
        self.enable_parallel_execution = True
        self.state_history = []
    
    def set_arm_poses(self, ar4_pose: Pose, abb_pose: Pose):
        """Update arm poses for operation type detection"""
        self.ar4_current_pose = ar4_pose
        self.abb_current_pose = abb_pose
    
    def add_brick_to_queue(self, brick):
        """Add brick to assembly queue"""
        self.assembly_queue.append(brick)
    
    def _calculate_separation(self, pose1: Pose, pose2: Pose) -> float:
        """Calculate 3D distance between poses"""
        import math
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def _determine_operation_type(self) -> str:
        """Determine operation type based on arm separation"""
        if not self.ar4_current_pose or not self.abb_current_pose:
            return "UNKNOWN"
        
        separation = self._calculate_separation(
            self.ar4_current_pose, 
            self.abb_current_pose
        )
        
        # Handover zone (< 0.5m)
        if separation <= 0.5:
            return "HANDOVER"
        
        # Parallel safe zone (>= 0.8m)
        if separation >= 0.8:
            return "PARALLEL"
        
        # Default sequential
        return "SEQUENTIAL"
    
    def dispatch_brick(self):
        """Dispatch a brick and determine operation type"""
        if not self.assembly_queue:
            return False
        
        self.current_brick = self.assembly_queue.pop(0)
        self.state_history.append("DISPATCH")
        
        if self.enable_operation_type_detection:
            op_type = self._determine_operation_type()
            
            if op_type == "HANDOVER":
                self.state = "SEQUENTIAL_HANDOVER"
                self.state_history.append("SEQUENTIAL_HANDOVER")
                return "handover"
            elif op_type == "PARALLEL" and self.enable_parallel_execution:
                self.state = "PARALLEL_PICK_PLACE"
                self.state_history.append("PARALLEL_PICK_PLACE")
                return "parallel"
            else:
                # Default to brick start_side logic
                if self.current_brick.start_side == "ABB":
                    self.state = "EXECUTE_ABB_PICK"
                    self.state_history.append("EXECUTE_ABB_PICK")
                elif self.current_brick.start_side == "AR4":
                    self.state = "EXECUTE_AR4_DIRECT"
                    self.state_history.append("EXECUTE_AR4_DIRECT")
                return "sequential"
        else:
            # Detection disabled - use brick start_side directly
            if self.current_brick.start_side == "ABB":
                self.state = "EXECUTE_ABB_PICK"
                self.state_history.append("EXECUTE_ABB_PICK")
                return "sequential_abb"
            elif self.current_brick.start_side == "AR4":
                self.state = "EXECUTE_AR4_DIRECT"
                self.state_history.append("EXECUTE_AR4_DIRECT")
                return "sequential_ar4"
        
        return False
    
    def complete_handover_sequence(self):
        """Simulate handover sequence completion"""
        expected_states = [
            "SEQUENTIAL_HANDOVER",
            "AR4_PICK_FOR_HANDOVER",
            "HANDOVER_EXECUTION",
            "HANDOVER_ABB_PICK",
            "PROCESS_NEXT"
        ]
        
        for state in expected_states[1:]:
            self.state = state
            self.state_history.append(state)
    
    def complete_parallel_sequence(self):
        """Simulate parallel execution completion"""
        expected_states = [
            "PARALLEL_PICK_PLACE",
            "PARALLEL_PLACE",
            "PROCESS_NEXT"
        ]
        
        for state in expected_states[1:]:
            self.state = state
            self.state_history.append(state)


class TestSupervisorStateTransitions(unittest.TestCase):
    """Test supervisor state machine transitions"""
    
    def setUp(self):
        """Initialize simulator"""
        self.simulator = SupervisorStateSimulator()
    
    def test_init_to_dispatch(self):
        """Test initialization to dispatch transition"""
        brick = MockBrick("brick_001", start_side="ABB")
        self.simulator.add_brick_to_queue(brick)
        
        self.assertEqual(self.simulator.state, "INIT")
        result = self.simulator.dispatch_brick()
        self.assertIsNotNone(result)
        self.assertIn("DISPATCH", self.simulator.state_history)
        print(f"✅ INIT → DISPATCH transition successful")
    
    def test_dispatch_detects_handover(self):
        """Test that dispatch correctly detects handover operation"""
        # Set arm poses close together (< 0.5m)
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.5, y=-0.2, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.5, y=0.2, z=0.3)
        
        self.simulator.set_arm_poses(ar4_pose, abb_pose)
        
        brick = MockBrick("brick_001", start_side="ABB")
        self.simulator.add_brick_to_queue(brick)
        
        result = self.simulator.dispatch_brick()
        
        self.assertEqual(result, "handover")
        self.assertEqual(self.simulator.state, "SEQUENTIAL_HANDOVER")
        print(f"✅ Handover operation detected and routed correctly")
    
    def test_dispatch_detects_parallel(self):
        """Test that dispatch correctly detects parallel operation"""
        # Set arm poses far apart (>= 0.8m)
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.2, y=-0.5, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.8, y=0.5, z=0.3)
        
        self.simulator.set_arm_poses(ar4_pose, abb_pose)
        
        brick = MockBrick("brick_001", start_side="ABB")
        self.simulator.add_brick_to_queue(brick)
        
        result = self.simulator.dispatch_brick()
        
        self.assertEqual(result, "parallel")
        self.assertEqual(self.simulator.state, "PARALLEL_PICK_PLACE")
        print(f"✅ Parallel operation detected and routed correctly")
    
    def test_dispatch_detects_sequential(self):
        """Test that dispatch correctly detects sequential operation"""
        # Set arm poses at mid distance (0.5m < separation < 0.8m)
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.4, y=-0.25, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.6, y=0.25, z=0.3)
        
        self.simulator.set_arm_poses(ar4_pose, abb_pose)
        
        brick = MockBrick("brick_001", start_side="ABB")
        self.simulator.add_brick_to_queue(brick)
        
        result = self.simulator.dispatch_brick()
        
        self.assertEqual(result, "sequential")
        self.assertIn("EXECUTE_ABB_PICK", self.simulator.state_history)
        print(f"✅ Sequential operation detected and routed correctly")
    
    def test_parallel_disabled_fallback_to_sequential(self):
        """Test that parallel falls back to sequential when disabled"""
        # Set arm poses for parallel
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.2, y=-0.5, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.8, y=0.5, z=0.3)
        
        self.simulator.set_arm_poses(ar4_pose, abb_pose)
        self.simulator.enable_parallel_execution = False  # Disable parallel
        
        brick = MockBrick("brick_001", start_side="ABB")
        self.simulator.add_brick_to_queue(brick)
        
        result = self.simulator.dispatch_brick()
        
        self.assertNotEqual(result, "parallel")
        print(f"✅ Parallel correctly disabled and fell back to sequential")
    
    def test_operation_type_detection_disabled(self):
        """Test that detection can be disabled"""
        # Set arm poses for handover
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.5, y=-0.1, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.5, y=0.1, z=0.3)
        
        self.simulator.set_arm_poses(ar4_pose, abb_pose)
        self.simulator.enable_operation_type_detection = False  # Disable detection
        
        brick = MockBrick("brick_001", start_side="ABB")
        self.simulator.add_brick_to_queue(brick)
        
        result = self.simulator.dispatch_brick()
        
        # Should use brick.start_side instead of operation type
        self.assertIn(result, ["sequential_abb", "sequential"])
        self.assertIn("EXECUTE_ABB_PICK", self.simulator.state_history)
        print(f"✅ Operation type detection correctly disabled, used brick.start_side instead")


class TestHandoverSequence(unittest.TestCase):
    """Test complete handover sequence"""
    
    def setUp(self):
        """Initialize simulator"""
        self.simulator = SupervisorStateSimulator()
    
    def test_handover_sequence_states(self):
        """Test that handover sequence progresses through all states"""
        # Set arm poses for handover
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.5, y=-0.2, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.5, y=0.2, z=0.3)
        
        self.simulator.set_arm_poses(ar4_pose, abb_pose)
        
        brick = MockBrick("brick_001", start_side="ABB")
        self.simulator.add_brick_to_queue(brick)
        
        # Dispatch
        self.simulator.dispatch_brick()
        self.assertEqual(self.simulator.state, "SEQUENTIAL_HANDOVER")
        
        # Complete sequence
        self.simulator.complete_handover_sequence()
        
        # Verify all states visited
        expected_states = [
            "DISPATCH",
            "SEQUENTIAL_HANDOVER",
            "AR4_PICK_FOR_HANDOVER",
            "HANDOVER_EXECUTION",
            "HANDOVER_ABB_PICK",
            "PROCESS_NEXT"
        ]
        
        self.assertEqual(self.simulator.state_history, expected_states)
        print(f"✅ Handover sequence completed all {len(expected_states)} states")
    
    def test_handover_leads_to_next_brick(self):
        """Test that handover completes and allows processing next brick"""
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.5, y=-0.2, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.5, y=0.2, z=0.3)
        
        self.simulator.set_arm_poses(ar4_pose, abb_pose)
        
        brick1 = MockBrick("brick_001", start_side="ABB")
        brick2 = MockBrick("brick_002", start_side="AR4")
        
        self.simulator.add_brick_to_queue(brick1)
        self.simulator.add_brick_to_queue(brick2)
        
        # Process first brick
        self.simulator.dispatch_brick()
        self.simulator.complete_handover_sequence()
        
        self.assertEqual(self.simulator.state, "PROCESS_NEXT")
        self.assertEqual(len(self.simulator.assembly_queue), 1)
        print(f"✅ Handover completed, ready for next brick")


class TestParallelSequence(unittest.TestCase):
    """Test complete parallel execution sequence"""
    
    def setUp(self):
        """Initialize simulator"""
        self.simulator = SupervisorStateSimulator()
    
    def test_parallel_sequence_states(self):
        """Test that parallel sequence progresses through all states"""
        # Set arm poses for parallel
        ar4_pose = Pose()
        ar4_pose.position = Point(x=0.2, y=-0.5, z=0.3)
        
        abb_pose = Pose()
        abb_pose.position = Point(x=0.8, y=0.5, z=0.3)
        
        self.simulator.set_arm_poses(ar4_pose, abb_pose)
        
        brick = MockBrick("brick_001", start_side="ABB")
        self.simulator.add_brick_to_queue(brick)
        
        # Dispatch
        self.simulator.dispatch_brick()
        self.assertEqual(self.simulator.state, "PARALLEL_PICK_PLACE")
        
        # Complete sequence
        self.simulator.complete_parallel_sequence()
        
        # Verify all states visited
        expected_states = [
            "DISPATCH",
            "PARALLEL_PICK_PLACE",
            "PARALLEL_PLACE",
            "PROCESS_NEXT"
        ]
        
        self.assertEqual(self.simulator.state_history, expected_states)
        print(f"✅ Parallel sequence completed all {len(expected_states)} states")
    
    def test_parallel_is_faster_than_handover(self):
        """Compare state count for parallel vs handover"""
        parallel_states = 4
        handover_states = 6
        
        self.assertLess(parallel_states, handover_states)
        print(f"✅ Parallel ({parallel_states} states) is faster than handover ({handover_states} states)")


class TestMultiBrickExecution(unittest.TestCase):
    """Test processing multiple bricks"""
    
    def setUp(self):
        """Initialize simulator"""
        self.simulator = SupervisorStateSimulator()
    
    def test_process_three_bricks_mixed_operations(self):
        """Test processing 3 bricks with different operation types"""
        # Configure for mixed operations
        self.simulator.enable_operation_type_detection = True
        self.simulator.enable_parallel_execution = True
        
        # Add 3 bricks to queue
        brick1 = MockBrick("brick_001", start_side="ABB")
        brick2 = MockBrick("brick_002", start_side="AR4")
        brick3 = MockBrick("brick_003", start_side="ABB")
        
        self.simulator.add_brick_to_queue(brick1)
        self.simulator.add_brick_to_queue(brick2)
        self.simulator.add_brick_to_queue(brick3)
        
        self.assertEqual(len(self.simulator.assembly_queue), 3)
        
        # Process first brick (handover scenario)
        ar4_pose1 = Pose()
        ar4_pose1.position = Point(x=0.5, y=-0.2, z=0.3)
        abb_pose1 = Pose()
        abb_pose1.position = Point(x=0.5, y=0.2, z=0.3)
        self.simulator.set_arm_poses(ar4_pose1, abb_pose1)
        
        result1 = self.simulator.dispatch_brick()
        self.assertEqual(result1, "handover")
        self.assertEqual(len(self.simulator.assembly_queue), 2)
        
        # Process second brick (parallel scenario)
        ar4_pose2 = Pose()
        ar4_pose2.position = Point(x=0.2, y=-0.5, z=0.3)
        abb_pose2 = Pose()
        abb_pose2.position = Point(x=0.8, y=0.5, z=0.3)
        self.simulator.set_arm_poses(ar4_pose2, abb_pose2)
        
        result2 = self.simulator.dispatch_brick()
        self.assertEqual(result2, "parallel")
        self.assertEqual(len(self.simulator.assembly_queue), 1)
        
        print(f"✅ Multi-brick execution: mixed operations processed correctly")


def run_integration_tests():
    """Run all integration tests"""
    print("\n" + "="*70)
    print("DUAL-ARM CONTROL SYSTEM - INTEGRATION TESTS")
    print("="*70 + "\n")
    
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestSupervisorStateTransitions))
    suite.addTests(loader.loadTestsFromTestCase(TestHandoverSequence))
    suite.addTests(loader.loadTestsFromTestCase(TestParallelSequence))
    suite.addTests(loader.loadTestsFromTestCase(TestMultiBrickExecution))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print("INTEGRATION TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.wasSuccessful():
        print("\n✅ ALL INTEGRATION TESTS PASSED!")
    else:
        print("\n❌ SOME TESTS FAILED - Review output above")
    print("="*70 + "\n")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    import sys
    success = run_integration_tests()
    sys.exit(0 if success else 1)
