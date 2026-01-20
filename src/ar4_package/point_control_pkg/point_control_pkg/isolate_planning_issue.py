#!/usr/bin/env python3
import rclpy
from moveit_commander import MoveGroupCommander, RobotCommander

def main():
    rclpy.init()
    
    # Initialize MoveIt
    group = MoveGroupCommander("ar_manipulator")
    robot = RobotCommander()
    
    # 1️⃣ Print current joint values
    joints = group.get_current_joint_values()
    print("=== Current Joint Values ===")
    for i, val in enumerate(joints):
        print(f"Joint {i+1}: {val:.3f}")
    
    # 2️⃣ Print current EE pose
    current_pose = group.get_current_pose().pose
    print("\n=== Current End-Effector Pose ===")
    print(f"Position: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
    print(f"Orientation: x={current_pose.orientation.x:.3f}, y={current_pose.orientation.y:.3f}, z={current_pose.orientation.z:.3f}, w={current_pose.orientation.w:.3f}")
    
    # 3️⃣ Define goal pose
    goal_pose = current_pose
    goal_pose.position.x = 1.00
    goal_pose.position.y = 0.007
    goal_pose.position.z = 1.40
    goal_pose.orientation.x = 0.0  # optional identity orientation
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0
    
    # Try direct plan
    group.set_pose_target(goal_pose)
    print("\nAttempting direct plan to goal...")
    plan = group.plan()
    if plan[0]:
        print("✅ Direct plan succeeded")
    else:
        print("❌ Direct plan failed")
    
    # 4️⃣ Small-step plan (2 cm offsets)
    small_goal = current_pose
    small_goal.position.x -= 0.02
    small_goal.position.z -= 0.02
    group.set_pose_target(small_goal)
    print("\nAttempting small-step plan (2cm)...")
    plan_small = group.plan()
    if plan_small[0]:
        print("✅ Small-step plan succeeded")
    else:
        print("❌ Small-step plan failed")
    
    # 5️⃣ Plan with loose orientation tolerance
    group.set_pose_target(goal_pose)
    group.set_goal_orientation_tolerance(0.5)  # ~28°
    print("\nAttempting plan with loose orientation tolerance...")
    plan_loose = group.plan()
    if plan_loose[0]:
        print("✅ Plan with loose orientation succeeded")
    else:
        print("❌ Plan with loose orientation failed")
    
    print("\n✅ Isolation checks complete")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
