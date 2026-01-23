#include <csignal>
#include <iostream>
#include <limits>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.hpp>

using moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

std::atomic<bool> running(true);

void signalHandler(int) { running = false; }

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signalHandler);

  auto node = std::make_shared<rclcpp::Node>("ar4_pose_commander");
  auto logger = node->get_logger();

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  MoveGroupInterface move_group(node, "ar_manipulator");

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setMaxVelocityScalingFactor(0.7);
  move_group.setMaxAccelerationScalingFactor(0.3);
  move_group.setGoalPositionTolerance(0.001);
  move_group.setGoalOrientationTolerance(0.01);
  move_group.setGoalJointTolerance(0.001);

  const std::string ee_link = move_group.getEndEffectorLink();
  const std::string planning_frame = move_group.getPlanningFrame();

  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());
  RCLCPP_INFO(logger, "End effector link: %s", ee_link.c_str());

  char option = 0;

  while (rclcpp::ok() && running) {
    rclcpp::spin_some(node);

    std::cout << "\nHome (H), Pose (P), Quit (Q): ";
    std::cin >> option;

    if (option == 'Q' || option == 'q')
      break;

    /* ---------------- HOME ---------------- */
    if (option == 'H' || option == 'h') {
      move_group.setNamedTarget("home");
      move_group.setStartStateToCurrentState();

      MoveGroupInterface::Plan plan;
      if (move_group.plan(plan)) {
        move_group.execute(plan);
        move_group.stop();
        move_group.clearPoseTargets();
        RCLCPP_INFO(logger, "Moved to HOME");
      } else {
        RCLCPP_ERROR(logger, "Failed to plan HOME");
      }
    }

    /* ---------------- POSE ---------------- */
    if (option == 'P' || option == 'p') {
      double x, y, z, roll_deg, pitch_deg, yaw_deg;

      std::cout << "\nEnter target pose (x y z roll pitch yaw in degrees): ";
      if (!(std::cin >> x >> y >> z >> roll_deg >> pitch_deg >> yaw_deg)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        RCLCPP_ERROR(logger, "Invalid input");
        continue;
      }

      tf2::Quaternion q;
      q.setRPY(roll_deg * M_PI / 180.0, pitch_deg * M_PI / 180.0,
               yaw_deg * M_PI / 180.0);
      q.normalize();

      geometry_msgs::msg::PoseStamped ee_pose_dummy;
      ee_pose_dummy.header.frame_id = "ABB_base_link";
      ee_pose_dummy.header.stamp = node->now();
      ee_pose_dummy.pose.position.x = x;
      ee_pose_dummy.pose.position.y = y;
      ee_pose_dummy.pose.position.z = z;
      ee_pose_dummy.pose.orientation = tf2::toMsg(q);

      geometry_msgs::msg::PoseStamped ee_pose_base;
      try {
        ee_pose_base = tf_buffer.transform(ee_pose_dummy, planning_frame,
                                           tf2::durationFromSec(1.0));
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(logger, "TF transform failed: %s", ex.what());
        continue;
      }

      move_group.clearPoseTargets();
      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(ee_pose_base);

      MoveGroupInterface::Plan plan;
      if (move_group.plan(plan)) {
        move_group.execute(plan);
        move_group.stop();
        move_group.clearPoseTargets();
        RCLCPP_INFO(logger, "Moved to target pose");
      } else {
        RCLCPP_ERROR(logger, "Pose planning failed");
      }
    }
  }

  rclcpp::shutdown();
  return 0;
}
