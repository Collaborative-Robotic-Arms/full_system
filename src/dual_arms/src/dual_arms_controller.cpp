#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <mutex>
#include <atomic>

using std::placeholders::_1;
namespace moveit_robot = moveit::core;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_arms_controller");

std::mutex mutex;
geometry_msgs::msg::Pose pose_ar4, pose_irb;
std::atomic<bool> ar4_pose_ready(false);
std::atomic<bool> irb_pose_ready(false);

void reset_flag_after_delay(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub, rclcpp::Node::SharedPtr node) {
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std_msgs::msg::Bool msg;
  msg.data = false;
  pub->publish(msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dual_arms_controller");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface group(node, "dual_arms");

  group.setMaxVelocityScalingFactor(0.7);
  group.setMaxAccelerationScalingFactor(0.7);
  group.setPlanningTime(10.0);

  auto ar4_flag_pub = node->create_publisher<std_msgs::msg::Bool>("/ar4/reached_goal", 10);
  auto irb_flag_pub = node->create_publisher<std_msgs::msg::Bool>("/abb/reached_goal", 10);

  auto sub_ar4 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_ar4", 10,
      [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        pose_ar4 = msg->pose;
        ar4_pose_ready = true;
        RCLCPP_INFO(LOGGER, "Received AR4 pose.");
      });

  auto sub_irb = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_irb120", 10,
      [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        pose_irb = msg->pose;
        irb_pose_ready = true;
        RCLCPP_INFO(LOGGER, "Received IRB120 pose.");
      });

  while (rclcpp::ok()) {
    while (rclcpp::ok() && (!ar4_pose_ready && !irb_pose_ready)) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!rclcpp::ok()) break;

    moveit_robot::RobotState joint_goal(*group.getCurrentState());
    const moveit_robot::JointModelGroup* ar4_jmg = joint_goal.getJointModelGroup("ar_manipulator");
    const moveit_robot::JointModelGroup* irb_jmg = joint_goal.getJointModelGroup("irb120_arm");

    bool ar4_ok = !ar4_pose_ready;
    bool irb_ok = !irb_pose_ready;

    if (ar4_pose_ready) {
      ar4_ok = joint_goal.setFromIK(ar4_jmg, pose_ar4, "ar4_ee_link", 0.5);
    }
    if (irb_pose_ready) {
      irb_ok = joint_goal.setFromIK(irb_jmg, pose_irb, "tool0", 0.5);
    }

    if (!ar4_ok || !irb_ok) {
      RCLCPP_ERROR(LOGGER, "IK failed for one or both arms. Skipping...");
      ar4_pose_ready = false;
      irb_pose_ready = false;
      continue;
    }

    group.setStartStateToCurrentState();
    group.setJointValueTarget(joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan dual_plan;
    if (group.plan(dual_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Planning succeeded. Executing...");
      if (group.execute(dual_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Execution completed successfully!");
        if (ar4_pose_ready) {
          std_msgs::msg::Bool msg;
          msg.data = true;
          ar4_flag_pub->publish(msg);
          std::thread(reset_flag_after_delay, ar4_flag_pub, node).detach();
        }
        if (irb_pose_ready) {
          std_msgs::msg::Bool msg;
          msg.data = true;
          irb_flag_pub->publish(msg);
          std::thread(reset_flag_after_delay, irb_flag_pub, node).detach();
        }
      } else {
        RCLCPP_ERROR(LOGGER, "Execution failed.");
      }
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed.");
    }

    ar4_pose_ready = false;
    irb_pose_ready = false;
  }

  rclcpp::shutdown();
  return 0;
}
