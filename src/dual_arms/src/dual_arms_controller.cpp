#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <thread>

namespace moveit_robot = moveit::core;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_arms_controller");

void reset_flag_after_delay(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub) {
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std_msgs::msg::Bool msg;
  msg.data = false;
  pub->publish(msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("dual_arms_controller", node_options);

  auto cb_group_ar4 = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cb_group_irb = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto group_ar4 = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ar_manipulator");
  auto group_irb = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "irb120_arm");

  group_ar4->setMaxVelocityScalingFactor(0.7);
  group_ar4->setMaxAccelerationScalingFactor(0.7);
  group_ar4->setPlanningTime(10.0);

  group_irb->setMaxVelocityScalingFactor(0.7);
  group_irb->setMaxAccelerationScalingFactor(0.7);
  group_irb->setPlanningTime(10.0);

  auto ar4_flag_pub = node->create_publisher<std_msgs::msg::Bool>("/ar4/reached_goal", 10);
  auto irb_flag_pub = node->create_publisher<std_msgs::msg::Bool>("/abb/reached_goal", 10);

  // NEW: Direct publishers to bypass MoveGroup execution lock
  auto ar4_traj_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/ar4_trajectory_controller/joint_trajectory", 10);
  auto irb_traj_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/irb120_trajectory_controller/joint_trajectory", 10);

  rclcpp::SubscriptionOptions sub_opt_ar4;
  sub_opt_ar4.callback_group = cb_group_ar4;
  
  auto sub_ar4 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_ar4", 10,
      [group_ar4, ar4_flag_pub, ar4_traj_pub, node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          RCLCPP_INFO(LOGGER, "AR4 thread: Received pose, calculating IK...");
          
          moveit_robot::RobotStatePtr current_state = group_ar4->getCurrentState();
          const moveit_robot::JointModelGroup* jmg = current_state->getJointModelGroup("ar_manipulator");
          
          if (current_state->setFromIK(jmg, msg->pose, "ar4_ee_link", 0.5)) {
              group_ar4->setJointValueTarget(*current_state);
              moveit::planning_interface::MoveGroupInterface::Plan plan;
              
              if (group_ar4->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                  RCLCPP_INFO(LOGGER, "AR4 thread: Bypassing MoveGroup -> Executing Directly!");
                  
                  // Extract the raw trajectory, stamp it, and fire it!
                  auto traj = plan.trajectory.joint_trajectory;
                  traj.header.stamp.sec = 0;
                  traj.header.stamp.nanosec = 0;
                  ar4_traj_pub->publish(traj);

                  std_msgs::msg::Bool flag_msg;
                  flag_msg.data = true;
                  ar4_flag_pub->publish(flag_msg);
                  std::thread(reset_flag_after_delay, ar4_flag_pub).detach();
              } else {
                  RCLCPP_ERROR(LOGGER, "AR4 thread: Planning failed.");
              }
          } else {
              RCLCPP_ERROR(LOGGER, "AR4 thread: IK failed. Skipping...");
          }
      }, sub_opt_ar4);

  rclcpp::SubscriptionOptions sub_opt_irb;
  sub_opt_irb.callback_group = cb_group_irb;

  auto sub_irb = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_irb120", 10,
      [group_irb, irb_flag_pub, irb_traj_pub, node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          RCLCPP_INFO(LOGGER, "ABB thread: Received pose, calculating IK...");
          
          moveit_robot::RobotStatePtr current_state = group_irb->getCurrentState();
          const moveit_robot::JointModelGroup* jmg = current_state->getJointModelGroup("irb120_arm");
          
          if (current_state->setFromIK(jmg, msg->pose, "tool0", 0.5)) {
              group_irb->setJointValueTarget(*current_state);
              moveit::planning_interface::MoveGroupInterface::Plan plan;
              
              if (group_irb->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                  RCLCPP_INFO(LOGGER, "ABB thread: Bypassing MoveGroup -> Executing Directly!");
                  
                  // Extract the raw trajectory, stamp it, and fire it!
                  auto traj = plan.trajectory.joint_trajectory;
                  traj.header.stamp.sec = 0;
                  traj.header.stamp.nanosec = 0;
                  irb_traj_pub->publish(traj);

                  std_msgs::msg::Bool flag_msg;
                  flag_msg.data = true;
                  irb_flag_pub->publish(flag_msg);
                  std::thread(reset_flag_after_delay, irb_flag_pub).detach();
              } else {
                  RCLCPP_ERROR(LOGGER, "ABB thread: Planning failed.");
              }
          } else {
              RCLCPP_ERROR(LOGGER, "ABB thread: IK failed. Skipping...");
          }
      }, sub_opt_irb);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  RCLCPP_INFO(LOGGER, "Dual arm controller ready and waiting for poses...");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}