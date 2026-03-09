#include <memory>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <supervisor_package/action/move_to_pose.hpp>

namespace point_control {
class PoseCommanderAction : public rclcpp::Node {
public:
  using MoveToPose = supervisor_package::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

  PoseCommanderAction() : Node("ar4_pose_commander") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void init() {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ar_manipulator");
    
    this->action_server_ = rclcpp_action::create_server<MoveToPose>(
      this, "ar4_point_control",
      std::bind(&PoseCommanderAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PoseCommanderAction::handle_cancel, this, std::placeholders::_1),
      std::bind(&PoseCommanderAction::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "AR4 Pose Commander Action Server Ready.");
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request with strategy: %s", goal->strategy.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // =========================================================================
  // 🛑 E-STOP CANCEL HANDLER
  // =========================================================================
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
    (void)goal_handle;
    RCLCPP_ERROR(this->get_logger(), "🛑 AR4 EMERGENCY CANCEL RECEIVED! HALTING ARM!");
    
    // 1. Tell MoveIt to stop tracking/planning
    if (move_group_) {
        move_group_->stop(); 
    }

    // 2. PHYSICAL FREEZE: Clear the hardware controller queue
    // We create a temporary publisher to send an empty trajectory
    auto stop_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/ar4_controller/joint_trajectory", 10);
    
    trajectory_msgs::msg::JointTrajectory empty_msg;
    empty_msg.header.stamp = this->get_clock()->now();
    
    // Standard AR4 joint names
    empty_msg.joint_names = {
        "joint_1", "joint_2", "joint_3", 
        "joint_4", "joint_5", "joint_6"
    };
    
    // Sending the message with joint names but NO points forces the controller to stop
    stop_pub->publish(empty_msg);
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
    std::thread{std::bind(&PoseCommanderAction::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // =========================================================================
  // EXECUTION LOGIC WITH CANCELLATION CHECKS
  // =========================================================================
  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();

    move_group_->clearPoseTargets();
    
    if (goal->strategy == "HOME") {
      move_group_->setNamedTarget("home");
    } else {
      geometry_msgs::msg::PoseStamped target_stamped;
      target_stamped.header.frame_id = "base_link";
      target_stamped.header.stamp = this->get_clock()->now();
      target_stamped.pose = goal->target_pose; 

      move_group_->setPoseTarget(target_stamped);
    }

    // 1. Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool plan_success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (plan_success) {
      // Check if we were already canceled while planning
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      // 2. Execute and capture the ACTUAL MoveIt return code
      auto exec_code = move_group_->execute(my_plan);

      // 3. Check if the execution was stopped by handle_cancel()
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "🛑 AR4 execution HALTED mid-trajectory.");
        result->success = false;
        goal_handle->canceled(result);
      } 
      // Check for normal success
      else if (exec_code == moveit::core::MoveItErrorCode::SUCCESS) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Execution successful");
      } 
      // Handle hardware/MoveIt failures
      else {
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Execution failed in MoveIt.");
      }
    } else {
      result->success = false;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
    }
  }
};
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<point_control::PoseCommanderAction>();
  node->init(); 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}