#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "point_control_pkg/action/move_to_pose.hpp"

using MoveToPose = point_control_pkg::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

class PoseCommanderActionServer : public rclcpp::Node
{
public:
  PoseCommanderActionServer()
  : Node("pose_commander_action")
  {
    action_server_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "ar4_move_to_pose",
      std::bind(&PoseCommanderActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PoseCommanderActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PoseCommanderActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "AR4 Pose Commander Action Server Ready");
  }

private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(get_logger(),
      "Received new goal: [%.3f, %.3f, %.3f]",
      goal->target_pose.position.x,
      goal->target_pose.position.y,
      goal->target_pose.position.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_WARN(get_logger(), "Goal cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    std::thread{std::bind(&PoseCommanderActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    auto result = std::make_shared<MoveToPose::Result>();

    RCLCPP_INFO(get_logger(), "Creating MoveGroupInterface...");
    moveit::planning_interface::MoveGroupInterface move_group(
      rclcpp::Node::shared_from_this(), "ar_manipulator");
    move_group.setWorkspace(0.0, -2.0, -5.0, 4.0, 4.0, 0.1);
    move_group.setPlanningTime(5.0);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.01);

    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(get_logger(), "=== Current End-Effector Pose ===");
    RCLCPP_INFO(get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z);
    RCLCPP_INFO(get_logger(), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w);

    // --- Apply tiny offsets to avoid OMPL GOAL_STATE_INVALID ---
    geometry_msgs::msg::Pose safe_goal = goal->target_pose;
    safe_goal.position.x = (safe_goal.position.x == 0.0) ? 1e-6 : safe_goal.position.x;
    safe_goal.position.y = (safe_goal.position.y == 0.0) ? 1e-6 : safe_goal.position.y;
    safe_goal.position.z = (safe_goal.position.z == 0.0) ? 1e-6 : safe_goal.position.z;

    safe_goal.orientation.x = (safe_goal.orientation.x == 0.0) ? 1e-6 : safe_goal.orientation.x;
    safe_goal.orientation.y = (safe_goal.orientation.y == 0.0) ? 1e-6 : safe_goal.orientation.y;
    safe_goal.orientation.z = (safe_goal.orientation.z == 0.0) ? 1e-6 : safe_goal.orientation.z;
    safe_goal.orientation.w = 1.0; // keep w=1 for identity

    RCLCPP_INFO(get_logger(), "Setting pose target...");
    move_group.setPoseTarget(safe_goal);

    RCLCPP_INFO(get_logger(), "Planning motion...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_success = move_group.plan(plan);

    if (plan_success != moveit::core::MoveItErrorCode::SUCCESS)
    {
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "Planning failed");
      return;
    }

    RCLCPP_INFO(get_logger(), "Starting execution...");
    for (int i = 0; i <= 10; i++)
    {
      if (goal_handle->is_canceling())
      {
        result->success = false;
        result->message = "Goal canceled";
        goal_handle->canceled(result);
        RCLCPP_WARN(get_logger(), "Goal canceled during execution");
        return;
      }
      feedback->progress = i * 10.0;
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      result->success = false;
      result->message = "Execution failed";
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "Execution failed");
      return;
    }

    result->success = true;
    result->message = "Motion completed successfully";
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal execution succeeded");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseCommanderActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
