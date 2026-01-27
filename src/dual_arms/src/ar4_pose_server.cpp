#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp" // DIRECT DRIVER CONTROL

#include "supervisor_package/action/move_to_pose.hpp"

using MoveToPose = supervisor_package::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
using moveit::planning_interface::MoveGroupInterface;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class AR4ActionServer : public rclcpp::Node {
public:
    AR4ActionServer() : Node("AR4_pose_commander"), tf_buffer_(this->get_clock()) {
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    }

    void init() {
        // 1. Setup MoveIt
        move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "ar_manipulator");
        move_group_->setPlanningPipelineId("move_group");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setMaxVelocityScalingFactor(0.8); 
        move_group_->setMaxAccelerationScalingFactor(0.4);

        // 2. Setup Direct Connection to Driver (Bypassing MoveIt for execution)
        // Ensure this topic matches your robot's controller! 
        // For AR4 it is usually: /ar4_trajectory_controller/follow_joint_trajectory
        this->driver_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, 
            "/ar4_trajectory_controller/follow_joint_trajectory"
        );

        this->action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            "ar4_point_control",
            std::bind(&AR4ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AR4ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&AR4ActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "AR4 Parallel Action Server Ready.");
    }

private:
    std::shared_ptr<MoveGroupInterface> move_group_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr driver_client_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal) {
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        if (move_group_) move_group_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        // Run in a detached thread to not block ROS spin
        std::thread{std::bind(&AR4ActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveToPose::Result>();

        // --- 1. SETUP TARGET ---
        if (goal->strategy == "HOME") {
            move_group_->setNamedTarget("home");
        } 
        else {
            geometry_msgs::msg::PoseStamped target_msg;
            target_msg.header.stamp = this->now();
            target_msg.header.frame_id = "base_link"; // Trust the Supervisor's Frame
            target_msg.pose = goal->target_pose;

            move_group_->setWorkspace(-1.5, -1.5, -0.5, 1.5, 1.5, 2.0);
            move_group_->clearPoseTargets();
            move_group_->setPoseTarget(target_msg);
        }

        // --- 2. PLAN (Uses MoveIt, blocks briefly) ---
        move_group_->setStartStateToCurrentState();
        MoveGroupInterface::Plan plan;
        bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (plan_success) {
            RCLCPP_INFO(this->get_logger(), "Planning Success. Handing off to Driver for execution...");

            // --- 3. EXECUTE (Directly to Driver, Unblocking MoveIt) ---
            
            if (!driver_client_->wait_for_action_server(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "Driver Action Server unavailable!");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            auto traj_goal = FollowJointTrajectory::Goal();
            traj_goal.trajectory = plan.trajectory.joint_trajectory;

            auto goal_future = driver_client_->async_send_goal(traj_goal);
            if (goal_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
                 result->success = false; goal_handle->abort(result); return;
            }

            auto driver_goal_handle = goal_future.get();
            if (!driver_goal_handle) {
                 result->success = false; goal_handle->abort(result); return;
            }

            // Wait for the ROBOT to finish, but MoveIt is already free to plan for the other arm!
            auto result_future = driver_client_->async_get_result(driver_goal_handle);
            if (result_future.wait_for(std::chrono::seconds(100)) == std::future_status::ready) {
                auto driver_result = result_future.get();
                if (driver_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                     result->success = true;
                     goal_handle->succeed(result);
                } else {
                     RCLCPP_ERROR(this->get_logger(), "Driver execution failed");
                     result->success = false; goal_handle->abort(result);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Driver execution timed out");
                result->success = false; goal_handle->abort(result);
            }

        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning Failed.");
            result->success = false;
            goal_handle->abort(result);
        }

        move_group_->clearPoseTargets();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AR4ActionServer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}