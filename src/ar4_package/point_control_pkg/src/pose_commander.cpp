#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "supervisor_package/action/move_to_pose.hpp"

class AR4PoseActionServer : public rclcpp::Node {
public:
    using MoveToPose = supervisor_package::action::MoveToPose;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveToPose>;

    AR4PoseActionServer() : Node("ar4_pose_action_server", 
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
    {
        // 1. Setup Action Server
        this->action_server_ = rclcpp_action::create_server<MoveToPose>(
            this, "ar4_point_control",
            std::bind(&AR4PoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AR4PoseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&AR4PoseActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "AR4 Action Server is starting up...");
    }

    // Since shared_from_this() cannot be called in the constructor, we init MoveGroup here
    void init_move_group() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ar_manipulator");
        
        // 2. Set default MoveIt parameters from your original file
        move_group_->setMaxVelocityScalingFactor(0.7);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        
        RCLCPP_INFO(this->get_logger(), "MoveIt Interface Initialized for AR4.");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal) {
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
        (void)goal_handle;
        move_group_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
        // Run execution in a separate thread to keep the node responsive
        std::thread{std::bind(&AR4PoseActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveToPose::Result>();
        auto feedback = std::make_shared<MoveToPose::Feedback>();

        geometry_msgs::msg::Pose target_pose = goal->target_pose;

        // --- 3. STRATEGY LOGIC ---
        if (goal->strategy == "home") {
            move_group_->setNamedTarget("home");
            RCLCPP_INFO(this->get_logger(), "Strategy: Moving to HOME");
        } 
        else if (goal->strategy == "APPROACH_OFFSET") {
            // Apply the 15cm Z-offset requested by your state machine
            target_pose.position.z += 0.15; 
            move_group_->setPoseTarget(target_pose);
            RCLCPP_INFO(this->get_logger(), "Strategy: Approach with 15cm Offset");
        }
        else if (goal->strategy == "GRASP" || goal->strategy == "RELEASE") {
            // Direct movement to exact pose (e.g., after visual servoing)
            move_group_->setPoseTarget(target_pose);
            RCLCPP_INFO(this->get_logger(), "Strategy: %s", goal->strategy.c_str());
        }
        else {
            move_group_->setPoseTarget(target_pose);
        }

        // --- 4. PLANNING & EXECUTION ---
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            // In a real system, you could loop here and check distance for feedback
            // For MoveIt, we usually trigger execute()
            auto move_result = move_group_->execute(my_plan);
            
            if (move_result == moveit::core::MoveItErrorCode::SUCCESS) {
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Move Complete!");
            } else {
                result->success = false;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Execution failed!");
            }
        } else {
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AR4PoseActionServer>();
    
    // Initialize move group after the node is shared
    node->init_move_group();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}