/*
# ===================================== COPYRIGHT ===================================== #
#  IFRA Group - Cranfield University
#  Authors: Mikel Bueno Viso, Dr. Seemal Asif, Prof. Phil Webb
#  Date: August, 2023.
# ===================================== COPYRIGHT ===================================== #
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include "ros2srrc_data/action/robmove.hpp"
#include <memory>
#include <thread>

// =============================================================================== //
//  PARAM -> ROBOT:

class ros2_RobotParam : public rclcpp::Node
{
public:
    ros2_RobotParam() : Node("ros2_RobotParam") 
    {
        this->declare_parameter("ROB_PARAM", "none");
        param_ROB = this->get_parameter("ROB_PARAM").as_string();
        RCLCPP_INFO(this->get_logger(), "ROB_PARAM received -> %s", param_ROB.c_str());
    }

    std::string get_robot_param() {
        return param_ROB;
    }

private:
    std::string param_ROB;
};

// =============================================================================== //
// MoveIt!2 -> MoveGroupInterface/Plan function:

moveit::planning_interface::MoveGroupInterface::Plan plan_ROB(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface->plan(my_plan).val == moveit::core::MoveItErrorCode::SUCCESS);

    return my_plan;
}

// =============================================================================== //
// ROS2 Action Server to move the ROBOT:

class ActionServer : public rclcpp::Node
{
public:
    using Robmove = ros2srrc_data::action::Robmove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Robmove>;

    explicit ActionServer(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
        : Node("ros2srrc_RobMove"), move_group_(move_group)
    {
        action_server_ = rclcpp_action::create_server<Robmove>(
            this,
            "/Robmove",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }

private:
    rclcpp_action::Server<Robmove>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Robmove::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received Robot Movement Request (Type: %s, Speed: %.2f)", goal->type.c_str(), goal->speed);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread(&ActionServer::execute, this, goal_handle).detach();
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request.");
        move_group_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto GOAL = goal_handle->get_goal();
        auto RESULT = std::make_shared<Robmove::Result>();

        geometry_msgs::msg::Pose TARGET_POSE;
        TARGET_POSE.position.x = GOAL->x;
        TARGET_POSE.position.y = GOAL->y;
        TARGET_POSE.position.z = GOAL->z;
        TARGET_POSE.orientation.x = GOAL->qx;
        TARGET_POSE.orientation.y = GOAL->qy;
        TARGET_POSE.orientation.z = GOAL->qz;
        TARGET_POSE.orientation.w = GOAL->qw;

        move_group_->setPoseTarget(TARGET_POSE);
        move_group_->setPlannerId(GOAL->type);
        move_group_->setMaxVelocityScalingFactor(GOAL->speed);

        auto MyPlan = plan_ROB(move_group_);

        auto exec_result = move_group_->move();
        bool ExecSUCCESS = (exec_result.val == moveit::core::MoveItErrorCode::SUCCESS);

        if (ExecSUCCESS) {
            RESULT->success = true;
            RESULT->message = "RobMove: SUCCESS";
            goal_handle->succeed(RESULT);
        } else {
            RESULT->success = false;
            RESULT->message = "RobMove: EXECUTION FAILED";
            goal_handle->succeed(RESULT);
        }
    }
};

// ===================================================================================== //
// ======================================= MAIN ======================================== //
// ===================================================================================== //

int main(int argc, char **argv)
{
    // Initialise ROS2
    rclcpp::init(argc, argv);

    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Ensure parameter is fully loaded

    std::string param_ROB = node_PARAM_ROB->get_robot_param();
    if (param_ROB == "none") {
        RCLCPP_ERROR(node_PARAM_ROB->get_logger(), "Invalid ROB_PARAM received! Exiting.");
        rclcpp::shutdown();
        return -1;
    }

    std::string ROBname = param_ROB + "_arm";
    auto moveit_node = std::make_shared<rclcpp::Node>("ros2srrc_RobMove");

    auto move_group_interface_ROB = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node, ROBname);

    auto action_server = std::make_shared<ActionServer>(move_group_interface_ROB);
    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}

