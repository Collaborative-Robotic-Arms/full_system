/*
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: April, 2023.                                                                   #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #
*/

// Include standard libraries:
#include <string>
#include <vector>
#include <thread>
#include <future>

// Include -> YAML file parser:
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// INCLUDE -> FUNCTIONS:
#include "ros2srrc_execution/movej.h"
#include "ros2srrc_execution/movel.h"
#include "ros2srrc_execution/mover.h"
#include "ros2srrc_execution/moverot.h"
#include "ros2srrc_execution/moverp.h"
#include "ros2srrc_execution/moveg.h"

// Include RCLCPP and RCLCPP_ACTION:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// Include the move ROS2 ACTION:
#include "ros2srrc_data/action/move.hpp"

// Include the ROS2 MSG messages:
#include "ros2srrc_data/msg/joint.hpp"
#include "ros2srrc_data/msg/joints.hpp"
#include "ros2srrc_data/msg/xyz.hpp"
#include "ros2srrc_data/msg/xyzypr.hpp"
#include "ros2srrc_data/msg/ypr.hpp"
#include "ros2srrc_data/msg/specs.hpp"

// Declaration of GLOBAL VARIABLES --> ROBOT / END-EFFECTOR / ENVIRONMENT PARAMETERS:
std::string param_ROB = "none";
std::string param_EE = "none";
std::string param_ENV = "none";

// Declaration of GLOBAL VARIABLES --> MoveIt!2 Interface:
moveit::planning_interface::MoveGroupInterface move_group_interface_ROB;
moveit::planning_interface::MoveGroupInterface move_group_interface_EE;

// Declaration of GLOBAL VARIABLES --> JointModelGroup:
const moveit::core::JointModelGroup* joint_model_group_ROB;
const moveit::core::JointModelGroup* joint_model_group_EE;

// Declaration of GLOBAL VARIABLE --> RES:
std::string RES = "none";

// Declaration of GLOBAL VARIABLES --> robotSPECS and eeSPECS:
ros2srrc_data::msg::Specs robotSPECS;
ros2srrc_data::msg::Specs eeSPECS;

// ======================================================================================================================== //
// ==================== PARAM: ROBOT + END-EFFECTOR ==================== //

class ros2_RobotParam : public rclcpp::Node
{
public:
    ros2_RobotParam() : Node("ros2_RobotParam") 
    {
        this->declare_parameter("ROB_PARAM", "none");
        param_ROB = this->get_parameter("ROB_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "ROB_PARAM received -> %s", param_ROB.c_str());
    }
private:
};

class ros2_EEParam : public rclcpp::Node
{
public:
    ros2_EEParam() : Node("ros2_EEParam") 
    {
        this->declare_parameter("EE_PARAM", "none");
        param_EE = this->get_parameter("EE_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "EE_PARAM received -> %s", param_EE.c_str());
    }
private:
};

// ======================================================================================================================== //
// ==================== FUNCTIONS ==================== //

// ===== PLAN ===== //
// ROBOT:
moveit::planning_interface::MoveGroupInterface::Plan plan_ROB() {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_ROB.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RES = "PLANNING: OK";
    }
    else
    {
        RES = "PLANNING: ERROR";
    }
    return my_plan;
}
// END-EFFECTOR:
moveit::planning_interface::MoveGroupInterface::Plan plan_EE() {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_EE.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RES = "PLANNING: OK";
    }
    else
    {
        RES = "PLANNING: ERROR (EE)";
    }
    return my_plan;
}

// ======================================================================================================================== //
// ==================== ACTION SERVER CLASS ==================== //

class ActionServer : public rclcpp::Node
{
public:
    using Move = ros2srrc_data::action::Move;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Move>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("MOVE_INTERFACE", options)
    {
        action_server_ = rclcpp_action::create_server<Move>(
            this,
            "/Move",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<Move>::SharedPtr action_server_;
    
    // Accept goal and notify which action is requested:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Move::Goal> goal)
    {
        std::string action = goal->action;
        double speed = goal->speed;
        if (action == "MoveJ"){
            auto MoveJGoal = goal->movej;
            RCLCPP_INFO(this->get_logger(), "Received GOAL: MoveJ -> (%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
                        MoveJGoal.joint1, MoveJGoal.joint2, MoveJGoal.joint3,
                        MoveJGoal.joint4, MoveJGoal.joint5, MoveJGoal.joint6);
        }
        // ... (other actions omitted for brevity) ...
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }

    // Handle accepted goal in a separate thread:
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread([this, goal_handle]() {
            execute(goal_handle);
        }).detach();
    }

    // Handle cancel request:
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request.");
        if (param_ROB != "none"){
            move_group_interface_ROB.stop();
        }
        if (param_EE != "none" && param_ENV != "bringup"){
            move_group_interface_EE.stop();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Main execution loop:
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        std::string action = goal->action;
        auto result = std::make_shared<Move::Result>();
        moveit::planning_interface::MoveGroupInterface::Plan MyPlan;
        
        if (action == "MoveJ" && param_ROB != "none"){
            std::vector<double> JP;
            moveit::core::RobotStatePtr current_state = move_group_interface_ROB.getCurrentState(10);
            current_state->copyJointGroupPositions(joint_model_group_ROB, JP);
            MoveJSTRUCT MoveJRES = MoveJAction(goal->movej, JP, robotSPECS);
            JP = MoveJRES.JP;
            move_group_interface_ROB.setJointValueTarget(JP);
            move_group_interface_ROB.setMaxVelocityScalingFactor(goal->speed);
            move_group_interface_ROB.setPlannerId("PTP");
            if (MoveJRES.RES == "LIMITS: OK"){
                MyPlan = plan_ROB();
            } else {
                RES = MoveJRES.RES;
            }
        }
        else if (action == "MoveL" && param_ROB != "none"){
            auto POSE = move_group_interface_ROB.getCurrentPose();
            auto TARGET_POSE = MoveLAction(goal->movel, POSE);
            move_group_interface_ROB.setPoseTarget(TARGET_POSE);
            move_group_interface_ROB.setMaxVelocityScalingFactor(goal->speed);
            move_group_interface_ROB.setPlannerId("LIN");
            MyPlan = plan_ROB();
        }
        // ... (other actions omitted for brevity) ...
        else if (action == "MoveG" && param_EE != "none"){
            std::vector<double> JP;
            moveit::core::RobotStatePtr current_state = move_group_interface_EE.getCurrentState(10);
            current_state->copyJointGroupPositions(joint_model_group_EE, JP);
            MoveGSTRUCT MoveGRES = MoveGAction(goal->moveg, JP, eeSPECS);
            JP = MoveGRES.JP;
            move_group_interface_EE.setJointValueTarget(JP);
            move_group_interface_EE.setMaxVelocityScalingFactor(goal->speed);
            move_group_interface_EE.setPlannerId("PTP");
            if (MoveGRES.RES == "LIMITS: OK"){
                MyPlan = plan_EE();
            } else {
                RES = MoveGRES.RES;
            }
        }

        // EXECUTION using the current API (move())
        if ((RES == "PLANNING: OK") || (RES == "PLANNING: OK (EE)")){
            moveit::core::MoveItErrorCode exec_result;
            if (action == "MoveG" && param_EE != "none")
                exec_result = move_group_interface_EE.move();
            else
                exec_result = move_group_interface_ROB.move();
            
            if (goal_handle->is_canceling()){
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->result = action + ":CANCELED";
                goal_handle->canceled(result);
                return;
            }
            
            if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS){
                RCLCPP_INFO(this->get_logger(), "%s - %s: Movement executed!",
                            (action == "MoveG") ? param_EE.c_str() : param_ROB.c_str(), action.c_str());
                result->result = action + ":SUCCESS";
                goal_handle->succeed(result);
            } else {
                RCLCPP_INFO(this->get_logger(), "%s - %s: Movement execution failed!",
                            (action == "MoveG") ? param_EE.c_str() : param_ROB.c_str(), action.c_str());
                result->result = action + ":FAILED. Reason -> Execution error.";
                goal_handle->succeed(result);
            }
        }
        else if (RES == "PLANNING: ERROR" || RES == "PLANNING: ERROR (EE)"){
            RCLCPP_INFO(this->get_logger(), "%s - %s: Planning failed!",
                        (action == "MoveG") ? param_EE.c_str() : param_ROB.c_str(), action.c_str());
            result->result = action + ":FAILED. Reason -> Planning failed.";
            goal_handle->succeed(result);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "ERROR: %s", RES.c_str());
            result->result = action + ":FAILED. Reason -> " + RES;
            goal_handle->succeed(result);
        }
        
        // Reinitialize RES variable:
        RES = "none";
    }
};

// ======================================================================================================================== //
// ==================== MAIN ==================== //

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto const logger = rclcpp::get_logger("MOVE_INTERFACE");

    // Obtain ROBOT and END-EFFECTOR parameters:
    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);
    auto node_PARAM_EE = std::make_shared<ros2_EEParam>();
    rclcpp::spin_some(node_PARAM_EE);

    // Load Robot SPECIFICATIONS:
    if (param_ROB != "none"){
        std::string pkgPATH_R = ament_index_cpp::get_package_share_directory("ros2srrc_robots");
        std::string PATH_R = pkgPATH_R + "/" + param_ROB + "/config/joint_specifications.yaml";
        YAML::Node SPECIFICATIONS_R = YAML::LoadFile(PATH_R);
        robotSPECS.robot_max = SPECIFICATIONS_R["Limits"]["Max"].as<std::vector<double>>();
        robotSPECS.robot_min = SPECIFICATIONS_R["Limits"]["Min"].as<std::vector<double>>();
    }
    // Load End-Effector SPECIFICATIONS:
    if (param_EE != "none"){
        std::string pkgPATH = ament_index_cpp::get_package_share_directory("ros2srrc_endeffectors");
        std::string PATH = pkgPATH + "/" + param_EE + "/config/joint_specifications.yaml";
        YAML::Node SPECIFICATIONS = YAML::LoadFile(PATH);
        eeSPECS.ee_max = SPECIFICATIONS["Limits"]["Max"].as<double>();
        eeSPECS.ee_min = SPECIFICATIONS["Limits"]["Min"].as<double>();
        eeSPECS.ee_vector = SPECIFICATIONS["JointsVector"].as<std::vector<double>>();
    }

    // Create a node for MoveGroupInterface(s)
    auto node2 = std::make_shared<rclcpp::Node>("ros2srrc_move", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(node2);
    std::thread([&executor]() { executor.spin(); }).detach();

    using moveit::planning_interface::MoveGroupInterface;
    // Create ROBOT MoveGroupInterface:
    if (param_ROB != "none"){
        std::string group_name = param_ROB + "_arm";
        move_group_interface_ROB = MoveGroupInterface(node2, group_name);
        move_group_interface_ROB.setPlanningPipelineId("move_group");
        move_group_interface_ROB.setMaxVelocityScalingFactor(1.0);
        move_group_interface_ROB.setMaxAccelerationScalingFactor(1.0);
        joint_model_group_ROB = move_group_interface_ROB.getCurrentState()->getJointModelGroup(group_name);
        RCLCPP_INFO(logger, "Created MoveGroupInterface for ROBOT: %s", param_ROB.c_str());
    }
    // Create END-EFFECTOR MoveGroupInterface:
    if (param_EE != "none"){
        move_group_interface_EE = MoveGroupInterface(node2, param_EE);
        move_group_interface_EE.setPlanningPipelineId("move_group");
        move_group_interface_EE.setMaxVelocityScalingFactor(1.0);
        move_group_interface_EE.setMaxAccelerationScalingFactor(1.0);
        joint_model_group_EE = move_group_interface_EE.getCurrentState()->getJointModelGroup(param_EE);
        RCLCPP_INFO(logger, "Created MoveGroupInterface for END-EFFECTOR: %s", param_EE.c_str());
    }

    using moveit::planning_interface::PlanningSceneInterface;
    PlanningSceneInterface planning_scene_interface;

    // Create and spin ACTION SERVER:
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}
