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
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: August, 2023.                                                                  #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #
*/

// Required ROS2 headers:
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
using namespace std::chrono_literals;

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// Include the Robpose ROS2 Message:
#include <ros2srrc_data/msg/robpose.hpp>

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
//  ROBOT POSE PUBLISHER:

class RobPose_PUB : public rclcpp::Node
{
public:
    RobPose_PUB(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
        : Node("ros2srrc_RobPosePUB"), move_group_(move_group)
    {
        publisher_ = this->create_publisher<ros2srrc_data::msg::Robpose>("Robpose", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&RobPose_PUB::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto CP_INFO = move_group_->getCurrentPose();

        ros2srrc_data::msg::Robpose pose_msg;
        pose_msg.x = CP_INFO.pose.position.x;
        pose_msg.y = CP_INFO.pose.position.y;
        pose_msg.z = CP_INFO.pose.position.z;
        pose_msg.qx = CP_INFO.pose.orientation.x;
        pose_msg.qy = CP_INFO.pose.orientation.y;
        pose_msg.qz = CP_INFO.pose.orientation.z;
        pose_msg.qw = CP_INFO.pose.orientation.w;

        publisher_->publish(pose_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2srrc_data::msg::Robpose>::SharedPtr publisher_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

// ===================================================================================== //
// ======================================= MAIN ======================================== //
// ===================================================================================== //

int main(int argc, char **argv)
{
    // Initialise ROS2
    rclcpp::init(argc, argv);

    // Create a shared node for parameters
    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);  // Ensure parameters are loaded

    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Small delay to avoid race conditions

    // Retrieve ROB_PARAM safely
    std::string param_ROB = node_PARAM_ROB->get_robot_param();
    std::string ROBname = param_ROB + "_arm";

    if (param_ROB == "none")
    {
        RCLCPP_ERROR(node_PARAM_ROB->get_logger(), "Invalid ROB_PARAM received! Exiting.");
        rclcpp::shutdown();
        return -1;
    }

    // Create MoveIt!2 Node
    auto moveit_node = std::make_shared<rclcpp::Node>("ros2srrc_RobPose");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(moveit_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Initialize MoveGroupInterface with the correct robot name
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface_ROB = std::make_shared<MoveGroupInterface>(moveit_node, ROBname);

    // Launch Pose Publisher Node
    auto pose_publisher_node = std::make_shared<RobPose_PUB>(move_group_interface_ROB);
    rclcpp::spin(pose_publisher_node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}

