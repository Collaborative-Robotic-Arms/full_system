#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::atomic<bool> running(true);

void signalHandler(int signum) {
    (void)signum;
    running = false;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);

    // FIX: Initialize node with "automatically_declare_parameters_from_overrides" 
    // This allows setting the timing parameters without a manual declaration.
    // rclcpp::NodeOptions node_options;
    // node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("AR4_tool_control_final");
    
    // TIMING FIX: Setting the age limit to resolve the 1ms simulation race condition
    // node->set_parameter(rclcpp::Parameter("joint_state_monitor.max_joint_state_age", 1.0));

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    auto logger = rclcpp::get_logger("AR4_tool_control_final");
    
    // Initialize MoveGroup Interface
    MoveGroupInterface move_group_interface(node, "ar_manipulator");
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.01);
    move_group_interface.setPlanningTime(10.0);

    // Frame setup based on dual_arms_with_environment.xacro 
    const std::string ee_link = "ar4_ee_link";    
    const std::string ar4_base = "ar4_base_link"; 
    const std::string abb_base = "base_link"; 
    const double AR4_MAX_REACH = 0.6; 

    while (rclcpp::ok() && running) {
        rclcpp::spin_some(node);

        // Fetch state with a 2s timeout for simulation clock synchronization
        // auto current_state = move_group_interface.getCurrentState(2.0); 
        
        // if (!current_state) {
        //     RCLCPP_WARN(logger, "Synchronizing with Gazebo... Waiting for fresh joint states.");
        //     std::this_thread::sleep_for(500ms);
        //     continue;
        // }

        // EE Position Feedback
        geometry_msgs::msg::PoseStamped curr_pose = move_group_interface.getCurrentPose(ee_link);
        std::cout << "\n================================================";
        std::cout << "\nROBOT: AR4 | STATE: READY";
        std::cout << "\nEE Position (world) -> X: " << curr_pose.pose.position.x 
                  << " Y: " << curr_pose.pose.position.y 
                  << " Z: " << curr_pose.pose.position.z;
        std::cout << "\n================================================\n";

        char option;
        std::cout << "Home (H), Pose (P), Quit (Q): " << std::flush;
        
        if (!(std::cin >> option)) break;

        if (option == 'H' || option == 'h') {
            // move_group_interface.setStartStateToCurrentState();
            std::vector<double> joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            move_group_interface.setJointValueTarget(joint_group_positions);

            MoveGroupInterface::Plan msg;
            if(static_cast<bool>(move_group_interface.plan(msg))) {
                move_group_interface.execute(msg);
            } else {
                RCLCPP_ERROR(logger, "Homing failed!");
            }
        } 
        else if (option == 'P' || option == 'p') {
            double x, y, z, r, p, yaw;
            std::cout << "\nTarget relative to ABB base_link (x y z R P Y): ";
            if (!(std::cin >> x >> y >> z >> r >> p >> yaw)) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = abb_base;
            target_pose.header.stamp = node->now();
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;

            tf2::Quaternion q;
            q.setRPY(r*M_PI/180.0, p*M_PI/180.0, yaw*M_PI/180.0);
            target_pose.pose.orientation = tf2::toMsg(q);

            geometry_msgs::msg::PoseStamped target_in_ar4;
            try {
                // Transform target from ABB base_link to AR4 base_link 
                target_in_ar4 = tf_buffer.transform(target_pose, ar4_base, 1s);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(logger, "TF Transform Error: %s", ex.what());
                continue;
            }

            double dist = std::sqrt(std::pow(target_in_ar4.pose.position.x, 2) + 
                                    std::pow(target_in_ar4.pose.position.y, 2) + 
                                    std::pow(target_in_ar4.pose.position.z, 2));

            RCLCPP_INFO(logger, "Target distance from AR4 base: %.3f meters", dist);

            if (dist > AR4_MAX_REACH) {
                RCLCPP_ERROR(logger, "OUT OF REACH! Distance %.2fm > limit %.2fm.", dist, AR4_MAX_REACH);
                continue;
            }

            // move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPoseTarget(target_in_ar4, ee_link);

            MoveGroupInterface::Plan plan;
            if (static_cast<bool>(move_group_interface.plan(plan))) {
                RCLCPP_INFO(logger, "Plan found. Executing move...");
                move_group_interface.execute(plan);
            } else {
                RCLCPP_ERROR(logger, "Planning failed!");
            }
        }
        else if (option == 'Q' || option == 'q') {
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}