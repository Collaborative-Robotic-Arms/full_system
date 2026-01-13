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
#include <array>
#include <vector>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::atomic<bool> running(true);

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting...\n";
    running = false;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);

    auto node = std::make_shared<rclcpp::Node>("AR4_tool_control_final");

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    auto logger = rclcpp::get_logger("AR4_tool_control_final");

    MoveGroupInterface move_group_interface(node, "ar_manipulator");

    move_group_interface.setMaxVelocityScalingFactor(0.7);
    move_group_interface.setMaxAccelerationScalingFactor(0.3);
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.01);
    move_group_interface.setPlanningTime(10.0);

    // UPDATED NAMES BASED ON TF TREE
    const std::string ee_link = "ar4_ee_link";    // Actual physical link 
    const std::string ar4_base = "ar4_base_link";  // AR4 root 
    const std::string abb_base = "base_link";      // ABB root
    const double AR4_MAX_REACH = 0.6; 

    double x=0, y=0, z=0;
    double roll_deg=0, pitch_deg=0, yaw_deg=0;
    char option=0;

    while (rclcpp::ok() && running) {
        rclcpp::spin_some(node);

        auto current_state = move_group_interface.getCurrentState(1.0);
        if (!current_state) {
            RCLCPP_WARN(logger, "Waiting for robot state update...");
            std::this_thread::sleep_for(500ms);
            continue;
        }

        geometry_msgs::msg::PoseStamped curr_pose = move_group_interface.getCurrentPose(ee_link);
        std::cout << "\n------------------------------------------------";
        std::cout << "\nROBOT: AR4 | STATE: READY";
        std::cout << "\nEE Position (world) -> X: " << curr_pose.pose.position.x 
                  << " Y: " << curr_pose.pose.position.y 
                  << " Z: " << curr_pose.pose.position.z;
        std::cout << "\n------------------------------------------------";

        std::cout << "\nHome (H), Pose (P), Quit (Q): ";
        std::cin >> option;

        if (option == 'H' || option == 'h') {
            RCLCPP_INFO(logger, "Syncing state and homing joints...");
            move_group_interface.setStartState(*current_state);
            
            std::vector<double> joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            move_group_interface.setJointValueTarget(joint_group_positions);

            MoveGroupInterface::Plan msg;
            if(static_cast<bool>(move_group_interface.plan(msg))) {
                move_group_interface.execute(msg);
            } else {
                RCLCPP_ERROR(logger,"Homing failed!");
            }
        } 
        else if (option == 'P' || option == 'p') {
            std::cout << "\nEnter target relative to ABB base_link (x y z R P Y): ";
            if (!(std::cin >> x >> y >> z >> roll_deg >> pitch_deg >> yaw_deg)) {
                std::cout << "Invalid input.\n";
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            double roll = roll_deg * M_PI / 180.0;
            double pitch = pitch_deg * M_PI / 180.0;
            double yaw = yaw_deg * M_PI / 180.0;
            tf2::Quaternion q_target;
            q_target.setRPY(roll, pitch, yaw);
            q_target.normalize();

            geometry_msgs::msg::PoseStamped target_in_abb;
            target_in_abb.header.frame_id = abb_base; // Now using base_link
            target_in_abb.header.stamp = node->now();
            target_in_abb.pose.position.x = x;
            target_in_abb.pose.position.y = y;
            target_in_abb.pose.position.z = z;
            target_in_abb.pose.orientation = tf2::toMsg(q_target);

            geometry_msgs::msg::PoseStamped target_in_ar4;
            try {
                // Transform from ABB base_link to ar4_base_link
                target_in_ar4 = tf_buffer.transform(target_in_abb, ar4_base, 1s);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(logger, "TF Error: %s", ex.what());
                continue;
            }

            // SAFETY CHECK
            double tx = target_in_ar4.pose.position.x;
            double ty = target_in_ar4.pose.position.y;
            double tz = target_in_ar4.pose.position.z;
            double dist = std::sqrt(tx*tx + ty*ty + tz*tz);

            RCLCPP_INFO(logger, "Calculated distance from AR4 base: %.3f meters", dist);

            if (dist > AR4_MAX_REACH) {
                RCLCPP_ERROR(logger, "OUT OF REACH! Distance %.2fm > limit %.2fm.", dist, AR4_MAX_REACH);
                continue;
            }

            move_group_interface.setStartState(*current_state);
            move_group_interface.clearPoseTargets();
            move_group_interface.setPoseTarget(target_in_ar4, ee_link);

            MoveGroupInterface::Plan plan;
            if (static_cast<bool>(move_group_interface.plan(plan))) {
                RCLCPP_INFO(logger, "Plan found. Executing...");
                move_group_interface.execute(plan);
            } else {
                RCLCPP_ERROR(logger, "Planning failed! Check for collisions or IK singularities.");
            }
        }
        else if (option == 'Q' || option == 'q') {
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}