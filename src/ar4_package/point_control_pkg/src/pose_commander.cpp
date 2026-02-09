#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "moveit/move_group_interface/move_group_interface.hpp"

// Import your custom action definition
#include "supervisor_package/action/move_to_pose.hpp"

using MoveToPose = supervisor_package::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
using moveit::planning_interface::MoveGroupInterface;

class AR4ActionServer : public rclcpp::Node {
public:
    AR4ActionServer() : Node("AR4_pose_commander"), tf_buffer_(this->get_clock()) {
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        RCLCPP_INFO(this->get_logger(), "Node created, awaiting initialization...");
    }
    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif
    // This is the missing init() method the compiler couldn't find
    void init() {
        // Now it is safe to use shared_from_this()
        move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "ar_manipulator");

        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setMaxVelocityScalingFactor(0.7);
        move_group_->setMaxAccelerationScalingFactor(0.3);
        move_group_->setGoalPositionTolerance(0.001);
        move_group_->setGoalOrientationTolerance(0.01);
        move_group_->setGoalJointTolerance(0.001);

        this->action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            "ar4_point_control",
            std::bind(&AR4ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AR4ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&AR4ActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "AR4 Action Server initialized and ready.");
    }

private:
    std::shared_ptr<MoveGroupInterface> move_group_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToPose::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received strategy: %s", goal->strategy.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        (void)goal_handle;
        if (move_group_) move_group_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        std::thread{std::bind(&AR4ActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveToPose::Result>();

        // 1. Handle Named Targets (like HOME)
        if (goal->strategy == "HOME") {
            RCLCPP_INFO(this->get_logger(), "Strategy: HOME - Moving to named target");
            move_group_->setNamedTarget("home");
        } 
        // 2. Handle Pose Targets (APPROACH, GRASP, PLACE, etc.)
        else {
            RCLCPP_INFO(this->get_logger(), "Strategy: %s - Processing Pose Target", goal->strategy.c_str());

            // Create a PoseStamped from the goal request
            geometry_msgs::msg::PoseStamped target_msg;
            target_msg.header.stamp = this->now();
            // Ensure this matches what your Mock/Supervisor uses (e.g., "base_link" or "world")
            target_msg.header.frame_id = "ABB_base_link"; 
            target_msg.pose = goal->target_pose;
            
            // Log the incoming coordinates for debugging
            RCLCPP_INFO(this->get_logger(), "Received Target position[base_link]: x=%.3f, y=%.3f, z=%.3f", 
                        target_msg.pose.position.x, target_msg.pose.position.y, target_msg.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Received Target orientation[base_link]: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                        target_msg.pose.orientation.x, target_msg.pose.orientation.y, target_msg.pose.orientation.z, target_msg.pose.orientation.w);

            geometry_msgs::msg::PoseStamped transformed_goal;
            try {
                // Transform the goal into the robot's specific planning frame (e.g., "link0" or "ar4_base")
                // This is what made your manual script work!
                transformed_goal = tf_buffer_.transform(target_msg, move_group_->getPlanningFrame(), tf2::durationFromSec(1.0));
                
                RCLCPP_INFO(this->get_logger(), "Transformed Target [%s]: x=%.3f, y=%.3f, z=%.3f", 
                            move_group_->getPlanningFrame().c_str(),
                            transformed_goal.pose.position.x, transformed_goal.pose.position.y, transformed_goal.pose.position.z);
                RCLCPP_INFO(this->get_logger(), "RECEIVED Quat -> w: %.3f, x: %.3f, y: %.3f, z: %.3f",
                            transformed_goal.pose.orientation.w, transformed_goal.pose.orientation.x,
                            transformed_goal.pose.orientation.y, transformed_goal.pose.orientation.z);
            } 
            catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "TF Transform failed: %s", ex.what());
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            // Set Workspace to avoid "Planning volume not specified" warnings
            // Parameters: minX, minY, minZ, maxX, maxY, maxZ
            move_group_->setWorkspace(-1.5, -1.5, -0.5, 1.5, 1.5, 2.0);

            move_group_->clearPoseTargets();
            move_group_->setPoseTarget(transformed_goal);
            // move_group_->setPositionTarget(transformed_goal.pose.position.x,
            //                               transformed_goal.pose.position.y,
            //                               transformed_goal.pose.position.z);
            // move_group_->setOrientationTarget(transformed_goal.pose.orientation.x, transformed_goal.pose.orientation.y, 
            //                          transformed_goal.pose.orientation.z, transformed_goal.pose.orientation.w);
        }

        // 3. Plan and Execute
        move_group_->setStartStateToCurrentState();
        
        MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing movement...");
            
            auto move_result = move_group_->execute(plan);
            
            if (move_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Execution complete.");
                result->success = true;
                goal_handle->succeed(result);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Execution failed!");
                result->success = false;
                goal_handle->abort(result);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed! Goal might be unreachable or in collision.");
            result->success = false;
            goal_handle->abort(result);
        }

        // Always stop and clear targets after an attempt
        move_group_->stop();
        move_group_->clearPoseTargets();
    }
};

int main(int argc, char **argv) {
    // FIXED: Changed rclpy to rclcpp
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AR4ActionServer>();
    
    // This will now work because the init() method is defined above
    node->init();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}