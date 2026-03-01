#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <atomic>
#include <thread>

// Custom messages (you may need to create these)
#include <dual_arms_msgs/action/execute_task.hpp>
#include <dual_arms_msgs/srv/get_handover_zone.hpp>

using namespace moveit::task_constructor;

namespace dual_arms_mtc {

// Enum for control modes
enum class ControlMode {
    MULTITHREADED,  // Direct arm control via multithreading
    MTC_HANDOVER    // MTC-based collaborative control
};

// Structure for handover zone configuration
struct HandoverZone {
    geometry_msgs::msg::Pose center;
    double radius_x;  // Half-size in X
    double radius_y;  // Half-size in Y
    double radius_z;  // Half-size in Z
};

class HybridMTCController : public rclcpp::Node {
public:
    HybridMTCController();
    ~HybridMTCController();

    // Zone detection and mode switching
    bool is_in_handover_zone(const geometry_msgs::msg::Pose& pose);
    void switch_to_mtc_mode();
    void switch_to_multithreaded_mode();
    ControlMode get_current_mode() const;

    // MTC task creation methods
    Task create_collaborative_handover_task(
        const geometry_msgs::msg::Pose& ar4_start,
        const geometry_msgs::msg::Pose& abb_start,
        const geometry_msgs::msg::Pose& handover_point);

    Task create_synchronized_approach_task(
        const geometry_msgs::msg::Pose& ar4_target,
        const geometry_msgs::msg::Pose& abb_target);

    Task create_handover_transfer_task(
        const geometry_msgs::msg::Pose& transfer_pose);

    // Gripper control
    bool set_ar4_gripper(bool open);
    bool set_abb_gripper(bool open);

    // Status callbacks
    bool is_task_complete() const;
    bool has_task_failed() const;
    std::string get_task_status() const;

private:
    // Solvers for motion planning
    std::shared_ptr<solvers::PipelinePlanner> planning_pipeline_;
    
    // TF2 for zone detection
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Zone configuration
    HandoverZone handover_zone_;
    
    // Control state
    std::atomic<ControlMode> current_mode_;
    std::atomic<bool> task_active_;
    std::atomic<bool> task_failed_;
    std::string task_status_;

    // Action clients
    rclcpp_action::Client<dual_arms_msgs::action::ExecuteTask>::SharedPtr ar4_action_client_;
    rclcpp_action::Client<dual_arms_msgs::action::ExecuteTask>::SharedPtr abb_action_client_;

    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ar4_gripper_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr abb_gripper_client_;
    rclcpp::Client<dual_arms_msgs::srv::GetHandoverZone>::SharedPtr zone_client_;

    // Methods
    void initialize_solvers();
    void load_handover_zone_config();
    std::vector<moveit::task_constructor::StagePtr> 
        create_arm_movements(const std::string& arm_name,
                            const geometry_msgs::msg::Pose& start,
                            const geometry_msgs::msg::Pose& target,
                            bool synchronized = false);
    
    void on_task_complete();
    void on_task_failed(const std::string& reason);
};

}  // namespace dual_arms_mtc
