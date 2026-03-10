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
#include <condition_variable>
#include <mutex>

// Custom messages (you may need to create these)
#include <dual_arms_msgs/action/execute_task.hpp>
#include <dual_arms_msgs/srv/get_handover_zone.hpp>
#include <dual_arms_msgs/srv/resolve_collision.hpp>

// Custom headers
#include <dual_arms_mtc/control_strategy.hpp>

using namespace moveit::task_constructor;

namespace dual_arms_mtc {

// Enum for control modes
enum class ControlMode {
    MULTITHREADED,        // Direct arm control via multithreading (pick/place)
    SEQUENTIAL_HANDOVER,  // MTC-based sequential handover control
    PARALLEL_PICK_PLACE   // MTC-based parallel pick/place control
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

    // Initialization
    void init();

    // Zone detection and mode switching
    bool is_in_handover_zone(const geometry_msgs::msg::Pose& pose);
    void switch_to_mtc_mode();
    void switch_to_multithreaded_mode();
    ControlMode get_current_mode() const;

    // ========================================================================
    // DYNAMIC COLLISION RESOLUTION - NEW
    // ========================================================================
    void handle_resolve_collision(
        const std::shared_ptr<dual_arms_msgs::srv::ResolveCollision::Request> request,
        std::shared_ptr<dual_arms_msgs::srv::ResolveCollision::Response> response);

    moveit::task_constructor::Task create_safe_resolution_task(
        const geometry_msgs::msg::Pose& ar4_target,
        const geometry_msgs::msg::Pose& abb_target);

    // ========================================================================
    // OPERATION TYPE MANAGEMENT - NEW FOR SEQUENTIAL/PARALLEL CONTROL
    // ========================================================================
    
    /**
     * @brief Initialize operation context for a new task
     * @param operation_type The type of operation (HANDOVER or PICK_PLACE)
     * @return Generated operation ID
     */
    std::string initialize_operation(OperationType operation_type);
    
    /**
     * @brief Get current operation context
     */
    OperationContext get_operation_context() const;
    
    /**
     * @brief Check if operation is sequential (handover) or parallel (pick/place)
     */
    ExecutionModel get_execution_model(OperationType op_type) const;
    
    /**
     * @brief Update operation phase (for sequential handover coordination)
     */
    void update_operation_phase(OperationPhase new_phase);
    
    /**
     * @brief Signal that ARM1 (AR4) has completed its task
     */
    void signal_arm1_complete();
    
    /**
     * @brief Wait for ARM1 (AR4) to complete before ARM2 starts (sequential handover only)
     * @param timeout_ms Maximum time to wait
     * @return true if ARM1 completed, false if timeout
     */
    bool wait_for_arm1_completion(uint64_t timeout_ms = 5000);
    
    /**
     * @brief Signal that ARM2 (ABB) is ready to proceed (sequential handover only)
     */
    void signal_arm2_ready();
    
    /**
     * @brief Begin sequential handover operation
     */
    void begin_sequential_handover(
        const geometry_msgs::msg::Pose& ar4_start,
        const geometry_msgs::msg::Pose& abb_start,
        const geometry_msgs::msg::Pose& handover_point,
        const geometry_msgs::msg::Pose& intermediate_pose);
    
    /**
     * @brief Begin parallel pick/place operation
     */
    void begin_parallel_pick_place(
        const geometry_msgs::msg::Pose& ar4_target,
        const geometry_msgs::msg::Pose& abb_target);

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
    
    // ========================================================================
    // OPERATION TRACKING - NEW FOR SEQUENTIAL/PARALLEL CONTROL
    // ========================================================================
    
    // Current operation context
    OperationContext current_operation_;
    mutable std::mutex operation_mutex_;
    
    // Synchronization primitives for sequential handover
    std::condition_variable arm1_completion_cv_;
    std::condition_variable arm2_ready_cv_;
    std::atomic<bool> arm1_completed_{false};
    std::atomic<bool> arm2_ready_{false};
    std::atomic<uint64_t> operation_start_time_{0};

    // Action clients
    rclcpp_action::Client<dual_arms_msgs::action::ExecuteTask>::SharedPtr ar4_action_client_;
    rclcpp_action::Client<dual_arms_msgs::action::ExecuteTask>::SharedPtr abb_action_client_;

    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ar4_gripper_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr abb_gripper_client_;
    rclcpp::Client<dual_arms_msgs::srv::GetHandoverZone>::SharedPtr zone_client_;

    // MTC Resolution Service Server
    rclcpp::Service<dual_arms_msgs::srv::ResolveCollision>::SharedPtr safe_resolution_service_;
    // Methods
    void initialize_solvers();
    void load_handover_zone_config();
    std::vector<moveit::task_constructor::StagePtr> 
        create_arm_movements(const std::string& arm_name,
                            const geometry_msgs::msg::Pose& start,
                            const geometry_msgs::msg::Pose& target,
                            bool synchronized = false);
    
    // ========================================================================
    // SEQUENTIAL/PARALLEL EXECUTION HELPERS - NEW
    // ========================================================================
    
    /**
     * @brief Execute sequential handover operation phases
     * Phase 1: AR4 picks brick and moves to intermediate pose
     * Phase 2: ABB waits for AR4 completion, then picks from intermediate pose
     * Phase 3: AR4 releases and retracts, ABB moves to place position
     */
    Task create_sequential_handover_task(
        const geometry_msgs::msg::Pose& ar4_start,
        const geometry_msgs::msg::Pose& abb_start,
        const geometry_msgs::msg::Pose& intermediate_pose,
        const geometry_msgs::msg::Pose& abb_target);
    
    /**
     * @brief Create synchronized approach task for parallel pick/place
     * Both arms move simultaneously without collision risk
     */
    Task create_parallel_pick_place_task(
        const geometry_msgs::msg::Pose& ar4_target,
        const geometry_msgs::msg::Pose& abb_target);
    
    /**
     * @brief Monitor and enforce execution model (sequential vs parallel)
     */
    void enforce_execution_model();
    
    /**
     * @brief Validate operation sequence timing
     */
    bool is_operation_phase_valid();
    
    /**
     * @brief Generate unique operation ID
     */
    std::string generate_operation_id();
    
    void on_task_complete();
    void on_task_failed(const std::string& reason);
};

}  // namespace dual_arms_mtc
