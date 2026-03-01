#include <dual_arms_mtc/hybrid_mtc_controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <rclcpp_action/rclcpp_action.hpp>

namespace dual_arms_mtc {

HybridMTCController::HybridMTCController()
    : Node("hybrid_mtc_controller"),
      current_mode_(ControlMode::MULTITHREADED),
      task_active_(false),
      task_failed_(false),
      task_status_("IDLE") {
    
    RCLCPP_INFO(get_logger(), "Initializing Hybrid MTC Controller...");

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize solvers
    initialize_solvers();

    // Load handover zone configuration
    load_handover_zone_config();

    // Create action clients for both arms
    ar4_action_client_ = rclcpp_action::create_client<dual_arms_msgs::action::ExecuteTask>(
        this, "ar4_controller/execute_task");
    abb_action_client_ = rclcpp_action::create_client<dual_arms_msgs::action::ExecuteTask>(
        this, "abb_controller/execute_task");

    // Create service clients for grippers
    ar4_gripper_client_ = create_client<std_srvs::srv::SetBool>("ar4_gripper/set");
    abb_gripper_client_ = create_client<std_srvs::srv::SetBool>("abb_gripper/set");

    RCLCPP_INFO(get_logger(), "Hybrid MTC Controller initialized successfully");
}

HybridMTCController::~HybridMTCController() {
    RCLCPP_INFO(get_logger(), "Shutting down Hybrid MTC Controller");
}

void HybridMTCController::initialize_solvers() {
    // Create a pipeline planner that uses MoveIt's motion planning pipeline
    planning_pipeline_ = std::make_shared<solvers::PipelinePlanner>(shared_from_this());
    
    // Optional: Configure solver parameters
    // planning_pipeline_->setProperty("timeout", 5.0);
    
    RCLCPP_INFO(get_logger(), "Solvers initialized");
}

void HybridMTCController::load_handover_zone_config() {
    // Load handover zone parameters from ROS parameters
    declare_parameter("handover_zone.center.x", 0.5);
    declare_parameter("handover_zone.center.y", 0.0);
    declare_parameter("handover_zone.center.z", 0.3);
    declare_parameter("handover_zone.radius_x", 0.2);
    declare_parameter("handover_zone.radius_y", 0.2);
    declare_parameter("handover_zone.radius_z", 0.2);

    handover_zone_.center.position.x = get_parameter("handover_zone.center.x").as_double();
    handover_zone_.center.position.y = get_parameter("handover_zone.center.y").as_double();
    handover_zone_.center.position.z = get_parameter("handover_zone.center.z").as_double();
    handover_zone_.radius_x = get_parameter("handover_zone.radius_x").as_double();
    handover_zone_.radius_y = get_parameter("handover_zone.radius_y").as_double();
    handover_zone_.radius_z = get_parameter("handover_zone.radius_z").as_double();

    RCLCPP_INFO(get_logger(), 
        "Handover Zone loaded - Center: (%.2f, %.2f, %.2f), Radius: (%.2f, %.2f, %.2f)",
        handover_zone_.center.position.x, handover_zone_.center.position.y, 
        handover_zone_.center.position.z, handover_zone_.radius_x, handover_zone_.radius_y, 
        handover_zone_.radius_z);
}

bool HybridMTCController::is_in_handover_zone(const geometry_msgs::msg::Pose& pose) {
    double dx = pose.position.x - handover_zone_.center.position.x;
    double dy = pose.position.y - handover_zone_.center.position.y;
    double dz = pose.position.z - handover_zone_.center.position.z;

    bool in_zone = 
        (std::abs(dx) <= handover_zone_.radius_x) &&
        (std::abs(dy) <= handover_zone_.radius_y) &&
        (std::abs(dz) <= handover_zone_.radius_z);

    if (in_zone) {
        RCLCPP_DEBUG(get_logger(), "Pose (%.2f, %.2f, %.2f) is in handover zone", 
            pose.position.x, pose.position.y, pose.position.z);
    }

    return in_zone;
}

void HybridMTCController::switch_to_mtc_mode() {
    if (current_mode_ != ControlMode::MTC_HANDOVER) {
        current_mode_ = ControlMode::MTC_HANDOVER;
        task_status_ = "SWITCHED_TO_MTC";
        RCLCPP_WARN(get_logger(), "Switched to MTC mode for collaborative handover");
    }
}

void HybridMTCController::switch_to_multithreaded_mode() {
    if (current_mode_ != ControlMode::MULTITHREADED) {
        current_mode_ = ControlMode::MULTITHREADED;
        task_status_ = "SWITCHED_TO_MULTITHREADED";
        RCLCPP_WARN(get_logger(), "Switched to multithreaded mode");
    }
}

ControlMode HybridMTCController::get_current_mode() const {
    return current_mode_.load();
}

// ============================================================================
// MAIN MTC TASK CREATION METHODS
// ============================================================================

Task HybridMTCController::create_collaborative_handover_task(
    const geometry_msgs::msg::Pose& ar4_start,
    const geometry_msgs::msg::Pose& abb_start,
    const geometry_msgs::msg::Pose& handover_point) {

    Task t;
    t.stages()->setName("Collaborative Handover Task");

    // Stage 0: Get current state
    auto current_state = std::make_unique<stages::CurrentState>("current_state");
    t.add(std::move(current_state));

    // Stage 1: Synchronous approach - both arms move to handover simultaneously
    RCLCPP_INFO(get_logger(), "Creating synchronized approach stages");
    
    // AR4 approach
    auto ar4_approach = std::make_unique<stages::MoveRelative>("ar4_approach");
    ar4_approach->setGroup("ar4_arm");
    ar4_approach->setIKFrame("ar4_tool_link");
    ar4_approach->properties().set("trajectory_builder", std::static_pointer_cast<TrajectoryBuilder>(planning_pipeline_));
    
    // Add a small offset approach (move upward before the handover)
    geometry_msgs::msg::Vector3Stamped approach_offset;
    approach_offset.header.frame_id = "ar4_tool_link";
    approach_offset.vector.z = -0.05;  // Approach from above
    ar4_approach->setDirection(approach_offset);
    ar4_approach->setMinMaxDistance(0.0, 0.05);

    t.add(std::move(ar4_approach));

    // ABB approach (symmetric)
    auto abb_approach = std::make_unique<stages::MoveRelative>("abb_approach");
    abb_approach->setGroup("abb_arm");
    abb_approach->setIKFrame("abb_tool_link");
    abb_approach->properties().set("trajectory_builder", std::static_pointer_cast<TrajectoryBuilder>(planning_pipeline_));
    
    approach_offset.header.frame_id = "abb_tool_link";
    approach_offset.vector.z = 0.05;   // Approach from below (gripper orientation)
    abb_approach->setDirection(approach_offset);
    abb_approach->setMinMaxDistance(0.0, 0.05);

    t.add(std::move(abb_approach));

    // Stage 2: Object hand-off (detach from AR4, attach to ABB)
    auto handoff = std::make_unique<stages::ModifyPlanningScene>("brick_handoff");
    handoff->detachObject("brick", "ar4_gripper");
    handoff->attachObject("brick", "abb_gripper");
    t.add(std::move(handoff));

    // Stage 3: Retract AR4
    auto ar4_retract = std::make_unique<stages::MoveRelative>("ar4_retract");
    ar4_retract->setGroup("ar4_arm");
    ar4_retract->setIKFrame("ar4_tool_link");
    ar4_retract->properties().set("trajectory_builder", std::static_pointer_cast<TrajectoryBuilder>(planning_pipeline_));
    
    approach_offset.header.frame_id = "ar4_tool_link";
    approach_offset.vector.z = 0.15;   // Retract upward
    ar4_retract->setDirection(approach_offset);
    ar4_retract->setMinMaxDistance(0.0, 0.15);

    t.add(std::move(ar4_retract));

    // Stage 4: Return to ready pose
    auto return_to_ready = std::make_unique<stages::MoveTo>("return_to_ready");
    return_to_ready->setGroup("dual_arms");
    return_to_ready->properties().set("trajectory_builder", std::static_pointer_cast<TrajectoryBuilder>(planning_pipeline_));
    
    // Both arms return to their start positions
    // (This would require setting named poses or using current state)

    t.add(std::move(return_to_ready));

    return t;
}

Task HybridMTCController::create_synchronized_approach_task(
    const geometry_msgs::msg::Pose& ar4_target,
    const geometry_msgs::msg::Pose& abb_target) {

    Task t;
    t.stages()->setName("Synchronized Approach Task");

    // Stage 0: Current state
    auto current_state = std::make_unique<stages::CurrentState>("current_state");
    t.add(std::move(current_state));

    // Stage 1a: Connect to position both arms to handover location
    stages::Connect::GroupPlannerVector planners;
    planners.push_back({"ar4_arm", planning_pipeline_});
    planners.push_back({"abb_arm", planning_pipeline_});

    auto synchronized_move = std::make_unique<stages::Connect>("sync_approach", planners);
    t.add(std::move(synchronized_move));

    // Stage 2: Allow collision between gripper tools during handover
    auto allow_collision = std::make_unique<stages::ModifyPlanningScene>("allow_collision");
    allow_collision->allowCollisions("ar4_gripper", "abb_gripper", true);
    t.add(std::move(allow_collision));

    return t;
}

Task HybridMTCController::create_handover_transfer_task(
    const geometry_msgs::msg::Pose& transfer_pose) {

    Task t;
    t.stages()->setName("Handover Transfer Task");

    // Stage 0: Current state
    auto current_state = std::make_unique<stages::CurrentState>("current_state");
    t.add(std::move(current_state));

    // Stage 1: Move to transfer pose
    auto move_to_transfer = std::make_unique<stages::MoveTo>("move_to_transfer");
    move_to_transfer->setGroup("dual_arms");
    move_to_transfer->properties().set("trajectory_builder", std::static_pointer_cast<TrajectoryBuilder>(planning_pipeline_));
    t.add(std::move(move_to_transfer));

    // Stage 2: gripper synchronization
    // Close AR4 gripper (release object)
    auto ar4_open = std::make_unique<stages::ModifyPlanningScene>("ar4_gripper_open");
    ar4_open->detachObject("brick", "ar4_gripper");
    t.add(std::move(ar4_open));

    // Close ABB gripper (grasp object)
    auto abb_close = std::make_unique<stages::ModifyPlanningScene>("abb_gripper_close");
    abb_close->attachObject("brick", "abb_gripper");
    t.add(std::move(abb_close));

    return t;
}

// ============================================================================
// GRIPPER CONTROL METHODS
// ============================================================================

bool HybridMTCController::set_ar4_gripper(bool open) {
    if (!ar4_gripper_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "AR4 gripper service not available");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = open;

    auto result = ar4_gripper_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "AR4 gripper set to %s", open ? "OPEN" : "CLOSED");
        return result.get()->success;
    }
    return false;
}

bool HybridMTCController::set_abb_gripper(bool open) {
    if (!abb_gripper_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "ABB gripper service not available");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = open;

    auto result = abb_gripper_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "ABB gripper set to %s", open ? "OPEN" : "CLOSED");
        return result.get()->success;
    }
    return false;
}

// ============================================================================
// STATUS AND COMPLETION METHODS
// ============================================================================

bool HybridMTCController::is_task_complete() const {
    return !task_active_.load();
}

bool HybridMTCController::has_task_failed() const {
    return task_failed_.load();
}

std::string HybridMTCController::get_task_status() const {
    return task_status_;
}

void HybridMTCController::on_task_complete() {
    task_active_ = false;
    task_failed_ = false;
    task_status_ = "COMPLETED";
    RCLCPP_INFO(get_logger(), "Task completed successfully");
}

void HybridMTCController::on_task_failed(const std::string& reason) {
    task_active_ = false;
    task_failed_ = true;
    task_status_ = "FAILED: " + reason;
    RCLCPP_ERROR(get_logger(), "Task failed: %s", reason.c_str());
}

}  // namespace dual_arms_mtc

// ============================================================================
// MAIN ENTRY POINT
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto controller = std::make_shared<dual_arms_mtc::HybridMTCController>();
    
    // Use MultiThreadedExecutor to handle concurrent callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}