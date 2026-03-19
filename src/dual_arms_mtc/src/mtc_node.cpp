#include <dual_arms_mtc/hybrid_mtc_controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
namespace dual_arms_mtc {

HybridMTCController::HybridMTCController()
    : Node("hybrid_mtc_controller"),
      current_mode_(ControlMode::MULTITHREADED),
      task_active_(false),
      task_failed_(false),
      task_status_("IDLE") {
    
    RCLCPP_INFO(get_logger(), "Hybrid MTC Controller Node Created. Waiting for initialization...");
}

void HybridMTCController::init() {
    RCLCPP_INFO(get_logger(), "Initializing Hybrid MTC Controller...");

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize solvers (shared_from_this() is now safe to use here!)
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
    // Initialize the Collision Resolution Service
    safe_resolution_service_ = create_service<dual_arms_msgs::srv::ResolveCollision>(
        "mtc_controller/resolve_collision",
        std::bind(&HybridMTCController::handle_resolve_collision, this, std::placeholders::_1, std::placeholders::_2)
    );
    // Initialize the Handover Execution Service
    handover_service_ = create_service<dual_arms_msgs::srv::ExecuteMTCHandover>(
        "mtc_controller/execute_handover",
        std::bind(&HybridMTCController::handle_execute_handover, this, std::placeholders::_1, std::placeholders::_2)
    );
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
    if (current_mode_ != ControlMode::SEQUENTIAL_HANDOVER && current_mode_ != ControlMode::PARALLEL_PICK_PLACE) {
        task_status_ = "SWITCHED_TO_MTC";
        RCLCPP_WARN(get_logger(), "Switched to MTC mode");
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
    t.loadRobotModel(shared_from_this(), "robot_description");
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
    ar4_approach->properties().set("trajectory_builder", planning_pipeline_);
    
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
    abb_approach->properties().set("trajectory_builder", planning_pipeline_);
    
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
    ar4_retract->properties().set("trajectory_builder", planning_pipeline_);
    
    approach_offset.header.frame_id = "ar4_tool_link";
    approach_offset.vector.z = 0.15;   // Retract upward
    ar4_retract->setDirection(approach_offset);
    ar4_retract->setMinMaxDistance(0.0, 0.15);

    t.add(std::move(ar4_retract));

    // Stage 4: Return to ready pose
    auto return_to_ready = std::make_unique<stages::MoveTo>("return_to_ready");
    return_to_ready->setGroup("dual_arms");
    return_to_ready->properties().set("trajectory_builder", planning_pipeline_);
    
    // Both arms return to their start positions
    // (This would require setting named poses or using current state)

    t.add(std::move(return_to_ready));

    return t;
}

Task HybridMTCController::create_synchronized_approach_task(
    const geometry_msgs::msg::Pose& ar4_target,
    const geometry_msgs::msg::Pose& abb_target) {

    Task t;
    t.loadRobotModel(shared_from_this(), "robot_description");
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
    t.loadRobotModel(shared_from_this(), "robot_description");
    t.stages()->setName("Handover Transfer Task");

    // Stage 0: Current state
    auto current_state = std::make_unique<stages::CurrentState>("current_state");
    t.add(std::move(current_state));

    // Stage 1: Move to transfer pose
    auto move_to_transfer = std::make_unique<stages::MoveTo>("move_to_transfer");
    move_to_transfer->setGroup("dual_arms");
    move_to_transfer->properties().set("trajectory_builder", planning_pipeline_);
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
// SEQUENTIAL HANDOVER METHODS - NEW FOR EXCLUSIVE SEQUENTIAL CONTROL
// ============================================================================

Task HybridMTCController::create_sequential_handover_task(
    const geometry_msgs::msg::Pose& ar4_start,
    const geometry_msgs::msg::Pose& abb_start,
    const geometry_msgs::msg::Pose& intermediate_pose,
    const geometry_msgs::msg::Pose& abb_target) {

    Task t;
    t.loadRobotModel(shared_from_this(), "robot_description");
    t.stages()->setName("Sequential Handover Task");

    RCLCPP_INFO(get_logger(), "Creating SEQUENTIAL handover task (AR4 → intermediate → ABB → place)");

    // Stage 0: Get current state
    auto current_state = std::make_unique<stages::CurrentState>("current_state");
    t.add(std::move(current_state));

    // ========================================================================
    // PHASE 1: AR4 PICKS AND MOVES TO INTERMEDIATE POSITION
    // ========================================================================
    
    RCLCPP_INFO(get_logger(), "Stage 1: AR4 picks brick and moves to intermediate position");

    // Stage 1a: AR4 approaches pick location
    auto ar4_pick_approach = std::make_unique<stages::MoveTo>("ar4_pick_approach");
    ar4_pick_approach->setGroup("ar4_arm");
    ar4_pick_approach->properties().set("trajectory_builder", planning_pipeline_);
    t.add(std::move(ar4_pick_approach));

    // Stage 1b: AR4 grasps object
    auto ar4_grasp = std::make_unique<stages::ModifyPlanningScene>("ar4_grasp");
    ar4_grasp->attachObject("brick", "ar4_gripper");
    t.add(std::move(ar4_grasp));

    // Stage 1c: AR4 retracts from pick location
    auto ar4_retract_pick = std::make_unique<stages::MoveRelative>("ar4_retract_pick");
    ar4_retract_pick->setGroup("ar4_arm");
    ar4_retract_pick->setIKFrame("ar4_tool_link");
    ar4_retract_pick->properties().set("trajectory_builder", planning_pipeline_);
    
    geometry_msgs::msg::Vector3Stamped retract_offset;
    retract_offset.header.frame_id = "ar4_tool_link";
    retract_offset.vector.z = 0.10;  // Retract upward
    ar4_retract_pick->setDirection(retract_offset);
    ar4_retract_pick->setMinMaxDistance(0.0, 0.10);
    t.add(std::move(ar4_retract_pick));

    // Stage 1d: AR4 moves to intermediate handover pose
    auto ar4_to_intermediate = std::make_unique<stages::MoveTo>("ar4_to_intermediate");
    ar4_to_intermediate->setGroup("ar4_arm");
    ar4_to_intermediate->properties().set("trajectory_builder", planning_pipeline_);
    t.add(std::move(ar4_to_intermediate));

    // Signal that AR4 (ARM1) has completed its task
    RCLCPP_INFO(get_logger(), "AR4 move to intermediate complete - ARM1_COMPLETE");

    // ========================================================================
    // PHASE 2: ABB PICKS FROM INTERMEDIATE POSITION
    // ========================================================================
    
    RCLCPP_INFO(get_logger(), "Stage 2: ABB picks from intermediate position");

    // Stage 2a: ABB approaches intermediate pose
    auto abb_approach_intermediate = std::make_unique<stages::MoveTo>("abb_approach_intermediate");
    abb_approach_intermediate->setGroup("abb_arm");
    abb_approach_intermediate->properties().set("trajectory_builder", planning_pipeline_);
    t.add(std::move(abb_approach_intermediate));

    // Stage 2b: Transfer object from AR4 to ABB
    auto handoff = std::make_unique<stages::ModifyPlanningScene>("handoff_transfer");
    handoff->detachObject("brick", "ar4_gripper");
    handoff->attachObject("brick", "abb_gripper");
    t.add(std::move(handoff));

    RCLCPP_INFO(get_logger(), "Handoff complete - object transferred from AR4 to ABB");

    // Stage 2c: AR4 retracts away from handover zone
    auto ar4_retract_handover = std::make_unique<stages::MoveRelative>("ar4_retract_handover");
    ar4_retract_handover->setGroup("ar4_arm");
    ar4_retract_handover->setIKFrame("ar4_tool_link");
    ar4_retract_handover->properties().set("trajectory_builder", planning_pipeline_);
    
    retract_offset.header.frame_id = "ar4_tool_link";
    retract_offset.vector.z = 0.15;  // Retract upward further
    ar4_retract_handover->setDirection(retract_offset);
    ar4_retract_handover->setMinMaxDistance(0.0, 0.15);
    t.add(std::move(ar4_retract_handover));

    // ========================================================================
    // PHASE 3: ABB MOVES TO FINAL PLACEMENT POSITION
    // ========================================================================
    
    RCLCPP_INFO(get_logger(), "Stage 3: ABB moves to placement position");

    // Stage 3a: ABB moves to target placement location
    auto abb_to_place = std::make_unique<stages::MoveTo>("abb_to_place");
    abb_to_place->setGroup("abb_arm");
    abb_to_place->properties().set("trajectory_builder", planning_pipeline_);
    t.add(std::move(abb_to_place));

    // Stage 3b: ABB places object
    auto abb_place = std::make_unique<stages::ModifyPlanningScene>("abb_place");
    abb_place->detachObject("brick", "abb_gripper");
    t.add(std::move(abb_place));

    // Stage 3c: ABB retracts to ready position
    auto abb_retract = std::make_unique<stages::MoveRelative>("abb_retract");
    abb_retract->setGroup("abb_arm");
    abb_retract->setIKFrame("abb_tool_link");
    abb_retract->properties().set("trajectory_builder", planning_pipeline_);
    
    retract_offset.header.frame_id = "abb_tool_link";
    retract_offset.vector.z = 0.10;
    abb_retract->setDirection(retract_offset);
    abb_retract->setMinMaxDistance(0.0, 0.10);
    t.add(std::move(abb_retract));

    RCLCPP_INFO(get_logger(), "Sequential handover task created successfully");
    return t;
}

// ============================================================================
// PARALLEL PICK/PLACE METHODS - NEW FOR INDEPENDENT ARM CONTROL
// ============================================================================

Task HybridMTCController::create_parallel_pick_place_task(
    const geometry_msgs::msg::Pose& ar4_target,
    const geometry_msgs::msg::Pose& abb_target) {

    Task t;
    t.loadRobotModel(shared_from_this(), "robot_description");
    t.stages()->setName("Parallel Dual-Arm Pick/Place Task");

    RCLCPP_INFO(get_logger(), 
        "Creating PARALLEL pick/place task - AR4: (%.2f, %.2f, %.2f) | ABB: (%.2f, %.2f, %.2f)",
        ar4_target.position.x, ar4_target.position.y, ar4_target.position.z,
        abb_target.position.x, abb_target.position.y, abb_target.position.z);

    // Stage 0: Get current state
    auto current_state = std::make_unique<stages::CurrentState>("current_state");
    t.add(std::move(current_state));

    // Stage 1: Both arms move simultaneously to their respective targets
    // Using Connect stage to allow simultaneous planning and execution
    stages::Connect::GroupPlannerVector planners;
    planners.push_back({"ar4_arm", planning_pipeline_});
    planners.push_back({"abb_arm", planning_pipeline_});

    auto parallel_move = std::make_unique<stages::Connect>("parallel_movement", planners);
    parallel_move->setTimeout(10.0);  // 10 second timeout for planning
    t.add(std::move(parallel_move));

    // Stage 2: Allow collision between arms if needed
    auto allow_collision = std::make_unique<stages::ModifyPlanningScene>("allow_arm_collision");
    allow_collision->allowCollisions("ar4_arm_link", "abb_arm_link", true);
    t.add(std::move(allow_collision));

    RCLCPP_INFO(get_logger(), "Parallel pick/place task created successfully");
    return t;
}

// ============================================================================
// OPERATION MANAGEMENT METHODS - NEW FOR TRACKING AND COORDINATION
// ============================================================================

std::string HybridMTCController::generate_operation_id() {
    // Generate unique operation ID using timestamp and random component
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    std::string id = "OP" + std::to_string(timestamp % 1000000);
    return id;
}

std::string HybridMTCController::initialize_operation(OperationType operation_type) {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    current_operation_.type = operation_type;
    current_operation_.model = get_execution_model(operation_type);
    current_operation_.phase = OperationPhase::INITIATING;
    current_operation_.operation_id = generate_operation_id();
    current_operation_.start_timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    current_operation_.phase_timestamp_ms = current_operation_.start_timestamp_ms;
    
    // Reset synchronization flags
    arm1_completed_ = false;
    arm2_ready_ = false;
    
    RCLCPP_INFO(get_logger(), 
        "Operation initialized: %s | Type: %s | Model: %s",
        current_operation_.operation_id.c_str(),
        (operation_type == OperationType::HANDOVER) ? "HANDOVER" : "PICK_PLACE",
        (current_operation_.model == ExecutionModel::SEQUENTIAL) ? "SEQUENTIAL" : "PARALLEL");
    
    return current_operation_.operation_id;
}

OperationContext HybridMTCController::get_operation_context() const {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    return current_operation_;
}

ExecutionModel HybridMTCController::get_execution_model(OperationType op_type) const {
    if (op_type == OperationType::HANDOVER) {
        return ExecutionModel::SEQUENTIAL;
    } else if (op_type == OperationType::PICK_PLACE) {
        return ExecutionModel::PARALLEL;
    }
    return ExecutionModel::PARALLEL;  // Default to parallel for safety
}

void HybridMTCController::update_operation_phase(OperationPhase new_phase) {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    OperationPhase old_phase = current_operation_.phase;
    current_operation_.phase = new_phase;
    current_operation_.phase_timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::string phase_str = (new_phase == OperationPhase::INITIATING) ? "INITIATING" :
                           (new_phase == OperationPhase::ARM1_ACTIVE) ? "ARM1_ACTIVE" :
                           (new_phase == OperationPhase::ARM1_COMPLETE) ? "ARM1_COMPLETE" :
                           (new_phase == OperationPhase::ARM2_ACTIVE) ? "ARM2_ACTIVE" :
                           (new_phase == OperationPhase::ARM2_COMPLETE) ? "ARM2_COMPLETE" :
                           (new_phase == OperationPhase::COMPLETION) ? "COMPLETION" : "ERROR";
    
    RCLCPP_INFO(get_logger(), "Operation phase update: %s", phase_str.c_str());
}

void HybridMTCController::signal_arm1_complete() {
    {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        current_operation_.arm1_complete = true;
    }
    arm1_completed_ = true;
    arm1_completion_cv_.notify_all();
    
    RCLCPP_INFO(get_logger(), "ARM1 (AR4) task complete - signaling ARM2");
}

bool HybridMTCController::wait_for_arm1_completion(uint64_t timeout_ms) {
    std::unique_lock<std::mutex> lock(operation_mutex_);
    
    auto start_time = std::chrono::system_clock::now();
    
    // Wait for arm1 completion or timeout
    bool result = arm1_completion_cv_.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms),
        [this]() { return arm1_completed_.load(); });
    
    if (result) {
        RCLCPP_INFO(get_logger(), "ARM2 (ABB) proceeding after ARM1 completion");
    } else {
        RCLCPP_WARN(get_logger(), "ARM2 timeout waiting for ARM1 completion (%.0fms)", (double)timeout_ms);
    }
    
    return result;
}

void HybridMTCController::signal_arm2_ready() {
    {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        current_operation_.arm2_ready = true;
    }
    arm2_ready_ = true;
    arm2_ready_cv_.notify_all();
    
    RCLCPP_INFO(get_logger(), "ARM2 (ABB) ready for operation");
}

void HybridMTCController::begin_sequential_handover(
    const geometry_msgs::msg::Pose& ar4_start,
    const geometry_msgs::msg::Pose& abb_start,
    const geometry_msgs::msg::Pose& handover_point,
    const geometry_msgs::msg::Pose& intermediate_pose) {

    // Initialize operation as HANDOVER (which maps to SEQUENTIAL model)
    std::string op_id = initialize_operation(OperationType::HANDOVER);
    
    // Switch to sequential handover mode
    current_mode_ = ControlMode::SEQUENTIAL_HANDOVER;
    
    RCLCPP_WARN(get_logger(), 
        "🔄 BEGINNING SEQUENTIAL HANDOVER [%s]",
        op_id.c_str());
    
    RCLCPP_INFO(get_logger(), 
        "AR4 Start: (%.3f, %.3f, %.3f)", 
        ar4_start.position.x, ar4_start.position.y, ar4_start.position.z);
    RCLCPP_INFO(get_logger(), 
        "Intermediate: (%.3f, %.3f, %.3f)", 
        intermediate_pose.position.x, intermediate_pose.position.y, intermediate_pose.position.z);
    RCLCPP_INFO(get_logger(), 
        "ABB Start: (%.3f, %.3f, %.3f)", 
        abb_start.position.x, abb_start.position.y, abb_start.position.z);
    
    // Create task
    Task handover_task = create_sequential_handover_task(ar4_start, abb_start, intermediate_pose, handover_point);
    
    // Task would be executed by MoveIt Task Constructor framework
    task_active_ = true;
    task_status_ = "SEQUENTIAL_HANDOVER_EXECUTING";
}

void HybridMTCController::begin_parallel_pick_place(
    const geometry_msgs::msg::Pose& ar4_target,
    const geometry_msgs::msg::Pose& abb_target) {

    // Initialize operation as PICK_PLACE (which maps to PARALLEL model)
    std::string op_id = initialize_operation(OperationType::PICK_PLACE);
    
    // Switch to parallel pick/place mode
    current_mode_ = ControlMode::PARALLEL_PICK_PLACE;
    
    RCLCPP_WARN(get_logger(), 
        "⚡ BEGINNING PARALLEL PICK/PLACE [%s]",
        op_id.c_str());
    
    RCLCPP_INFO(get_logger(), 
        "AR4 Target: (%.3f, %.3f, %.3f)", 
        ar4_target.position.x, ar4_target.position.y, ar4_target.position.z);
    RCLCPP_INFO(get_logger(), 
        "ABB Target: (%.3f, %.3f, %.3f)", 
        abb_target.position.x, abb_target.position.y, abb_target.position.z);
    
    // Create task
    Task pickup_task = create_parallel_pick_place_task(ar4_target, abb_target);
    
    // Task would be executed by MoveIt Task Constructor framework with parallel execution
    task_active_ = true;
    task_status_ = "PARALLEL_PICK_PLACE_EXECUTING";
}

void HybridMTCController::enforce_execution_model() {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    if (current_operation_.model == ExecutionModel::SEQUENTIAL) {
        // For sequential operations, ARM2 must wait for ARM1
        if (current_operation_.phase == OperationPhase::ARM1_ACTIVE &&
            !current_operation_.arm1_complete) {
            // Block ARM2 from proceeding
            RCLCPP_DEBUG(get_logger(), "Sequential mode: ARM2 waiting for ARM1 completion");
        }
    } else if (current_operation_.model == ExecutionModel::PARALLEL) {
        // For parallel operations, both arms can proceed independently
        RCLCPP_DEBUG(get_logger(), "Parallel mode: Both arms executing independently");
    }
}

bool HybridMTCController::is_operation_phase_valid() {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    uint64_t phase_duration = current_time - current_operation_.phase_timestamp_ms;
    
    if (phase_duration > current_operation_.max_phase_duration_ms) {
        RCLCPP_ERROR(get_logger(), 
            "Phase timeout! Duration: %lu ms, Max: %lu ms",
            phase_duration, current_operation_.max_phase_duration_ms);
        return false;
    }
    
    return true;
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
void HybridMTCController::handle_resolve_collision(
    const std::shared_ptr<dual_arms_msgs::srv::ResolveCollision::Request> request,
    std::shared_ptr<dual_arms_msgs::srv::ResolveCollision::Response> response) {
    
    RCLCPP_WARN(get_logger(), "MTC TAKING OVER: Resolving dual-arm proximity conflict...");
    switch_to_mtc_mode();

    try {
        auto task = create_safe_resolution_task(request->ar4_target_pose, request->abb_target_pose);

        if (!task.plan(2)) { // Allow MTC 2 attempts to find a safe route
            response->success = false;
            response->message = "MTC Failed to find a safe resolution path.";
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
            switch_to_multithreaded_mode();
            return;
        }

        RCLCPP_INFO(get_logger(), "Safe path found. Executing 12-DOF synchronized trajectory...");
        
        // 1-Line Execution using the standard MTC Action Server
        auto result = task.execute(*task.solutions().front());
        
        if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            response->success = true;
            response->message = "Conflict resolved safely.";
            RCLCPP_INFO(get_logger(), "✅ MTC Resolution Complete.");
        } else {
            response->success = false;
            response->message = "Execution failed.";
            RCLCPP_ERROR(get_logger(), "❌ Execution Failed!");
        }

    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("MTC Exception: ") + e.what();
    }
    
    switch_to_multithreaded_mode();
}
Task HybridMTCController::create_safe_resolution_task(
    const geometry_msgs::msg::Pose& ar4_target,
    const geometry_msgs::msg::Pose& abb_target) {

    Task t;
    t.stages()->setName("Safe Conflict Resolution");
    t.loadRobotModel(shared_from_this(), "robot_description");

    // --- THE FIX: Create a fresh pipeline locally for THIS specific task ---
    // This prevents the "Robot model isn't the same" exception on repeated collisions.
    auto local_pipeline = std::make_shared<solvers::PipelinePlanner>(shared_from_this(), "ompl");
    // --- THE FIX: ADD STRICT TIMEOUTS TO PREVENT CPU FREEZING ---
    local_pipeline->setProperty("timeout", 2.0); // Stop doing math after 2 seconds!

    // Stage 0: Grab current frozen state
    auto current_state = std::make_unique<stages::CurrentState>("current_state");
    t.add(std::move(current_state));

    // Stage 1: Move AR4 to safety (ABB is treated as a static obstacle)
    auto move_ar4 = std::make_unique<stages::MoveTo>("move_ar4_safe", local_pipeline);
    move_ar4->setGroup("ar_manipulator");
    move_ar4->setIKFrame("ar4_ee_link");
    
    geometry_msgs::msg::PoseStamped ar4_stamped;
    ar4_stamped.header.frame_id = "abb_table";
    ar4_stamped.pose = ar4_target;
    move_ar4->setGoal(ar4_stamped);
    t.add(std::move(move_ar4));

    // Stage 2: Move ABB to safety (AR4's new position is treated as a static obstacle)
    auto move_abb = std::make_unique<stages::MoveTo>("move_abb_safe", local_pipeline);
    move_abb->setGroup("irb120_arm");
    move_abb->setIKFrame("tool0");
    
    geometry_msgs::msg::PoseStamped abb_stamped;
    abb_stamped.header.frame_id = "abb_table";
    abb_stamped.pose = abb_target;
    move_abb->setGoal(abb_stamped);
    t.add(std::move(move_abb));

    return t;
}

void HybridMTCController::handle_execute_handover(
    const std::shared_ptr<dual_arms_msgs::srv::ExecuteMTCHandover::Request> request,
    std::shared_ptr<dual_arms_msgs::srv::ExecuteMTCHandover::Response> response) {
    
    RCLCPP_INFO(get_logger(), "MTC TAKING OVER: Planning full collaborative handover sequence...");
    switch_to_mtc_mode();

    try {
        // Create the task using your existing method
        auto task = create_collaborative_handover_task(
            request->ar4_start_pose, 
            request->abb_start_pose, 
            request->handover_pose
        );

        if (!task.plan(3)) { // Allow 3 attempts to find a valid handover graph
            response->success = false;
            response->status_message = "MTC Failed to find a valid handover trajectory.";
            RCLCPP_ERROR(get_logger(), "%s", response->status_message.c_str());
            switch_to_multithreaded_mode();
            return;
        }

        RCLCPP_INFO(get_logger(), "Handover path found! Executing...");
        
        auto result = task.execute(*task.solutions().front());
        
        if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            response->success = true;
            response->status_message = "Handover executed successfully.";
            response->execution_id = generate_operation_id(); 
        } else {
            response->success = false;
            response->status_message = "Execution failed.";
        }

    } catch (const std::exception& e) {
        response->success = false;
        response->status_message = std::string("MTC Exception: ") + e.what();
    }
    
    switch_to_multithreaded_mode();
}
}  // namespace dual_arms_mtc


// ============================================================================
// MAIN ENTRY POINT
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 1. Create the node (constructor runs safely)
    auto controller = std::make_shared<dual_arms_mtc::HybridMTCController>();
    
    // 2. Initialize the components (shared_from_this() now works)
    controller->init();
    
    // 3. Spin the node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}