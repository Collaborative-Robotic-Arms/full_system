#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread> // Added for std::thread
#include <chrono> // Added for std::chrono

// Logger specific to IRB120
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_to_pose_irb120");

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    RCLCPP_INFO(LOGGER, "ROS 2 initialized");

    // Create node with options - using IRB120 node name
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_to_pose_node_irb120", node_options);
    move_group_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    RCLCPP_INFO(LOGGER, "Node created with name: move_to_pose_node_irb120");

    // Spin the node in a separate thread (AR4 structure)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    RCLCPP_INFO(LOGGER, "Node spinning in background thread");

    // Define planning group - Hardcoded like AR4, but using IRB120 group name
    static const std::string PLANNING_GROUP = "irb120_arm";
    RCLCPP_INFO(LOGGER, "Planning group set to: %s", PLANNING_GROUP.c_str());

    // Initialize MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    RCLCPP_INFO(LOGGER, "MoveGroupInterface initialized for group: %s", PLANNING_GROUP.c_str());

    // Log basic information
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Prepare pose holder and flag
    geometry_msgs::msg::Pose target_pose;
    bool pose_received = false;

    // Subscription to target pose topic - Hardcoded like AR4, but using IRB120 topic name
    static const std::string TARGET_POSE_TOPIC = "/target_pose_irb120";
    auto subscription = move_group_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        TARGET_POSE_TOPIC, 10,
        [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            target_pose = msg->pose;
            pose_received = true;
            // Using detailed logging from original ABB code
            RCLCPP_INFO(LOGGER, "Received target pose: position(x=%f, y=%f, z=%f), orientation(w=%f, x=%f, y=%f, z=%f)",
                        target_pose.position.x, target_pose.position.y, target_pose.position.z,
                        target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
        });

    // Wait for the pose (Single shot like AR4)
    RCLCPP_INFO(LOGGER, "Waiting for target pose on %s...", TARGET_POSE_TOPIC.c_str());
    while (rclcpp::ok() && !pose_received) {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // Check if ROS shutdown was requested while waiting
    if (!rclcpp::ok()) {
      RCLCPP_INFO(LOGGER, "ROS shutdown requested before pose received.");
      return 0;
    }

    RCLCPP_INFO(LOGGER, "Target pose received, proceeding with motion planning");

    // Set pose target
    move_group.setPoseTarget(target_pose);
    RCLCPP_INFO(LOGGER, "Pose target assigned to MoveGroup");

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(LOGGER, "Starting motion planning...");
    // Using planning parameters from original ABB code
    move_group.setMaxVelocityScalingFactor(0.7);
    move_group.setMaxAccelerationScalingFactor(0.7);

    bool planning_success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (planning_success)
    {
        RCLCPP_INFO(LOGGER, "Planning succeeded!");
        RCLCPP_INFO(LOGGER, "Planning time: %f seconds", my_plan.planning_time);
        size_t num_points = my_plan.trajectory.joint_trajectory.points.size();
        RCLCPP_INFO(LOGGER, "Trajectory contains %zu points", num_points);
        // Optional: Keep detailed point logging if needed (like original ABB)
        // for (size_t i = 0; i < num_points; ++i) {
        //   RCLCPP_DEBUG(LOGGER, "Point %zu time_from_start: %f", i,
        //                my_plan.trajectory.joint_trajectory.points[i].time_from_start.sec +
        //                my_plan.trajectory.joint_trajectory.points[i].time_from_start.nanosec * 1e-9);
        // }

        // Execute the motion
        RCLCPP_INFO(LOGGER, "Attempting to execute the planned trajectory...");
        moveit::core::MoveItErrorCode execution_result = move_group.execute(my_plan);

        if (execution_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(LOGGER, "Execution succeeded! Robot should have moved.");
            // Log current pose after execution (like AR4 structure), using detailed format
            geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
            RCLCPP_INFO(LOGGER, "New current pose: position(x=%f, y=%f, z=%f), orientation(w=%f, x=%f, y=%f, z=%f)",
                        current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                        current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);
        }
        else
        {
            // Using simple execution failure log like original ABB
            RCLCPP_ERROR(LOGGER, "Execution failed!");
            // Note: Following AR4 structure, we don't exit here, we proceed to shutdown.
        }
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
        // Exiting on planning failure (like AR4 structure)
        rclcpp::shutdown();
        return 1;
    }

    // Wait briefly before shutdown (like AR4 structure)
    RCLCPP_INFO(LOGGER, "Task complete. Shutting down soon...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(LOGGER, "Shutting down...");

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
