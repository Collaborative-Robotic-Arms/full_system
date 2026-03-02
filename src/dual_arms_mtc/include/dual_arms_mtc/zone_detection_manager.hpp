#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <dual_arms_msgs/srv/get_handover_zone.hpp>
#include <dual_arms_mtc/control_strategy.hpp>
#include <memory>
#include <atomic>
#include <vector>
#include <cmath>

namespace dual_arms_mtc {

enum class ZoneType {
    SAFE_ZONE,      // Far from handover zone
    APPROACH_ZONE,  // Entering handover vicinity
    HANDOVER_ZONE,  // Critical handover area
    RETRACT_ZONE    // Leaving handover area
};

/**
 * @brief Defines operation zones for parallel vs sequential control
 */
enum class OperationZone {
    AR4_SAFE_ZONE,        // AR4 can pick/place independently
    ABB_SAFE_ZONE,        // ABB can pick/place independently
    HANDOVER_AREA,        // Sequential handover zone
    COLLISION_RISK_ZONE   // Too close - potential collision
};

struct ZoneTransition {
    ZoneType from_zone;
    ZoneType to_zone;
    int timestamp_ms;
};

/**
 * @brief Zone Detection Manager for hybrid MTC control
 * Monitors robot poses and detects when they enter/exit handover zones
 * Publishes state transitions for controller switching
 */
class ZoneDetectionManager : public rclcpp::Node {
public:
    ZoneDetectionManager();

    // Zone queries
    ZoneType get_zone_type(const geometry_msgs::msg::Pose& pose);
    bool is_in_handover_zone(const geometry_msgs::msg::Pose& pose);
    bool is_approaching_handover(const geometry_msgs::msg::Pose& pose);
    double get_distance_to_handover(const geometry_msgs::msg::Pose& pose);

    // Dual arm zone checking
    bool are_both_arms_ready_for_handover(
        const geometry_msgs::msg::Pose& ar4_pose,
        const geometry_msgs::msg::Pose& abb_pose);

    bool check_collision_risk(
        const geometry_msgs::msg::Pose& ar4_pose,
        const geometry_msgs::msg::Pose& abb_pose);
    
    // ========================================================================
    // OPERATION-SPECIFIC ZONE CHECKING - NEW FOR PARALLEL/SEQUENTIAL CONTROL
    // ========================================================================
    
    /**
     * @brief Determine operation zone for dual arm coordination
     * Returns whether arms are in safe zones, handover area, or collision risk
     */
    OperationZone get_operation_zone(
        const geometry_msgs::msg::Pose& ar4_pose,
        const geometry_msgs::msg::Pose& abb_pose);
    
    /**
     * @brief Check if both arms can safely pick/place in parallel
     * Returns true if arms are far enough apart to operate independently
     */
    bool can_operate_in_parallel(
        const geometry_msgs::msg::Pose& ar4_pose,
        const geometry_msgs::msg::Pose& abb_pose);
    
    /**
     * @brief Check if arms are in sequential handover mode
     * Returns true if at least one arm is in handover zone and both are ready
     */
    bool should_use_sequential_handover(
        const geometry_msgs::msg::Pose& ar4_pose,
        const geometry_msgs::msg::Pose& abb_pose);
    
    /**
     * @brief Calculate minimum safe separation for parallel operations
     * @return Required separation distance in meters
     */
    double get_required_parallel_separation() const;
    
    /**
     * @brief Determine which control mode should be active based on arm positions
     */
    OperationType determine_required_operation_type(
        const geometry_msgs::msg::Pose& ar4_pose,
        const geometry_msgs::msg::Pose& abb_pose);

    // Zone configuration
    void set_handover_zone_center(const geometry_msgs::msg::Pose& center);
    void set_handover_zone_radius(double radius_x, double radius_y, double radius_z);
    void set_approach_zone_margin(double margin);

    // Callbacks for state transitions
    using ZoneTransitionCallback = std::function<void(const ZoneTransition&)>;
    void register_zone_transition_callback(ZoneTransitionCallback callback);

private:
    struct ZoneConfig {
        geometry_msgs::msg::Pose center;
        double handover_radius_x = 0.2;
        double handover_radius_y = 0.2;
        double handover_radius_z = 0.2;
        double approach_margin = 0.15;  // Approach zone is 15cm outside handover zone
        double min_arm_separation = 0.1; // Minimum distance between arm TCP points
        double parallel_safe_separation = 0.3;  // Safe separation for parallel operations
    } zone_config_;

    // State tracking
    std::atomic<ZoneType> ar4_current_zone_;
    std::atomic<ZoneType> abb_current_zone_;
    std::vector<ZoneTransitionCallback> transition_callbacks_;

    // Publishers for diagnostics
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zone_status_pub_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    // Helper methods
    double calculate_distance(const geometry_msgs::msg::Pose& pose1, 
                             const geometry_msgs::msg::Pose& pose2);
    void publish_zone_status(const std::string& status);
    void on_diagnostic_timer();
    void notify_zone_transition(const ZoneTransition& transition);
};

}  // namespace dual_arms_mtc
