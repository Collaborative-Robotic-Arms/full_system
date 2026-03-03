#include <dual_arms_mtc/zone_detection_manager.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace dual_arms_mtc {

ZoneDetectionManager::ZoneDetectionManager()
    : Node("zone_detection_manager"),
      ar4_current_zone_(ZoneType::SAFE_ZONE),
      abb_current_zone_(ZoneType::SAFE_ZONE) {
    
    RCLCPP_INFO(get_logger(), "Initializing Zone Detection Manager");

    // Initialize TF2 Listener to "see" the simulation
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Load zone configuration from parameters
    declare_parameter("handover_zone.center.x", 0.5);
    declare_parameter("handover_zone.center.y", 0.0);
    declare_parameter("handover_zone.center.z", 0.3);
    declare_parameter("handover_zone.radius_x", 0.2);
    declare_parameter("handover_zone.radius_y", 0.2);
    declare_parameter("handover_zone.radius_z", 0.2);
    declare_parameter("handover_zone.approach_margin", 0.15);
    declare_parameter("handover_zone.min_arm_separation", 0.1);

    zone_config_.center.position.x = get_parameter("handover_zone.center.x").as_double();
    zone_config_.center.position.y = get_parameter("handover_zone.center.y").as_double();
    zone_config_.center.position.z = get_parameter("handover_zone.center.z").as_double();
    zone_config_.handover_radius_x = get_parameter("handover_zone.radius_x").as_double();
    zone_config_.handover_radius_y = get_parameter("handover_zone.radius_y").as_double();
    zone_config_.handover_radius_z = get_parameter("handover_zone.radius_z").as_double();
    zone_config_.approach_margin = get_parameter("handover_zone.approach_margin").as_double();
    zone_config_.min_arm_separation = get_parameter("handover_zone.min_arm_separation").as_double();

    // Create publisher for zone status
    zone_status_pub_ = create_publisher<std_msgs::msg::String>("zone_status", 10);

    // Create diagnostic timer (publish status every 500ms)
    diagnostic_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() { on_diagnostic_timer(); });

    RCLCPP_INFO(get_logger(), 
        "Zone Detection Manager initialized. Handover zone at (%.2f, %.2f, %.2f)",
        zone_config_.center.position.x, zone_config_.center.position.y, zone_config_.center.position.z);
}

ZoneType ZoneDetectionManager::get_zone_type(const geometry_msgs::msg::Pose& pose) {
    double dx = pose.position.x - zone_config_.center.position.x;
    double dy = pose.position.y - zone_config_.center.position.y;
    double dz = pose.position.z - zone_config_.center.position.z;

    // Check if in handover zone
    if (std::abs(dx) <= zone_config_.handover_radius_x &&
        std::abs(dy) <= zone_config_.handover_radius_y &&
        std::abs(dz) <= zone_config_.handover_radius_z) {
        return ZoneType::HANDOVER_ZONE;
    }

    // Check if in approach zone (within margin of handover zone)
    double margin = zone_config_.approach_margin;
    if (std::abs(dx) <= (zone_config_.handover_radius_x + margin) &&
        std::abs(dy) <= (zone_config_.handover_radius_y + margin) &&
        std::abs(dz) <= (zone_config_.handover_radius_z + margin)) {
        return ZoneType::APPROACH_ZONE;
    }

    return ZoneType::SAFE_ZONE;
}

bool ZoneDetectionManager::is_in_handover_zone(const geometry_msgs::msg::Pose& pose) {
    return get_zone_type(pose) == ZoneType::HANDOVER_ZONE;
}

bool ZoneDetectionManager::is_approaching_handover(const geometry_msgs::msg::Pose& pose) {
    ZoneType zone = get_zone_type(pose);
    return zone == ZoneType::APPROACH_ZONE || zone == ZoneType::HANDOVER_ZONE;
}

double ZoneDetectionManager::get_distance_to_handover(const geometry_msgs::msg::Pose& pose) {
    double dx = pose.position.x - zone_config_.center.position.x;
    double dy = pose.position.y - zone_config_.center.position.y;
    double dz = pose.position.z - zone_config_.center.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool ZoneDetectionManager::are_both_arms_ready_for_handover(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose) {
    
    ZoneType ar4_zone = get_zone_type(ar4_pose);
    ZoneType abb_zone = get_zone_type(abb_pose);

    if ((ar4_zone != ZoneType::HANDOVER_ZONE && ar4_zone != ZoneType::APPROACH_ZONE) ||
        (abb_zone != ZoneType::HANDOVER_ZONE && abb_zone != ZoneType::APPROACH_ZONE)) {
        return false;
    }

    double separation = calculate_distance(ar4_pose, abb_pose);
    if (separation < zone_config_.min_arm_separation) {
        RCLCPP_WARN(get_logger(), "Arms too close! Separation: %.3f m (min: %.3f m)",
            separation, zone_config_.min_arm_separation);
        return false;
    }
    return true;
}

bool ZoneDetectionManager::check_collision_risk(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose) {
    
    double separation = calculate_distance(ar4_pose, abb_pose);
    double collision_threshold = zone_config_.min_arm_separation * 0.5;

    if (separation < collision_threshold) {
        RCLCPP_ERROR(get_logger(), "COLLISION RISK! Separation: %.3f m", separation);
        return true;
    }
    return false;
}

void ZoneDetectionManager::set_handover_zone_center(const geometry_msgs::msg::Pose& center) {
    zone_config_.center = center;
    RCLCPP_INFO(get_logger(), "Handover zone center updated to (%.2f, %.2f, %.2f)",
        center.position.x, center.position.y, center.position.z);
}

void ZoneDetectionManager::set_handover_zone_radius(double radius_x, double radius_y, double radius_z) {
    zone_config_.handover_radius_x = radius_x;
    zone_config_.handover_radius_y = radius_y;
    zone_config_.handover_radius_z = radius_z;
}

void ZoneDetectionManager::set_approach_zone_margin(double margin) {
    zone_config_.approach_margin = margin;
}

void ZoneDetectionManager::register_zone_transition_callback(ZoneTransitionCallback callback) {
    transition_callbacks_.push_back(callback);
}

double ZoneDetectionManager::calculate_distance(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2) {
    
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void ZoneDetectionManager::publish_zone_status(const std::string& status) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = status;
    zone_status_pub_->publish(std::move(msg));
}

void ZoneDetectionManager::on_diagnostic_timer() {
    // 1. Fetch live poses from the simulation
    try {
        // AR4 live tracking
        auto t_ar4 = tf_buffer_->lookupTransform("world", "ar4_ee_link", tf2::TimePointZero);
        geometry_msgs::msg::Pose ar4_pose;
        ar4_pose.position.x = t_ar4.transform.translation.x;
        ar4_pose.position.y = t_ar4.transform.translation.y;
        ar4_pose.position.z = t_ar4.transform.translation.z;
        ar4_current_zone_.store(get_zone_type(ar4_pose));

        // ABB live tracking
        auto t_abb = tf_buffer_->lookupTransform("world", "tool0", tf2::TimePointZero);
        geometry_msgs::msg::Pose abb_pose;
        abb_pose.position.x = t_abb.transform.translation.x;
        abb_pose.position.y = t_abb.transform.translation.y;
        abb_pose.position.z = t_abb.transform.translation.z;
        abb_current_zone_.store(get_zone_type(abb_pose));

    } catch (const tf2::TransformException & ex) {
        // Silently fail if simulation isn't running yet to avoid log spam
    }

    // 2. Convert Zone ENUMS to string
    ZoneType ar4_zone = ar4_current_zone_.load();
    ZoneType abb_zone = abb_current_zone_.load();

    std::string ar4_zone_str = (ar4_zone == ZoneType::SAFE_ZONE) ? "SAFE" :
                               (ar4_zone == ZoneType::APPROACH_ZONE) ? "APPROACH" : "HANDOVER";
    std::string abb_zone_str = (abb_zone == ZoneType::SAFE_ZONE) ? "SAFE" :
                               (abb_zone == ZoneType::APPROACH_ZONE) ? "APPROACH" : "HANDOVER";

    // 3. Publish the live status
    publish_zone_status("AR4: " + ar4_zone_str + " | ABB: " + abb_zone_str);
}

void ZoneDetectionManager::notify_zone_transition(const ZoneTransition& transition) {
    for (auto& callback : transition_callbacks_) {
        callback(transition);
    }
}

OperationZone ZoneDetectionManager::get_operation_zone(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose) {
    
    double separation = calculate_distance(ar4_pose, abb_pose);
    ZoneType ar4_zone = get_zone_type(ar4_pose);
    ZoneType abb_zone = get_zone_type(abb_pose);

    if (check_collision_risk(ar4_pose, abb_pose)) {
        RCLCPP_ERROR(get_logger(), "⚠️  COLLISION RISK ZONE");
        return OperationZone::COLLISION_RISK_ZONE;
    }

    bool ar4_in_handover = (ar4_zone == ZoneType::HANDOVER_ZONE || ar4_zone == ZoneType::APPROACH_ZONE);
    bool abb_in_handover = (abb_zone == ZoneType::HANDOVER_ZONE || abb_zone == ZoneType::APPROACH_ZONE);

    if (ar4_in_handover || abb_in_handover) {
        return OperationZone::HANDOVER_AREA;
    }

    if (separation >= zone_config_.parallel_safe_separation) {
        if (ar4_zone == ZoneType::SAFE_ZONE) return OperationZone::AR4_SAFE_ZONE;
        if (abb_zone == ZoneType::SAFE_ZONE) return OperationZone::ABB_SAFE_ZONE;
    }

    return OperationZone::AR4_SAFE_ZONE;
}

bool ZoneDetectionManager::can_operate_in_parallel(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose) {
    
    double separation = calculate_distance(ar4_pose, abb_pose);
    ZoneType ar4_zone = get_zone_type(ar4_pose);
    ZoneType abb_zone = get_zone_type(abb_pose);

    if (ar4_zone != ZoneType::SAFE_ZONE || abb_zone != ZoneType::SAFE_ZONE) return false;
    if (separation < zone_config_.parallel_safe_separation) return false;
    if (check_collision_risk(ar4_pose, abb_pose)) return false;

    return true;
}

bool ZoneDetectionManager::should_use_sequential_handover(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose) {
    
    ZoneType ar4_zone = get_zone_type(ar4_pose);
    ZoneType abb_zone = get_zone_type(abb_pose);

    bool ar4_in_handover = (ar4_zone == ZoneType::HANDOVER_ZONE || ar4_zone == ZoneType::APPROACH_ZONE);
    bool abb_in_handover = (abb_zone == ZoneType::HANDOVER_ZONE || abb_zone == ZoneType::APPROACH_ZONE);

    if (!ar4_in_handover && !abb_in_handover) return false;
    if (!are_both_arms_ready_for_handover(ar4_pose, abb_pose)) return false;

    return true;
}

double ZoneDetectionManager::get_required_parallel_separation() const {
    return zone_config_.parallel_safe_separation;
}

OperationType ZoneDetectionManager::determine_required_operation_type(
    const geometry_msgs::msg::Pose& ar4_pose,
    const geometry_msgs::msg::Pose& abb_pose) {
    
    if (should_use_sequential_handover(ar4_pose, abb_pose)) {
        return OperationType::HANDOVER;
    }
    
    if (can_operate_in_parallel(ar4_pose, abb_pose)) {
        return OperationType::PICK_PLACE;
    }

    return OperationType::SYNCHRONIZED;
}

}  // namespace dual_arms_mtc

// Cleanly placed main function outside the namespace
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dual_arms_mtc::ZoneDetectionManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}