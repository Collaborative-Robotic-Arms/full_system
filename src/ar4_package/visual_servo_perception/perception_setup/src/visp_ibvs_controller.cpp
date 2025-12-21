#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> 
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp> 
#include <sensor_msgs/msg/camera_info.hpp> 

#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColVector.h>

#include "supervisor_package/action/align_to_target.hpp" 

#define NODE_NAME "visp_ibvs_controller"
#define NUM_FEATURES 3 

class VispIBVSController : public rclcpp::Node
{
public:
    using AlignToTarget = supervisor_package::action::AlignToTarget;
    using GoalHandleAlign = rclcpp_action::ServerGoalHandle<AlignToTarget>;

    VispIBVSController() : Node(NODE_NAME)
    {
        // Initialize basic variables
        cam_initialized_ = false;
        is_active_ = false; 
        error_norm_ = 100.0;
        cam_frame_id_ = "ar4_camera_link"; 
        v_prev_ = vpColVector(6, 0.0);
        alpha_ = 0.1;
        current_depth_z_ = 0.5; // Default safety depth

        // IBVS Setup
        servo_.setLambda(0.2); 
        servo_.setInteractionMatrixType(vpServo::CURRENT);
        servo_.setServo(vpServo::EYEINHAND_CAMERA);

        // Subscriptions
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "cameraAR4/camera_info", 10,
            std::bind(&VispIBVSController::infoCallback, this, std::placeholders::_1));

        feature_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/feature_coordinates_6D", 10,
            std::bind(&VispIBVSController::featureCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/camera_to_marker_depth", 10,
            std::bind(&VispIBVSController::depthCallback, this, std::placeholders::_1));

        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        // Action Server
        this->action_server_ = rclcpp_action::create_server<AlignToTarget>(
            this, "ar4_visual_servo",
            std::bind(&VispIBVSController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&VispIBVSController::handle_cancel, this, std::placeholders::_1),
            std::bind(&VispIBVSController::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ViSP IBVS Action Server started.");
    }

private:
    // --- Members (These were missing!) ---
    rclcpp_action::Server<AlignToTarget>::SharedPtr action_server_;
    
    // ROS Communication
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr feature_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

    // ViSP Objects
    vpServo servo_;
    vpCameraParameters cam_;
    vpFeaturePoint s_curr_[NUM_FEATURES], s_des_[NUM_FEATURES];
    vpColVector v_prev_;

    // State Variables
    bool cam_initialized_;
    bool is_active_;
    double error_norm_;
    double current_depth_z_;
    double alpha_;
    std::string cam_frame_id_;

    // --- Callbacks ---
    void featureCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!cam_initialized_ || !is_active_) return;
        if (msg->data.size() != NUM_FEATURES * 2) return;

        for (int i = 0; i < NUM_FEATURES; ++i) {
            double x_norm = (msg->data[i*2] - cam_.get_u0()) / cam_.get_px();
            double y_norm = (msg->data[i*2+1] - cam_.get_v0()) / cam_.get_py();
            s_curr_[i].set_x(x_norm); 
            s_curr_[i].set_y(y_norm); 
            s_curr_[i].set_Z(current_depth_z_); 
        }

        vpColVector v_opt(6, 0.0);
        try {
            v_opt = servo_.computeControlLaw();
            error_norm_ = servo_.getError().sumSquare(); 
        } catch (...) { v_opt = 0.0; }

        process_and_publish_velocity(v_opt);
    }

    void process_and_publish_velocity(vpColVector v_opt) {
        auto velocity_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        velocity_msg->header.stamp = this->get_clock()->now(); 
        velocity_msg->header.frame_id = cam_frame_id_; 

        velocity_msg->twist.linear.x = v_opt[2]; 
        velocity_msg->twist.linear.y = -v_opt[0]; 
        velocity_msg->twist.linear.z = -v_opt[1]; 
        velocity_pub_->publish(*velocity_msg);
    }

    void stop_robot() {
        auto stop_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        stop_msg->header.frame_id = cam_frame_id_;
        velocity_pub_->publish(*stop_msg);
    }

    // Action Logic
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const AlignToTarget::Goal>) 
    { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAlign>) 
    { is_active_ = false; stop_robot(); return rclcpp_action::CancelResponse::ACCEPT; }

    void handle_accepted(const std::shared_ptr<GoalHandleAlign> goal_handle) 
    { std::thread{std::bind(&VispIBVSController::execute, this, goal_handle)}.detach(); }

    void execute(const std::shared_ptr<GoalHandleAlign> goal_handle) {
        is_active_ = true;
        auto result = std::make_shared<AlignToTarget::Result>();
        rclcpp::Rate loop_rate(10);
        while (rclcpp::ok() && error_norm_ > 1e-3) {
            if (goal_handle->is_canceling()) {
                is_active_ = false; stop_robot();
                goal_handle->canceled(result); return;
            }
            loop_rate.sleep();
        }
        is_active_ = false; stop_robot();
        result->success = true;
        goal_handle->succeed(result);
    }

    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        cam_.initPersProjWithoutDistortion(msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
        cam_initialized_ = true;
    }
    
    void depthCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_depth_z_ = msg->data;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VispIBVSController>());
    rclcpp::shutdown();
    return 0;
}