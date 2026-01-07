#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "abb_robot_msgs/srv/set_io_signal.hpp"

using namespace std::chrono_literals;

class EndEffectorClient : public rclcpp::Node
{
public:
  EndEffectorClient()
  : Node("end_effector_clienttesttt"), toggle_(false)
  {
    client_ = this->create_client<abb_robot_msgs::srv::SetIOSignal>("/rws_client/set_gripper_state");

    timer_ = this->create_wall_timer(
      5s, std::bind(&EndEffectorClient::send_request, this));
  }

private:
  void send_request()
  {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for service...");
      return;
    }

    auto request = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
    request->signal = "do_gripper";  // Replace with your actual IO signal name
    request->value = toggle_ ? "1" : "0";

    RCLCPP_INFO(this->get_logger(), "Sending signalAAAAAAAAAAAAAAAAA: %s = %s",
                request->signal.c_str(), request->value.c_str());

    auto result_future = client_->async_send_request(request);

    // Wait up to 1s for response
    
    auto response = result_future.get();
    if (response->result_code == 0)
      {
        RCLCPP_INFO(this->get_logger(), "Signal set successfully.BBBBBBBBBBBBBBBBBBBBBBBBBBBB");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to set signal. MessageCCCCCCCCCCCCCCCCCCCCCCCCCC: %s",
                     response->message.c_str());
      }
    
   

    toggle_ = !toggle_;  // Flip between 1 and 0
  }

  rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool toggle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EndEffectorClient>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  // rclcpp::spin(std::make_shared<EndEffectorClient>());
  executor.spin();
  rclcpp::shutdown();
  return 0;

}
