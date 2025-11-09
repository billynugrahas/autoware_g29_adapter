#include <rclcpp/rclcpp.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <ros_g29_force_feedback/msg/force_feedback.hpp>
#include <cmath>
#include <algorithm>

namespace autoware_g29_adapter
{

class AutowareG29AdapterNode : public rclcpp::Node
{
public:
  AutowareG29AdapterNode() : Node("autoware_g29_adapter")
  {
    RCLCPP_INFO(this->get_logger(), "Autoware G29 Adapter Node has been started.");
    
    // Declare and get parameters
    this->declare_parameter("steering_ratio", 15.0);
    this->declare_parameter("max_steering_angle_deg", 300.0);
    this->declare_parameter("default_torque", 0.5);
    this->declare_parameter("invert_steering_direction", true);
    
    steering_ratio_ = this->get_parameter("steering_ratio").as_double();
    max_steering_angle_deg_ = this->get_parameter("max_steering_angle_deg").as_double();
    default_torque_ = this->get_parameter("default_torque").as_double();
    invert_steering_direction_ = this->get_parameter("invert_steering_direction").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  - steering_ratio: %.2f", steering_ratio_);
    RCLCPP_INFO(this->get_logger(), "  - max_steering_angle_deg: %.2f", max_steering_angle_deg_);
    RCLCPP_INFO(this->get_logger(), "  - default_torque: %.2f", default_torque_);
    RCLCPP_INFO(this->get_logger(), "  - invert_steering_direction: %s", invert_steering_direction_ ? "true" : "false");
    
    // Create subscriber for Autoware steering commands with compatible QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    steering_sub_ = this->create_subscription<autoware_adapi_v1_msgs::msg::SteeringCommand>(
      "/api/control/command/steering",
      qos,
      std::bind(&AutowareG29AdapterNode::steeringCommandCallback, this, std::placeholders::_1));
    
    // Create publisher for G29 force feedback
    ff_pub_ = this->create_publisher<ros_g29_force_feedback::msg::ForceFeedback>(
      "/ff_target",
      10);
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to: /api/control/command/steering");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /ff_target");
  }

private:
  void steeringCommandCallback(const autoware_adapi_v1_msgs::msg::SteeringCommand::SharedPtr msg)
  {
    // Convert tire angle from radians to degrees
    double tire_angle_deg = msg->steering_tire_angle * (180.0 / M_PI);
    
    // Convert tire angle to wheel angle using steering ratio
    double wheel_angle_deg = tire_angle_deg * steering_ratio_;
    
    // Apply steering direction inversion if needed
    // Autoware convention: negative = turn right (CW)
    // G29 convention: negative = turn left (CCW)
    // Inversion compensates for this difference
    if (invert_steering_direction_) {
      wheel_angle_deg = -wheel_angle_deg;
    }
    
    // Clamp wheel angle to maximum steering angle
    wheel_angle_deg = std::clamp(wheel_angle_deg, -max_steering_angle_deg_, max_steering_angle_deg_);
    
    // Normalize to G29 position range (-1.0 to 1.0)
    // G29 has range of ±450 degrees
    double normalized_position = wheel_angle_deg / 450.0;
    
    // Create and publish force feedback message
    auto ff_msg = ros_g29_force_feedback::msg::ForceFeedback();
    ff_msg.header.stamp = this->now();
    ff_msg.position = static_cast<float>(normalized_position);
    ff_msg.torque = static_cast<float>(default_torque_);
    
    ff_pub_->publish(ff_msg);
    
    RCLCPP_DEBUG(this->get_logger(),
      "Steering: tire_angle=%.2f rad (%.2f deg), wheel_angle=%.2f deg, position=%.3f, torque=%.2f",
      msg->steering_tire_angle, tire_angle_deg, wheel_angle_deg, normalized_position, default_torque_);
  }

  // Parameters
  double steering_ratio_;
  double max_steering_angle_deg_;
  double default_torque_;
  bool invert_steering_direction_;
  
  // ROS2 interfaces
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::SteeringCommand>::SharedPtr steering_sub_;
  rclcpp::Publisher<ros_g29_force_feedback::msg::ForceFeedback>::SharedPtr ff_pub_;
};

}  // namespace autoware_g29_adapter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware_g29_adapter::AutowareG29AdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}