#ifndef RAPTOR_DBW_INTERFACE__RAPTOR_DBW_INTERFACE_HPP_
#define RAPTOR_DBW_INTERFACE__RAPTOR_DBW_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp"
#include "raptor_dbw_msgs/msg/brake_cmd.hpp"
#include "raptor_dbw_msgs/msg/steering_cmd.hpp"
#include "raptor_dbw_msgs/msg/gear_cmd.hpp"
#include "raptor_dbw_msgs/msg/global_enable_cmd.hpp"
#include "raptor_dbw_msgs/msg/misc_cmd.hpp"

class RaptorDbwInterface : public rclcpp::Node
{
public:
  explicit RaptorDbwInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void ackermannCmdCallback(const autoware_control_msgs::msg::Control::SharedPtr msg);

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr ackermann_sub_;

  rclcpp::Publisher<raptor_dbw_msgs::msg::AcceleratorPedalCmd>::SharedPtr accel_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::BrakeCmd>::SharedPtr brake_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringCmd>::SharedPtr steering_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::GearCmd>::SharedPtr gear_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::GlobalEnableCmd>::SharedPtr enable_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::MiscCmd>::SharedPtr misc_pub_;

  // Rolling Counter
  uint8_t counter_;
};

#endif  // RAPTOR_DBW_INTERFACE__RAPTOR_DBW_INTERFACE_HPP_
