#include "raptor_dbw_interface/raptor_dbw_interface.hpp"

RaptorDbwInterface::RaptorDbwInterface(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_interface", options)
{
  // Publishers
  accel_pub_ = this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalCmd>("/raptor_dbw_interface/accelerator_pedal_cmd", 10);
  brake_pub_ = this->create_publisher<raptor_dbw_msgs::msg::BrakeCmd>( "/raptor_dbw_interface/brake_cmd", 10);
  steering_pub_ = this->create_publisher<raptor_dbw_msgs::msg::SteeringCmd>("/raptor_dbw_interface/steering_cmd", 10);
  gear_pub_ = this->create_publisher<raptor_dbw_msgs::msg::GearCmd>("/raptor_dbw_interface/gear_cmd", 10);
  enable_pub_ = this->create_publisher<raptor_dbw_msgs::msg::GlobalEnableCmd>("/raptor_dbw_interface/global_enable_cmd", 10);
  misc_pub_ = this->create_publisher<raptor_dbw_msgs::msg::MiscCmd>("/raptor_dbw_interface/misc_cmd", 10);

  // Subscriber
  ackermann_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd",
    10,
    std::bind(&RaptorDbwInterface::ackermannCmdCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "RaptorDbwInterface node initialized.");
}

void RaptorDbwInterface::ackermannCmdCallback(const autoware_control_msgs::msg::Control::SharedPtr msg)

{
  // Extract data from Autoware command
  double speed = msg->longitudinal.velocity;
  double accel = msg->longitudinal.acceleration;
  double steering_angle = msg->lateral.steering_tire_angle;

  // watchdog counter
  counter_++;
  if (counter_ > 15) {
    counter_ = 0;
  }

  // Accelerator command 
  raptor_dbw_msgs::msg::AcceleratorPedalCmd accel_cmd;
  //accel_cmd.pedal_cmd = (accel > 0.0) ? std::min(accel, 1.0) : 0.0;  // [0-1] 
  accel_cmd.speed_cmd = speed;
  accel_cmd.enable = true;
  accel_cmd.rolling_counter = counter_;
  accel_pub_->publish(accel_cmd);

  // Brake command
  raptor_dbw_msgs::msg::BrakeCmd brake_cmd;
  if (accel < 0.0) {
    brake_cmd.pedal_cmd = std::min(std::abs(accel), 1.0);
    brake_cmd.enable = true;
  } else {
    brake_cmd.pedal_cmd = 0.0;
    brake_cmd.enable = false;
  }
  brake_cmd.rolling_counter = counter_;
  brake_pub_->publish(brake_cmd);

  // Steering command
  raptor_dbw_msgs::msg::SteeringCmd steer_cmd;
  steer_cmd.angle_cmd = steering_angle;  // radian
  steer_cmd.enable = true;
  steer_cmd.rolling_counter = counter_;
  steering_pub_->publish(steer_cmd);

  // Gear command (default DRIVE)
  raptor_dbw_msgs::msg::GearCmd gear_cmd;
  gear_cmd.cmd.gear = raptor_dbw_msgs::msg::Gear::DRIVE;
  gear_cmd.rolling_counter = counter_;
  gear_pub_->publish(gear_cmd);

  // Turn signal
  raptor_dbw_msgs::msg::MiscCmd misc_msg;
  misc_msg.rolling_counter = counter_;
  misc_pub_->publish(misc_msg);

  // Enable command 
  raptor_dbw_msgs::msg::GlobalEnableCmd enable_cmd;
  enable_cmd.global_enable = true;
  enable_cmd.rolling_counter = counter_;
  enable_pub_->publish(enable_cmd);
}
