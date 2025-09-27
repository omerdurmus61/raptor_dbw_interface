#include "raptor_dbw_interface/raptor_dbw_interface.hpp"

RaptorDbwInterface::RaptorDbwInterface(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_interface", options)
{
  // Publishers (to Raptor DBW)
  accel_pub_    = this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalCmd>                ("/raptor_dbw_interface/accelerator_pedal_cmd", 10);
  brake_pub_    = this->create_publisher<raptor_dbw_msgs::msg::BrakeCmd>                           ( "/raptor_dbw_interface/brake_cmd", 10);
  steering_pub_ = this->create_publisher<raptor_dbw_msgs::msg::SteeringCmd>                        ("/raptor_dbw_interface/steering_cmd", 10);
  gear_pub_     = this->create_publisher<raptor_dbw_msgs::msg::GearCmd>                            ("/raptor_dbw_interface/gear_cmd", 10);
  enable_pub_   = this->create_publisher<raptor_dbw_msgs::msg::GlobalEnableCmd>                    ("/raptor_dbw_interface/global_enable_cmd", 10);
  misc_pub_     = this->create_publisher<raptor_dbw_msgs::msg::MiscCmd>                            ("/raptor_dbw_interface/misc_cmd", 10);

  // Publishers (to Autoware reports)
  control_mode_pub_     = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>    ("/vehicle/status/control_mode", 10);
  velocity_pub_         = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>       ("/vehicle/status/velocity_status", 10);
  steering_status_pub_  = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>       ("/vehicle/status/steering_status", 10);
  gear_status_pub_      = this->create_publisher<autoware_vehicle_msgs::msg::GearReport>           ("/vehicle/status/gear_status", 10);
  turn_indicators_pub_  = this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport> ("/vehicle/status/turn_indicators_status", 10);
  hazard_lights_pub_    = this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>   ("/vehicle/status/hazard_lights_status", 10);
  actuation_status_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>  ("/vehicle/status/actuation_status", 10);
  
  // Actuatiın status timer  50Hz
  actuation_timer_ = this->create_wall_timer(20ms,std::bind(&RaptorDbwInterface::publishActuationStatusTimerCallback, this));

  // Subscribers (from Autoware)
  ackermann_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd",10,
    std::bind(&RaptorDbwInterface::ackermannCmdCallback, this, std::placeholders::_1));

  // Subscribers (from DBW reports)
  steering_report_sub_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringReport>(
    "/raptor_dbw_interface/steering_report", 10,
    std::bind(&RaptorDbwInterface::steeringReportCallback, this, std::placeholders::_1));

  wheel_speed_report_sub_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
    "/raptor_dbw_interface/wheel_speed_report", 10,
    std::bind(&RaptorDbwInterface::wheelSpeedReportCallback, this, std::placeholders::_1));

  accel_report_sub_ = this->create_subscription<raptor_dbw_msgs::msg::AcceleratorPedalReport>(
    "/raptor_dbw_interface/accelerator_pedal_report", 10,
    std::bind(&RaptorDbwInterface::accelReportCallback, this, std::placeholders::_1));

  brake_report_sub_ = this->create_subscription<raptor_dbw_msgs::msg::BrakeReport>(
    "/raptor_dbw_interface/brake_report", 10,
    std::bind(&RaptorDbwInterface::brakeReportCallback, this, std::placeholders::_1));

  gear_report_sub_ = this->create_subscription<raptor_dbw_msgs::msg::GearReport>(
    "/raptor_dbw_interface/gear_report", 10,
    std::bind(&RaptorDbwInterface::gearReportCallback, this, std::placeholders::_1));

  driver_input_report_sub_ = this->create_subscription<raptor_dbw_msgs::msg::DriverInputReport>(
    "/raptor_dbw_interface/driver_input_report", 10,
    std::bind(&RaptorDbwInterface::driverInputReportCallback, this, std::placeholders::_1));

  misc_report_sub_ = this->create_subscription<raptor_dbw_msgs::msg::MiscReport>(
    "/raptor_dbw_interface/misc_report", 10,
    std::bind(&RaptorDbwInterface::miscReportCallback, this, std::placeholders::_1));


  RCLCPP_INFO(this->get_logger(), "RaptorDbwInterface node initialized.");
}


// === Autoware command → DBW ===
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

// === DBW → Autoware reports ===

void RaptorDbwInterface::publishActuationStatusTimerCallback()
{
  actuation_status_pub_->publish(actuation_status_data_);
}


void RaptorDbwInterface::steeringReportCallback(const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg)
{
  autoware_vehicle_msgs::msg::SteeringReport out;

  out.stamp = this->now();
  actuation_status_data_.header.stamp = this->now();

  out.steering_tire_angle = msg->steering_wheel_angle;  // unit conversion might be required

  actuation_status_data_.status.steer_status = msg->steering_wheel_angle;

  steering_status_pub_->publish(out);

}

void RaptorDbwInterface::wheelSpeedReportCallback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg)
{
  (void)msg;
  // odometry model might be used here
}

void RaptorDbwInterface::accelReportCallback(const raptor_dbw_msgs::msg::AcceleratorPedalReport::SharedPtr msg)
{
  actuation_status_data_.header.stamp = this->now();
  actuation_status_data_.status.accel_status = msg->pedal_output;
}

void RaptorDbwInterface::brakeReportCallback(const raptor_dbw_msgs::msg::BrakeReport::SharedPtr msg)
{
  actuation_status_data_.header.stamp = this->now();
  actuation_status_data_.status.brake_status = msg->pedal_output;
}

void RaptorDbwInterface::gearReportCallback(
  const raptor_dbw_msgs::msg::GearReport::SharedPtr msg)
{
  autoware_vehicle_msgs::msg::GearReport out;
  out.stamp = this->now();

  switch (msg->state.gear) {
    case raptor_dbw_msgs::msg::Gear::DRIVE:
      out.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
      break;
    case raptor_dbw_msgs::msg::Gear::REVERSE:
      out.report = autoware_vehicle_msgs::msg::GearReport::REVERSE;
      break;
    case raptor_dbw_msgs::msg::Gear::NEUTRAL:
      out.report = autoware_vehicle_msgs::msg::GearReport::NEUTRAL;
      break;
    case raptor_dbw_msgs::msg::Gear::PARK:
      out.report = autoware_vehicle_msgs::msg::GearReport::PARK;
      break;
    default:
      out.report = autoware_vehicle_msgs::msg::GearReport::NONE;
      break;
  }

  gear_status_pub_->publish(out);
}


void RaptorDbwInterface::driverInputReportCallback(
  const raptor_dbw_msgs::msg::DriverInputReport::SharedPtr msg)
{
  autoware_vehicle_msgs::msg::TurnIndicatorsReport out;
  autoware_vehicle_msgs::msg::HazardLightsReport out_hazar_lights_report;

  out.stamp = this->now();
  out_hazar_lights_report.stamp = this->now();

  switch (msg->turn_signal.value) {
    case raptor_dbw_msgs::msg::TurnSignal::LEFT:
      out.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
      out_hazar_lights_report.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
      break;
    case raptor_dbw_msgs::msg::TurnSignal::RIGHT:
      out.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
      out_hazar_lights_report.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
      break;
    case raptor_dbw_msgs::msg::TurnSignal::HAZARDS:
      out.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
      out_hazar_lights_report.report = autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE;
      break;
    default:
      out.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
      out_hazar_lights_report.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
      break;
  }

  turn_indicators_pub_->publish(out);
  hazard_lights_pub_->publish(out_hazar_lights_report);
}

void RaptorDbwInterface::miscReportCallback(const raptor_dbw_msgs::msg::MiscReport::SharedPtr msg)
{
  autoware_vehicle_msgs::msg::VelocityReport out;
  autoware_vehicle_msgs::msg::ControlModeReport out_control_mode;

  out.header.stamp = this->now();
  out_control_mode.stamp = this->now();

  // The WheelSpeedReport topic might be used to calculate the average velocity of the wheels
  out.longitudinal_velocity = msg->vehicle_speed / 3.6; // DBW (km/h) -> Autoware (m/s)
  out_control_mode.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;

  velocity_pub_->publish(out);
  control_mode_pub_->publish(out_control_mode);

}