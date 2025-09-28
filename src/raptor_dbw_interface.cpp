#include "raptor_dbw_interface/raptor_dbw_interface.hpp"

RaptorDbwInterface::RaptorDbwInterface(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_interface", options)
{ 

  // Vehicle Parameters
  steering_ratio_ = this->declare_parameter<double>("steering_ratio", 1.0);    // steering_wheel (deg) / tier angle (rad)
  max_decel_      = this->declare_parameter<double>("max_decel", 5.0);         // m/s^2
  max_accel_      = this->declare_parameter<double>("max_accel", 2.0);         // m/s^2
  max_jerk_       = this->declare_parameter<double>("max_jerk", 1.0);          // m/s^3  

  // Override flags
  brake_override_active_    = false;
  accel_override_active_    = false;
  steering_override_active_ = false;
  global_enable_active_     = true;

  //  Initial default values for Autoware command inputs (gear, misc, enable)
  gear_cmd_ = raptor_dbw_msgs::msg::GearCmd();
  gear_cmd_.cmd.gear = raptor_dbw_msgs::msg::Gear::PARK;
  gear_cmd_.enable = false;
  gear_cmd_.rolling_counter = 0;

  misc_merged_cmd_ = raptor_dbw_msgs::msg::MiscCmd();
  misc_merged_cmd_.cmd.value = raptor_dbw_msgs::msg::TurnSignal::NONE;
  misc_merged_cmd_.rolling_counter = 0;

  misc_turn_cmd_ = raptor_dbw_msgs::msg::MiscCmd();
  misc_turn_cmd_.cmd.value = raptor_dbw_msgs::msg::TurnSignal::NONE;
  misc_turn_cmd_.rolling_counter = 0;

  misc_hazard_cmd_ = raptor_dbw_msgs::msg::MiscCmd();
  misc_hazard_cmd_.cmd.value = raptor_dbw_msgs::msg::TurnSignal::NONE;
  misc_hazard_cmd_.rolling_counter = 0;

  enable_cmd_ = raptor_dbw_msgs::msg::GlobalEnableCmd();
  enable_cmd_.global_enable = true;
  enable_cmd_.rolling_counter = 0; 

  counter_ = 0;

  // Publishers (to Raptor DBW)
  accel_pub_    = this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalCmd>                ("/raptor_dbw_interface/accelerator_pedal_cmd", 10);
  brake_pub_    = this->create_publisher<raptor_dbw_msgs::msg::BrakeCmd>                           ("/raptor_dbw_interface/brake_cmd", 10);
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

  // Subscribers (from Autoware additional topics)
  gear_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 10,
    std::bind(&RaptorDbwInterface::gearCmdCallback, this, std::placeholders::_1));

  turn_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "/control/command/turn_indicators_cmd", 10,
    std::bind(&RaptorDbwInterface::turnCmdCallback, this, std::placeholders::_1));

  hazard_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    "/control/command/hazard_lights_cmd", 10,
    std::bind(&RaptorDbwInterface::hazardCmdCallback, this, std::placeholders::_1));

  engage_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/vehicle/engage", 10,
    std::bind(&RaptorDbwInterface::engageCallback, this, std::placeholders::_1));

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
  // Extract data from Autoware control command
  double velocity = msg->longitudinal.velocity;
  double accel = msg->longitudinal.acceleration;
  double steering_tire_angle = msg->lateral.steering_tire_angle;

  // Watchdog counter
  counter_++;
  if (counter_ > 15) {
    counter_ = 0;
  }

  gear_cmd_.rolling_counter        = counter_;
  misc_merged_cmd_.rolling_counter = counter_;
  enable_cmd_.rolling_counter      = counter_;

  // Accelerator command 
  raptor_dbw_msgs::msg::AcceleratorPedalCmd accel_cmd;
  accel_cmd.speed_cmd = velocity;
  accel_cmd.enable = true;
  accel_cmd.control_type.value = raptor_dbw_msgs::msg::ActuatorControlMode::CLOSED_LOOP_VEHICLE;
  accel_cmd.accel_limit                = max_accel_;  
  accel_cmd.accel_positive_jerk_limit  = max_jerk_;  
  accel_cmd.rolling_counter = counter_; 
  accel_pub_->publish(accel_cmd);

  // Brake command
  raptor_dbw_msgs::msg::BrakeCmd brake_cmd;
  brake_cmd.control_type.value = raptor_dbw_msgs::msg::ActuatorControlMode::CLOSED_LOOP_VEHICLE;

  if (accel < 0.0) {
    brake_cmd.pedal_cmd = std::clamp(std::abs(accel) / max_decel_, 0.0, 100.0);
    brake_cmd.enable = true;
  } else {
    brake_cmd.pedal_cmd = 0.0;
    brake_cmd.enable = false;
  }
  brake_cmd.rolling_counter = counter_;
  brake_pub_->publish(brake_cmd);

  // Steering command
  raptor_dbw_msgs::msg::SteeringCmd steer_cmd;
  // Autoware (tire angle) → DBW (steering wheel angle)
  steer_cmd.angle_cmd = steering_tire_angle * steering_ratio_ * 180.0 / M_PI;  
  steer_cmd.enable = true;
  steer_cmd.control_type.value = raptor_dbw_msgs::msg::ActuatorControlMode::CLOSED_LOOP_VEHICLE;
  steer_cmd.rolling_counter = counter_;
  steering_pub_->publish(steer_cmd);

  // Gear command (default DRIVE)
  gear_pub_->publish(gear_cmd_);

  // Turn signals and Hazard Lights
  misc_merged_cmd_.cmd.value =
  (misc_hazard_cmd_.cmd.value == raptor_dbw_msgs::msg::TurnSignal::HAZARDS)
    ? raptor_dbw_msgs::msg::TurnSignal::HAZARDS
    : misc_turn_cmd_.cmd.value;

  misc_pub_->publish(misc_merged_cmd_);

  // Enable command 
  enable_pub_->publish(enable_cmd_);
  //global_enable_active_ = true;

}

// === DBW → Autoware reports ===

void RaptorDbwInterface::publishActuationStatusTimerCallback()
{
  actuation_status_data_.header.stamp = this->now();
  actuation_status_pub_->publish(actuation_status_data_);
}


void RaptorDbwInterface::steeringReportCallback(const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg)
{
  autoware_vehicle_msgs::msg::SteeringReport out;

  out.stamp = this->now();

  // (steering wheel (deg) -> tire angle (rad)) unit conversion might be required
  out.steering_tire_angle = (msg->steering_wheel_angle / steering_ratio_) * M_PI / 180.0;

  actuation_status_data_.status.steer_status = msg->steering_wheel_angle;

  steering_status_pub_->publish(out);

  steering_override_active_ = msg->driver_activity;

}

void RaptorDbwInterface::accelReportCallback(const raptor_dbw_msgs::msg::AcceleratorPedalReport::SharedPtr msg)
{
  actuation_status_data_.status.accel_status = msg->pedal_output;
  accel_override_active_ = msg->driver_activity;
}

void RaptorDbwInterface::brakeReportCallback(const raptor_dbw_msgs::msg::BrakeReport::SharedPtr msg)
{
  actuation_status_data_.status.brake_status = msg->pedal_output;
  brake_override_active_ = msg->driver_activity;

}

void RaptorDbwInterface::wheelSpeedReportCallback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg)
{ 
  (void)msg;
  // TO DO: odometry model might be used here

  /*
  // TO DO: The velocity_status can be calculated based on the velocity of each wheel 
  autoware_vehicle_msgs::msg::VelocityReport out;
  out.header.stamp = this->now();

  // Average Velocity (m/s)
  double avg_speed = (wheel_radius_ * (msg->front_left + msg->front_right +
                                     msg->rear_left + msg->rear_right)) / 4.0;

  out.longitudinal_velocity = avg_speed;

  velocity_pub_->publish(out);
  */
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
  autoware_vehicle_msgs::msg::HazardLightsReport out_hazard_lights_report;

  out.stamp = this->now();
  out_hazard_lights_report.stamp = this->now();

  switch (msg->turn_signal.value) {
    case raptor_dbw_msgs::msg::TurnSignal::LEFT:
      out.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
      out_hazard_lights_report.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
      break;
    case raptor_dbw_msgs::msg::TurnSignal::RIGHT:
      out.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
      out_hazard_lights_report.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
      break;
    case raptor_dbw_msgs::msg::TurnSignal::HAZARDS:
      out.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
      out_hazard_lights_report.report = autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE;
      break;
    default:
      out.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
      out_hazard_lights_report.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
      break;
  }

  turn_indicators_pub_->publish(out);
  hazard_lights_pub_->publish(out_hazard_lights_report);
}

void RaptorDbwInterface::miscReportCallback(const raptor_dbw_msgs::msg::MiscReport::SharedPtr msg)
{ 

  // Velocity Report
  autoware_vehicle_msgs::msg::VelocityReport out;
  out.header.stamp = this->now();

  // The WheelSpeedReport topic might be used to calculate the average velocity of the wheels
  out.longitudinal_velocity = msg->vehicle_speed / 3.6; // DBW (km/h) -> Autoware (m/s)

  velocity_pub_->publish(out);

  // Control Mode Report 
  autoware_vehicle_msgs::msg::ControlModeReport out_control_mode;
  out_control_mode.stamp = this->now();

  bool driver_override = accel_override_active_ ||
                        brake_override_active_  ||
                        steering_override_active_;

  if (global_enable_active_ && !driver_override) {
    out_control_mode.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  } else {
    out_control_mode.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
  }
  control_mode_pub_->publish(out_control_mode);

}

// Additional Control Signals from Autoware
void RaptorDbwInterface::gearCmdCallback(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{

  switch (msg->command) {
    case autoware_vehicle_msgs::msg::GearCommand::DRIVE:
      gear_cmd_.cmd.gear = raptor_dbw_msgs::msg::Gear::DRIVE;
      break;
    case autoware_vehicle_msgs::msg::GearCommand::REVERSE:
      gear_cmd_.cmd.gear = raptor_dbw_msgs::msg::Gear::REVERSE;
      break;
    case autoware_vehicle_msgs::msg::GearCommand::NEUTRAL:
      gear_cmd_.cmd.gear = raptor_dbw_msgs::msg::Gear::NEUTRAL;
      break;
    case autoware_vehicle_msgs::msg::GearCommand::PARK:
      gear_cmd_.cmd.gear = raptor_dbw_msgs::msg::Gear::PARK;
      break;
    default:
      gear_cmd_.enable = false;
      return; 
  }
  gear_cmd_.enable = true;
}

void RaptorDbwInterface::turnCmdCallback(const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg)
{

  switch (msg->command) {
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT:
      misc_turn_cmd_.cmd.value = raptor_dbw_msgs::msg::TurnSignal::LEFT;
      break;
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT:
      misc_turn_cmd_.cmd.value = raptor_dbw_msgs::msg::TurnSignal::RIGHT;
      break;
    default:
      misc_turn_cmd_.cmd.value = raptor_dbw_msgs::msg::TurnSignal::NONE;
      break;
  }
}

void RaptorDbwInterface::hazardCmdCallback(
    const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg)
{
  misc_hazard_cmd_.cmd.value =
      (msg->command == autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE)
          ? raptor_dbw_msgs::msg::TurnSignal::HAZARDS
          : raptor_dbw_msgs::msg::TurnSignal::NONE;
}

void RaptorDbwInterface::engageCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  enable_cmd_.global_enable = msg->data;
  global_enable_active_ = msg->data;
}

