#ifndef RAPTOR_DBW_INTERFACE__RAPTOR_DBW_INTERFACE_HPP_
#define RAPTOR_DBW_INTERFACE__RAPTOR_DBW_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>

// Autoware Command messages
#include "autoware_control_msgs/msg/control.hpp"
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>

// Autoware Report messages
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"

// Raptor DBW command messages
#include "raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp"
#include "raptor_dbw_msgs/msg/brake_cmd.hpp"
#include "raptor_dbw_msgs/msg/steering_cmd.hpp"
#include "raptor_dbw_msgs/msg/gear_cmd.hpp"
#include "raptor_dbw_msgs/msg/global_enable_cmd.hpp"
#include "raptor_dbw_msgs/msg/misc_cmd.hpp"

// Raptor DBW Report messages
#include "raptor_dbw_msgs/msg/steering_report.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "raptor_dbw_msgs/msg/accelerator_pedal_report.hpp"
#include "raptor_dbw_msgs/msg/brake_report.hpp"
#include "raptor_dbw_msgs/msg/gear_report.hpp"
#include "raptor_dbw_msgs/msg/driver_input_report.hpp"
#include "raptor_dbw_msgs/msg/misc_report.hpp"

using namespace std::chrono_literals;

class RaptorDbwInterface : public rclcpp::Node
{
public:
  explicit RaptorDbwInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  // Callback functions for vehicle control 
  void ackermannCmdCallback (const autoware_control_msgs::msg::Control::SharedPtr msg);
  void gearCmdCallback      (const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg);
  void turnCmdCallback      (const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg);
  void hazardCmdCallback    (const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg);
  void engageCallback       (const std_msgs::msg::Bool::SharedPtr msg);

  // Actuation status timer 
  tier4_vehicle_msgs::msg::ActuationStatusStamped   actuation_status_data_;
  rclcpp::TimerBase::SharedPtr                      actuation_timer_;

  // Subscribers (from Autoware)
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr               ackermann_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr           gear_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr   hazard_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                               engage_sub_;


  // Publishers (to Autoware reports)
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr    control_mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr       velocity_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr       steering_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr           gear_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr   hazard_lights_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr  actuation_status_pub_;
  
  // Publishers (to Raptor DBW)
  rclcpp::Publisher<raptor_dbw_msgs::msg::AcceleratorPedalCmd>::SharedPtr        accel_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::BrakeCmd>::SharedPtr                   brake_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringCmd>::SharedPtr                steering_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::GearCmd>::SharedPtr                    gear_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::GlobalEnableCmd>::SharedPtr            enable_pub_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::MiscCmd>::SharedPtr                    misc_pub_;

  // Subscribers (from DBW reports)
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr          steering_report_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr        wheel_speed_report_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::AcceleratorPedalReport>::SharedPtr  accel_report_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::BrakeReport>::SharedPtr             brake_report_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::GearReport>::SharedPtr              gear_report_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::DriverInputReport>::SharedPtr       driver_input_report_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::MiscReport>::SharedPtr              misc_report_sub_;

  // Callback functions for DBW reports 
  void steeringReportCallback    (const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg);
  void wheelSpeedReportCallback  (const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);
  void accelReportCallback       (const raptor_dbw_msgs::msg::AcceleratorPedalReport::SharedPtr msg);
  void brakeReportCallback       (const raptor_dbw_msgs::msg::BrakeReport::SharedPtr msg);
  void gearReportCallback        (const raptor_dbw_msgs::msg::GearReport::SharedPtr msg);
  void driverInputReportCallback (const raptor_dbw_msgs::msg::DriverInputReport::SharedPtr msg);
  void miscReportCallback        (const raptor_dbw_msgs::msg::MiscReport::SharedPtr msg);
  void publishActuationStatusTimerCallback ();

  // Rolling Counter (this parameter is a common value for all published messages to Raptor DBW for each iteration)
  uint8_t counter_;

  //  Additional Control Signals from Autoware
  raptor_dbw_msgs::msg::GearCmd           gear_cmd_;
  raptor_dbw_msgs::msg::MiscCmd           misc_merged_cmd_;
  raptor_dbw_msgs::msg::MiscCmd           misc_turn_cmd_;
  raptor_dbw_msgs::msg::MiscCmd           misc_hazard_cmd_;
  raptor_dbw_msgs::msg::GlobalEnableCmd   enable_cmd_;

  // Override flags
  bool brake_override_active_;
  bool accel_override_active_;
  bool steering_override_active_;
  bool global_enable_active_;

  // Vehicle parameters
  double steering_ratio_;  

};

#endif  // RAPTOR_DBW_INTERFACE__RAPTOR_DBW_INTERFACE_HPP_
