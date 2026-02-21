#ifndef RAPTOR_DBW_INTERFACE__RAPTOR_ENCODER_ODOMETRY_HPP_
#define RAPTOR_DBW_INTERFACE__RAPTOR_ENCODER_ODOMETRY_HPP_

#include "rclcpp/rclcpp.hpp"

// Autoware Report messages
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"

// Raptor DBW Report messages
#include "raptor_dbw_msgs/msg/steering_report.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"

// TF messages and broadcaster
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class RaptorEncoderOdometry : public rclcpp::Node
{
public:
  explicit RaptorEncoderOdometry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  // Autoware Status msgs
  autoware_vehicle_msgs::msg::VelocityReport        velocity_report_;

  // Subscribers (from Autoware)
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr       velocity_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr       steering_status_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr  actuation_status_sub_;

  // Subscribers (from DBW reports)
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr             steering_report_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr           wheel_speed_report_sub_;

  // Callback functions for DBW reports 
  void steeringReportCallback    (const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg);
  void wheelSpeedReportCallback  (const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);

  // Callback functions for Autoware reports 
  void velocityStatusCallback    (const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);
  void steeringStatusCallback    (const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
  void actuationStatusCallback   (const tier4_vehicle_msgs::msg::ActuationStatusStamped::SharedPtr msg);

  // Timer callback functions for actuation status and control commands
  void publishEncoderOdometryTimerCallback();
  rclcpp::TimerBase::SharedPtr odometry_timer_;
  
  // TF Broadcaster and transform message
  geometry_msgs::msg::TransformStamped transform_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Odometry Publisher and Odometry message
  nav_msgs::msg::Odometry odom_msg_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Vehicle parameters
  double steering_ratio_;  
  double max_decel_;  
  double max_accel_;
  double max_jerk_;
  double wheel_radius_;
  double wheel_base_;
  double wheel_seperetion_;
  std::string base_link_frame_;
  std::string odom_frame_;
  bool publish_tf_;

  // Subscribed datum for odometry calculation
  double lin_velocity_;
  double steering_status_;
  double x_dot_;
  double y_dot_;
  double yaw_rate_;
  double x_;
  double y_;
  double yaw_;
  bool first_run_;
  rclcpp::Time last_time_;


};

#endif  // RAPTOR_DBW_INTERFACE__RAPTOR_ENCODER_ODOMETRY_HPP_
