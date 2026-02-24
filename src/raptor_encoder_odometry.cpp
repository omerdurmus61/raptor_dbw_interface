#include "raptor_dbw_interface/raptor_encoder_odometry.hpp"

RaptorEncoderOdometry::RaptorEncoderOdometry(const rclcpp::NodeOptions & options)
: Node("raptor_encoder_odometry", options)
{
    
  // Vehicle Parameters
  steering_ratio_     = this->declare_parameter<double>("steering_ratio", 16.2);     // steering_wheel (deg) / tier angle (rad)
  max_decel_          = this->declare_parameter<double>("max_decel", 5.0);           // m/s^2
  max_accel_          = this->declare_parameter<double>("max_accel", 2.0);           // m/s^2
  max_jerk_           = this->declare_parameter<double>("max_jerk", 1.0);            // m/s^3  
  wheel_radius_       = this->declare_parameter<double>("wheel_radius", 0.365);      // m 
  wheel_base_         = this->declare_parameter<double>("wheel_base", 3.0);          // m 
  wheel_seperetion_   = this->declare_parameter<double>("wheel_seperetion", 2.5);    // m 
  odom_frame_         = this->declare_parameter<std::string>("odom_frame", "raptor_encoder_odom");    
  base_link_frame_    = this->declare_parameter<std::string>("base_link_frame", "raptor_base_link");   
  publish_tf_         = this->declare_parameter<bool>("publish_tf", true);   

  RCLCPP_INFO(
        this->get_logger(),
        "Encoder odometry node initialized (odom_frame=%s, base_link=%s, tf=%s)",
        odom_frame_.c_str(),
        base_link_frame_.c_str(),
        publish_tf_ ? "on" : "off"
      );

  lin_velocity_ = 0.0;
  steering_status_ = 0.0;
  x_dot_ = 0.0;
  y_dot_ = 0.0;
  yaw_rate_ = 0.0;
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;

  first_run_ = true;
  last_time_ = this->get_clock()->now();;


  // Subscribers (from Autoware)
  velocity_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status",10,
    std::bind(&RaptorEncoderOdometry::velocityStatusCallback, this, std::placeholders::_1));

  steering_status_sub_= this->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status",10,
    std::bind(&RaptorEncoderOdometry::steeringStatusCallback, this, std::placeholders::_1));

  actuation_status_sub_= this->create_subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
    "/vehicle/status/actuation_status",10,
    std::bind(&RaptorEncoderOdometry::actuationStatusCallback, this, std::placeholders::_1));

  // Subscribers (from DBW reports)
  steering_report_sub_= this->create_subscription<raptor_dbw_msgs::msg::SteeringReport>(
        "/raptor_dbw_interface/steering_report",10,
        std::bind(&RaptorEncoderOdometry::steeringReportCallback, this, std::placeholders::_1));

  wheel_speed_report_sub_= this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "/raptor_dbw_interface/wheel_speed_report",10,
        std::bind(&RaptorEncoderOdometry::wheelSpeedReportCallback, this, std::placeholders::_1));

  // Odometry Publishers and broadcaster
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/raptor_dbw_interface/encoder_odometry", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Odometry Timer
  odometry_timer_    = this->create_wall_timer(20ms,std::bind(&RaptorEncoderOdometry::publishEncoderOdometryTimerCallback, this));

}


void RaptorEncoderOdometry::publishEncoderOdometryTimerCallback()
{   

  //Bicycle model
  rclcpp::Time now = this->get_clock()->now();
  
  double dt = (now - last_time_).seconds();
  last_time_ = now;

  yaw_rate_ = (lin_velocity_ * std::tan(steering_status_)) / wheel_base_;

  yaw_ += yaw_rate_ * dt;

  double vx = lin_velocity_ * std::cos(yaw_);
  double vy = lin_velocity_ * std::sin(yaw_);

  x_ += vx * dt;
  y_ += vy * dt;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  q.normalize();

  odom_msg_.header.stamp = now;
  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = base_link_frame_;

  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.position.z = 0.0;

  odom_msg_.pose.pose.orientation.x = q.x();
  odom_msg_.pose.pose.orientation.y = q.y();
  odom_msg_.pose.pose.orientation.z = q.z();
  odom_msg_.pose.pose.orientation.w = q.w();

  odom_msg_.twist.twist.linear.x = lin_velocity_;
  odom_msg_.twist.twist.linear.y = 0.0;
  odom_msg_.twist.twist.angular.z = yaw_rate_;

  // Publish odom
  odom_pub_->publish(odom_msg_);

  if(publish_tf_){
    
    transform_.header.stamp = now;
    transform_.header.frame_id = odom_frame_;
    transform_.child_frame_id = base_link_frame_;

    transform_.transform.translation.x = x_;
    transform_.transform.translation.y = y_;
    transform_.transform.translation.z = 0.0;

    transform_.transform.rotation.x = q.x();
    transform_.transform.rotation.y = q.y();
    transform_.transform.rotation.z = q.z();
    transform_.transform.rotation.w = q.w();

    // Broadcast TF
    tf_broadcaster_->sendTransform(transform_);
  }

}

void RaptorEncoderOdometry::velocityStatusCallback(
  const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  lin_velocity_ = msg->longitudinal_velocity;
}

void RaptorEncoderOdometry::steeringStatusCallback(
  const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
{
  steering_status_ = msg->steering_tire_angle;
}

void RaptorEncoderOdometry::actuationStatusCallback(
  const tier4_vehicle_msgs::msg::ActuationStatusStamped::SharedPtr )
{
  // TODO: implement
}

void RaptorEncoderOdometry::steeringReportCallback(
  const raptor_dbw_msgs::msg::SteeringReport::SharedPtr )
{
  // TODO: implement
}

void RaptorEncoderOdometry::wheelSpeedReportCallback(
  const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr )
{
  // TODO: implement
}
