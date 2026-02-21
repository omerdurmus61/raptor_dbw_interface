#include "rclcpp/rclcpp.hpp"
#include "raptor_dbw_interface/raptor_encoder_odometry.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaptorEncoderOdometry>());
  rclcpp::shutdown();
  return 0;
}
