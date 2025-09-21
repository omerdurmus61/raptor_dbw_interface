#include "rclcpp/rclcpp.hpp"
#include "raptor_dbw_interface/raptor_dbw_interface.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaptorDbwInterface>());
  rclcpp::shutdown();
  return 0;
}
