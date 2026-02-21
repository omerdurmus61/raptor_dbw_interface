cmake_minimum_required(VERSION 3.8)
project(raptor_dbw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(raptor_dbw_interface_node
  src/raptor_dbw_interface_node.cpp
  src/raptor_dbw_interface.cpp
)

add_executable(raptor_encoder_odometry_node
  src/raptor_encoder_odometry_node.cpp
  src/raptor_encoder_odometry.cpp
)

ament_target_dependencies(raptor_dbw_interface_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  raptor_dbw_msgs
  autoware_vehicle_msgs
  autoware_control_msgs
  tier4_vehicle_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(raptor_encoder_odometry_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  raptor_dbw_msgs
  autoware_vehicle_msgs
  autoware_control_msgs
  tier4_vehicle_msgs
  tf2
  tf2_ros
)

install(TARGETS
  raptor_dbw_interface_node
  raptor_encoder_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
