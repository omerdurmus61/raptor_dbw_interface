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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(raptor_dbw_interface_node
  src/main.cpp
  src/raptor_dbw_interface.cpp
)

ament_target_dependencies(raptor_dbw_interface_node
  rclcpp
  std_msgs
  raptor_dbw_msgs
  autoware_vehicle_msgs
  autoware_control_msgs
  tier4_vehicle_msgs
)

install(TARGETS
  raptor_dbw_interface_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

ament_package()
