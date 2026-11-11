cmake_minimum_required(VERSION 3.8)
project(go2_cmd_vel_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(go2_cmd_vel_bridge src/go2_cmd_vel_bridge.cpp)
target_compile_features(go2_cmd_vel_bridge PRIVATE cxx_std_17)
ament_target_dependencies(
  go2_cmd_vel_bridge
  rclcpp
  rcl_interfaces
  geometry_msgs
  unitree_api
)

install(TARGETS
  go2_cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()