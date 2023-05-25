cmake_minimum_required(VERSION 3.5)
project(turtlebot3_tiles)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(process process.cpp)
ament_target_dependencies(
  process rclcpp cv_bridge image_transport image_transport_plugins)

install(TARGETS process DESTINATION lib/${PROJECT_NAME})

ament_package()
