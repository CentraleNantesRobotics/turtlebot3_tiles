cmake_minimum_required(VERSION 3.5)
project(turtlebot3_tiles)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(VISP REQUIRED)

add_executable(process process.cpp)
target_include_directories(process PRIVATE include)
ament_target_dependencies(
  process rclcpp cv_bridge image_transport image_transport_plugins)
target_link_libraries(process ${OpenCV_LIBRARIES} ${VISP_LIBRARIES})

install(TARGETS process DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/replay.py src/build_model.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY images model DESTINATION share/${PROJECT_NAME})

ament_package()
