cmake_minimum_required(VERSION 3.8)
project(drone_fast_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(fast_planning_node src/fast_planning_node.cpp)
ament_target_dependencies(fast_planning_node
  rclcpp nav_msgs geometry_msgs sensor_msgs drone_msgs)

install(TARGETS fast_planning_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
