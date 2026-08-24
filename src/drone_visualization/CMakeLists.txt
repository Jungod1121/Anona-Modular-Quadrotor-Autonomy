cmake_minimum_required(VERSION 3.8)
project(drone_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(viz_node src/viz_node.cpp)
ament_target_dependencies(viz_node
  rclcpp nav_msgs geometry_msgs visualization_msgs)
target_include_directories(viz_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS viz_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
