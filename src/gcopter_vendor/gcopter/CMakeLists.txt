cmake_minimum_required(VERSION 3.8)
project(gcopter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-Wall -O3)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(drone_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(global_planning_node src/global_planning.cpp)

ament_target_dependencies(global_planning_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  drone_msgs
)

install(TARGETS global_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE VENDOR_NOTES.md
  DESTINATION share/${PROJECT_NAME}
)

# Headers used by the node are all header-only; install for dependents.
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
