cmake_minimum_required(VERSION 3.8)
project(decomp_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  decomp_util decomp_ros_msgs sensor_msgs nav_msgs geometry_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(decomp_util decomp_ros_msgs sensor_msgs nav_msgs geometry_msgs Eigen3)
ament_package()
