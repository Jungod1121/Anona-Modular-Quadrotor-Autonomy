cmake_minimum_required(VERSION 3.8)
project(drone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(cascade_pid SHARED src/cascade_pid.cpp)
target_include_directories(cascade_pid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cascade_pid Eigen3)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node cascade_pid)
ament_target_dependencies(controller_node
  rclcpp nav_msgs geometry_msgs sensor_msgs std_msgs drone_msgs)

install(TARGETS cascade_pid controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(cascade_pid)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mixer test/test_mixer.cpp)
  target_link_libraries(test_mixer cascade_pid)
  ament_target_dependencies(test_mixer Eigen3)
endif()

ament_package()
