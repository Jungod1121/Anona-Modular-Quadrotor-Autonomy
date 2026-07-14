cmake_minimum_required(VERSION 3.8)
project(drone_dynamics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(quadrotor_model SHARED
  src/quadrotor_model.cpp
)
target_include_directories(quadrotor_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(quadrotor_model Eigen3)

add_executable(dynamics_node src/dynamics_node.cpp)
target_link_libraries(dynamics_node quadrotor_model)
ament_target_dependencies(dynamics_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs drone_msgs
)

install(TARGETS quadrotor_model dynamics_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_allocation test/test_allocation.cpp)
  target_link_libraries(test_allocation quadrotor_model)
endif()

ament_package()
