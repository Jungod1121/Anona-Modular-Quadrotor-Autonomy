cmake_minimum_required(VERSION 3.8)
project(drone_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl" CACHE PATH "PCL CMake dir" FORCE)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  third_party
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(uniform_bspline SHARED src/uniform_bspline.cpp)
target_include_directories(uniform_bspline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(uniform_bspline Eigen3)

add_library(drone_planner_core SHARED
  src/occupancy_grid.cpp
  src/raycast.cpp
  src/dyn_astar.cpp
)
target_include_directories(drone_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(drone_planner_core Eigen3)

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node uniform_bspline drone_planner_core ${PCL_LIBRARIES})
ament_target_dependencies(planner_node
  rclcpp nav_msgs geometry_msgs sensor_msgs drone_msgs pcl_conversions)

install(TARGETS uniform_bspline drone_planner_core planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY third_party/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bspline_optimizer test/test_bspline_optimizer.cpp)
  target_link_libraries(test_bspline_optimizer uniform_bspline drone_planner_core)
  target_include_directories(test_bspline_optimizer PRIVATE include third_party)

  ament_add_gtest(test_dyn_astar test/test_dyn_astar.cpp)
  target_link_libraries(test_dyn_astar drone_planner_core)
  target_include_directories(test_dyn_astar PRIVATE include)

  ament_add_gtest(test_occupancy_grid test/test_occupancy_grid.cpp)
  target_link_libraries(test_occupancy_grid drone_planner_core)
  target_include_directories(test_occupancy_grid PRIVATE include)

  ament_add_gtest(test_path_shortcut test/test_path_shortcut.cpp)
  target_link_libraries(test_path_shortcut drone_planner_core)
  target_include_directories(test_path_shortcut PRIVATE include)
endif()

ament_package()
