cmake_minimum_required(VERSION 3.8)
project(drone_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# Ubuntu multiarch: resolve PCL via /usr/... so PCL_ROOT is /usr (not /)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl" CACHE PATH "PCL cmake dir" FORCE)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_library(map_generator SHARED
  src/map_generator.cpp
  src/ego_map.cpp
)
target_include_directories(map_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(map_generator ${PCL_LIBRARIES})
ament_target_dependencies(map_generator
  sensor_msgs visualization_msgs pcl_conversions
)

add_executable(map_node src/map_node.cpp)
target_link_libraries(map_node map_generator)
ament_target_dependencies(map_node
  rclcpp sensor_msgs visualization_msgs nav_msgs pcl_conversions
)

install(TARGETS map_generator map_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ego_map test/test_ego_map.cpp)
  target_link_libraries(test_ego_map map_generator)
endif()

ament_package()
