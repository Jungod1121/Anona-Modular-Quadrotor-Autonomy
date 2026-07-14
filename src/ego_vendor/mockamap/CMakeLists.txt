cmake_minimum_required(VERSION 3.8)
project(mockamap)

# Align deps with map_generator in this workspace (PCL + pcl_conversions; no pcl_ros).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include/mockamap
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Official Tree: mockamap.cpp + maps.cpp + perlinnoise.cpp.
# Skip ces_randommap.cpp (legacy alternate node, unused by Maps::generate).
add_executable(mockamap_node
  src/mockamap.cpp
  src/maps.cpp
  src/perlinnoise.cpp
)

ament_target_dependencies(mockamap_node
  rclcpp
  pcl_conversions
  nav_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  Eigen3
)
target_link_libraries(mockamap_node ${PCL_LIBRARIES})

install(TARGETS mockamap_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
