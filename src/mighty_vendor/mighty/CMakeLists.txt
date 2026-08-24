cmake_minimum_required(VERSION 3.8)
project(mighty)

# Plant-only Path E build: no Gazebo / mesh / optional tests.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Default build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(BUILD_SIMULATION OFF CACHE BOOL "Gazebo targets disabled for drone_ws" FORCE)

# Resolve PCL before any ROS packages that call find_package(PCL) without COMPONENTS.
find_package(PCL REQUIRED COMPONENTS common io filters kdtree octree search)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(decomp_rviz_plugins REQUIRED)
find_package(dynus_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
  /opt/ros/humble/include
  /opt/ros/humble/include/visualization_msgs
  /opt/ros/humble/include/geometry_msgs
  /opt/ros/humble/include/std_msgs
  /opt/ros/humble/include/nav_msgs
  /opt/ros/humble/include/sensor_msgs
  /opt/ros/humble/include/builtin_interfaces
  /opt/ros/humble/include/rclcpp
  /opt/ros/humble/include/rcutils
  /opt/ros/humble/include/rmw
  /opt/ros/humble/include/tf2
  /opt/ros/humble/include/tf2_ros
  /opt/ros/humble/include/tf2_geometry_msgs
  /opt/ros/humble/include/tf2_eigen
  /opt/ros/humble/include/std_srvs
  /opt/ros/humble/include/example_interfaces
  /opt/ros/humble/include/message_filters
  /opt/ros/humble/include/libstatistics_collector
  /opt/ros/humble/include/statistics_msgs
  /opt/ros/humble/include/tracetools
  /opt/ros/humble/include/rcl
  /opt/ros/humble/include/rcl_interfaces
  /opt/ros/humble/include/rcl_yaml_param_parser
  /opt/ros/humble/include/rcpputils
  /opt/ros/humble/include/rosidl_runtime_c
  /opt/ros/humble/include/rosidl_runtime_cpp
  /opt/ros/humble/include/rosidl_typesupport_interface
)

# Ensure vendored decomp_util headers are visible (INTERFACE export is fragile).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../decomp_util/include")
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../decomp_util/include")
endif()
# dynus / decomp installed headers (Humble nested layout)
foreach(_p
    "${CMAKE_INSTALL_PREFIX}/../dynus_interfaces/include"
    "${CMAKE_CURRENT_BINARY_DIR}/../../install/dynus_interfaces/include"
    "${CMAKE_INSTALL_PREFIX}/../dynus_interfaces/include"
    "${CMAKE_INSTALL_PREFIX}/../dynus_interfaces/include/dynus_interfaces"
    "${CMAKE_INSTALL_PREFIX}/../decomp_ros_msgs/include"
    "${CMAKE_INSTALL_PREFIX}/../decomp_ros_msgs/include/decomp_ros_msgs"
    "${CMAKE_INSTALL_PREFIX}/../decomp_rviz_plugins/include")
  if(EXISTS "${_p}")
    include_directories("${_p}")
  endif()
endforeach()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(OpenMP_CXX_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(MIGHTY_SOURCES
  src/mighty/mighty_node.cpp
  src/mighty/mighty.cpp
  src/hgp/hgp_manager.cpp
  src/hgp/utils.cpp
  src/hgp/hgp_planner.cpp
  src/hgp/graph_search.cpp
  src/mighty/utils.cpp
  src/mighty/lbfgs_solver.cpp
  src/mighty/lbfgs_solver_utils.cpp
  src/mighty/frontier_detector.cpp
  src/mighty/frontier_manager.cpp
)

add_executable(mighty ${MIGHTY_SOURCES})
target_link_libraries(mighty ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
ament_target_dependencies(mighty
  rclcpp example_interfaces dynus_interfaces visualization_msgs
  decomp_ros_msgs decomp_util decomp_rviz_plugins sensor_msgs std_msgs nav_msgs
  geometry_msgs tf2_geometry_msgs tf2 tf2_ros tf2_eigen std_srvs)


add_executable(convert_odom_to_state src/mighty/convert_odom_to_state.cpp)
ament_target_dependencies(convert_odom_to_state
  rclcpp dynus_interfaces nav_msgs geometry_msgs Eigen3)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS mighty convert_odom_to_state DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/repub_rviz_2Dgoal.py")
  install(PROGRAMS scripts/repub_rviz_2Dgoal.py DESTINATION lib/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mighty/__init__.py")
  ament_python_install_package(${PROJECT_NAME})
endif()

ament_package()
