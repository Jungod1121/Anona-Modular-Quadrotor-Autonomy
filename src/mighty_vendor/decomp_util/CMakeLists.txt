cmake_minimum_required(VERSION 3.8)
project(decomp_util)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(decomp_util INTERFACE)
target_include_directories(decomp_util INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS decomp_util EXPORT export_decomp_util)

ament_export_targets(export_decomp_util HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
