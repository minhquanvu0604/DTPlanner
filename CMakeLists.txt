cmake_minimum_required(VERSION 3.5)
project(dt_planner)

## Use C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
# ament_python_install_package()

###################################
## ament specific configuration ##
###################################
ament_package(
  INCLUDE_DIRS include
  LIBRARIES d_triang
  DEPENDS
    geometry_msgs
    eufs_msgs
    ackermann_msgs
    rclcpp
    OpenCV
    CGAL
    Qt5
    GMP
    MPFR
    yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${GMP_INCLUDE_DIR}
    ${MPFR_INCLUDE_DIR}
)

## Declare a C++ library
add_library(d_triang_planner src/geometry/d_triang_planner.cpp)
ament_target_dependencies(d_triang_planner rclcpp OpenCV CGAL Qt5 GMP MPFR yaml-cpp)

add_library(d_triang_realtime src/realtime/d_triang_realtime.cpp src/geometry/d_triang_planner.cpp)
ament_target_dependencies(d_triang_realtime rclcpp OpenCV CGAL Qt5 GMP MPFR yaml-cpp)

add_library(d_triang_realtime_ros src/ros_integration/d_triang_realtime_ros.cpp src/realtime/d_triang_realtime.cpp src/geometry/d_triang_planner.cpp)
ament_target_dependencies(d_triang_realtime_ros rclcpp OpenCV CGAL Qt5 GMP MPFR yaml-cpp)

add_library(d_triang_planner_color_light src/geometry/d_triang_planner_color_light.cpp)
ament_target_dependencies(d_triang_planner_color_light rclcpp OpenCV CGAL Qt5 GMP MPFR yaml-cpp)

add_library(d_triang_color_light_ros src/ros_integration/d_triang_color_light_ros.cpp src/geometry/d_triang_planner_color_light.cpp)
ament_target_dependencies(d_triang_color_light_ros rclcpp OpenCV CGAL Qt5 GMP MPFR yaml-cpp)

## Declare a C++ executable
add_executable(utsma_path_planner src/ros_integration/utsma_path_planner.cpp)
ament_target_dependencies(utsma_path_planner rclcpp OpenCV CGAL Qt5 GMP MPFR yaml-cpp)


### TEST EXECUTABLES
# Geometry Tests
add_executable(get_next_edges_test test/geometry/get_next_edges_test.cpp)
ament_target_dependencies(get_next_edges_test
  rclcpp
  d_triang_planner
  GTest
  GMP
  MPFR
  yaml-cpp
)
target_link_libraries(get_next_edges_test gtest_main pthread)

add_executable(print_test test/geometry/print_test.cpp)
ament_target_dependencies(print_test
  rclcpp
  d_triang_planner
  GMP
  MPFR
  yaml-cpp
)
target_link_libraries(print_test gtest_main pthread)

add_executable(return_first_edge_test test/geometry/return_first_edge_test.cpp)
ament_target_dependencies(return_first_edge_test
  rclcpp
  d_triang_planner
  GTest
  GMP
  MPFR
  yaml-cpp
)
target_link_libraries(return_first_edge_test gtest_main pthread)

add_executable(expand_test test/geometry/expand_test.cpp)
ament_target_dependencies(expand_test
  rclcpp
  d_triang_planner
  GTest
  GMP
  MPFR
  yaml-cpp
)
target_link_libraries(expand_test gtest_main pthread)

# Realtime Test
add_executable(multi_step_test test/realtime/multi_step_test.cpp)
ament_target_dependencies(multi_step_test
  rclcpp
  d_triang_planner
  d_triang_realtime
  GTest
  GMP
  MPFR
  yaml-cpp
)
target_link_libraries(multi_step_test gtest_main pthread)

# Color Tests
add_executable(process_group_1_test test/color/process_group_1_test.cpp)
ament_target_dependencies(process_group_1_test
  rclcpp
  d_triang_planner_color_light
  GTest
  GMP
  MPFR
  yaml-cpp
)
target_link_libraries(process_group_1_test pthread)

add_executable(offset_path_test test/color/offset_path_test.cpp)
ament_target_dependencies(offset_path_test
  rclcpp
  d_triang_planner_color_light
  GTest
  GMP
  MPFR
  yaml-cpp
)
target_link_libraries(offset_path_test pthread)

add_executable(offset_path_plot test/color/offset_path_plot.cpp)
ament_target_dependencies(offset_path_plot
  rclcpp
  d_triang_planner_color_light
  GTest
  GMP
  MPFR
  yaml-cpp
  OpenCV
)
target_link_libraries(offset_path_plot pthread)


## Install targets
install(TARGETS
  utsma_path_planner
  d_triang_planner
  d_triang_realtime
  d_triang_realtime_ros
  d_triang_planner_color_light
  d_triang_color_light_ros
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

## Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Add test executables here
endif()

## Additional CMake rules for installation and testing can be added similarly
