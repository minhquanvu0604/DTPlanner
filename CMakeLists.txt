cmake_minimum_required(VERSION 3.0.2)
project(d_triang)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

#find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #sensor_msgs
  #std_msgs
  #nav_msgs
  #rosbag # Needed to rosbag manipulation
  roslib # Needed for ros::package::getPath
  geometry_msgs
  ackermann_msgs
  eufs_msgs
  can_msgs
  gazebo_msgs
  OpenCV REQUIRED
)

find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES d_triang
    CATKIN_DEPENDS 
    gazebo_msgs
    roscpp
    nodelet
    geometry_msgs
    eufs_msgs
    ackermann_msgs
    tf2_ros
    tf

    DEPENDS 
    eufs_msgs 
    ackermann_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    include/geometry
    include/realtime
    include/ros_integration
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${GMP_INCLUDE_DIR}
    ${MPFR_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)


#add_subdirectory(testing-code)

## Declare a C++ library
add_library(d_triang_planner src/geometry/d_triang_planner.cpp)
add_library(d_triang_realtime src/realtime/d_triang_realtime.cpp src/geometry/d_triang_planner.cpp)
add_library(d_triang_realtime_ros src/ros_integration/d_triang_realtime_ros.cpp src/realtime/d_triang_realtime.cpp src/geometry/d_triang_planner.cpp)

add_library(d_triang_planner_color_light src/geometry/d_triang_planner_color_light.cpp)
add_library(d_triang_color_light_ros src/ros_integration/d_triang_color_light_ros.cpp src/geometry/d_triang_planner_color_light.cpp)




##########add_library(geometry_test_fixture test/geometry/geometry_test_fixture.cpp)
##########add_library(realtime_test_fixture test/realtime/realtime_test_fixture.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/d_triang_node.cpp)

add_executable(get_next_edges_test test/geometry/get_next_edges_test.cpp)
add_executable(print_test test/geometry/print_test.cpp )
add_executable(return_first_edge_test test/geometry/return_first_edge_test.cpp)
add_executable(expand_test test/geometry/expand_test.cpp)

add_executable(multi_step_test test/realtime/multi_step_test.cpp)

add_executable(process_group_1_test test/color/process_group_1_test.cpp)
add_executable(offset_path_test test/color/offset_path_test.cpp)
add_executable(offset_path_plot test/color/offset_path_plot.cpp)


add_executable(utsma_path_planner src/ros_integration/utsma_path_planner.cpp)




## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(get_next_edges_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set_target_properties(print_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set_target_properties(return_first_edge_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set_target_properties(expand_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set_target_properties(multi_step_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set_target_properties(process_group_1_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set_target_properties(offset_path_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set_target_properties(offset_path_plot PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

    


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(utsma_path_planner ${utsma_path_planner_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#Geometry 
target_link_libraries(get_next_edges_test 
    d_triang_planner 
    #geometry_test_fixture 
    ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} 
    gtest_main pthread)

target_link_libraries(print_test 
    d_triang_planner 
    #geometry_test_fixture 
    ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} 
    gtest_main pthread)
    
target_link_libraries(return_first_edge_test 
    d_triang_planner 
    #geometry_test_fixture 
    ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} 
    gtest_main pthread)

target_link_libraries(expand_test 
    d_triang_planner 
    #geometry_test_fixture 
    ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} 
    gtest_main pthread)


# Realtime 
target_link_libraries(multi_step_test 
    d_triang_planner
    d_triang_realtime 
    ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} 
    gtest_main pthread)


# Color
target_link_libraries(process_group_1_test 
    d_triang_planner_color_light 
    ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} 
    pthread)

target_link_libraries(offset_path_test 
    d_triang_planner_color_light 
    ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} 
    pthread)

target_link_libraries(offset_path_plot 
    d_triang_planner_color_light 
    ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES}
    pthread)

    


# ROS Node
target_link_libraries(utsma_path_planner 
    # d_triang_planner
    # d_triang_realtime  
    #d_triang_realtime_ros
    d_triang_color_light_ros
    ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES}
    pthread)


# Offset line
# target_link_libraries(offset_path 
#     d_triang_planner_color_light
#     ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES}
#     pthread)


# # Link libraries
# target_link_libraries(d_triang_realtime_ros 
#     d_triang_planner
#     d_triang_realtime  
#     )

# target_link_libraries(d_triang_realtime 
#     d_triang_planner  
#     )



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(geometry_tests test/geometry/get_next_edges_test.cpp)
# if(TARGET geometry_tests)
#   target_link_libraries(geometry_tests d_triang_planner ${GTEST_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} gtest_main pthread)
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
