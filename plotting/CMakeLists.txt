cmake_minimum_required(VERSION 3.12)
project(d_triang_plot)

# Find CGAL package
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    ../include/geometry
    include
    src
    )


# Find Boost
# find_package(Boost REQUIRED COMPONENTS multiprecision)

# Find GMP
#find_package(GMP REQUIRED)

# Include directories
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
#include_directories(${GMP_INCLUDE_DIRS})

# Add executable
add_executable(d_triang_plot src/d_triang_plot.cpp)
add_executable(d_triang_plot_color src/d_triang_plot_color.cpp)



add_library(d_triang_planner ../src/geometry/d_triang_planner.cpp)
add_library(d_triang_planner_color_light ../src/geometry/d_triang_planner_color_light.cpp)

add_library(d_triang_Qt src/d_triang_Qt.cpp)




# Link against CGAL, Boost, and GMP libraries
target_link_libraries(d_triang_plot
    CGAL::CGAL 
    ${Boost_LIBRARIES} 
    #${GMP_LIBRARIES} 
    ${YAML_CPP_LIBRARIES}
    d_triang_planner
    d_triang_Qt
    Qt5::Widgets
    )

target_link_libraries(
    d_triang_Qt
    Qt5::Widgets
)

target_link_libraries(d_triang_plot_color
    CGAL::CGAL 
    ${Boost_LIBRARIES} 
    #${GMP_LIBRARIES} 
    ${YAML_CPP_LIBRARIES}
    d_triang_planner_color_light
    d_triang_Qt
    Qt5::Widgets
    )