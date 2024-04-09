cmake_minimum_required(VERSION 3.12)
project(test_compute_DT)

# Find CGAL package
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(../include)


# Find Boost
# find_package(Boost REQUIRED COMPONENTS multiprecision)

# Find GMP
find_package(GMP REQUIRED)

# Include directories
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${GMP_INCLUDE_DIRS})

# Add executable
add_executable(${PROJECT_NAME} test_compute_DT.cpp)
# add_executable(test_CGAL testCGAl.cpp)

# Link against CGAL, Boost, and GMP libraries
target_link_libraries(${PROJECT_NAME} 
    CGAL::CGAL 
    ${Boost_LIBRARIES} 
    ${GMP_LIBRARIES} 
    Qt5::Widgets
    ${YAML_CPP_LIBRARIES}
    )
