cmake_minimum_required(VERSION 2.8.3)
project(tmplanner_tools)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED gazebo_ros )
find_package(Eigen3 REQUIRED)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME}_factory SHARED sim/descriptions/models/factory.cc)

catkin_simple(ALL_DEPS_REQUIRED)

target_link_libraries(${PROJECT_NAME}_factory
  ${GAZEBO_LIBRARIES}
)

cs_install()
cs_export()
