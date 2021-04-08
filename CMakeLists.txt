cmake_minimum_required(VERSION 2.8.3)
project(grid_localisation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  message_filters
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)


catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


## Declare a C++ library
add_library(libraries
  src/motionModel.cpp
  src/measurementModel.cpp
  src/gridLocalisation.cpp
  src/localisationServer.cpp
  src/utils.cpp
  src/map_utils.cpp
)

target_link_libraries(libraries ${catkin_LIBRARIES})

find_package(Threads)
add_executable(main src/main.cpp)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} libraries)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O3")