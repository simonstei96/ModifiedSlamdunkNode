cmake_minimum_required(VERSION 2.8.12)
project(slamdunk_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  tf2_ros
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)
pkg_search_module(KALAMOS_CONTEXT REQUIRED kalamos-context)


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
#  LIBRARIES slamdunk_node
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(slamdunk_node src/main.cpp src/cvutils.cpp)

target_include_directories(slamdunk_node
   PRIVATE
   ${Boost_INCLUDE_DIRS}
   ${KALAMOS_CONTEXT_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(slamdunk_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${KALAMOS_CONTEXT_LIBRARIES}
   yaml-cpp
)

