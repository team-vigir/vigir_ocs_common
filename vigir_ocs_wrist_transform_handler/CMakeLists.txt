cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_wrist_transform_handler)

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp  
  geometry_msgs
  tf
  tf_conversions
  moveit_core
  moveit_ros_planning
  vigir_ocs_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vigir_drake_matlab
CATKIN_DEPENDS   
  geometry_msgs
  tf
  tf_conversions
  moveit_core
  moveit_ros_planning
  vigir_ocs_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE
  src/main.cpp
  src/wrist_transform_handler.cpp
  include/vigir_ocs_wrist_transform_handler/wrist_transform_handler.h
)

## Declare a cpp executable
add_executable(wrist_transform_handler ${SOURCE})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(wrist_transform_handler vigir_planning_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(wrist_transform_handler
   ${catkin_LIBRARIES}
 )

### Install node
install(TARGETS wrist_transform_handler
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
