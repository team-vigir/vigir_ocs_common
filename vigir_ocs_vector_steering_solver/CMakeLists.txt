cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_vector_steering_solver)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

## Declare things to be passed to other projects
catkin_package(
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS roscpp rospy std_msgs
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/vector_steering_solver.cpp
  src/main.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(vector_steering_solver ${SOURCE_FILES})
target_link_libraries(vector_steering_solver ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
