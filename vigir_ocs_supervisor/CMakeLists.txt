cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_supervisor)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs rviz flor_ocs_msgs flor_control_msgs)

## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS ${catkin_INCLUDE_DIRS} src
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS roscpp rospy std_msgs rviz flor_ocs_msgs flor_control_msgs
)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we add QtCreator ui files
qt4_wrap_ui(UIC_FILES
  src/ui/widget.ui
)

qt4_wrap_cpp(MOC_FILES
  
  src/ui/widget.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/main.cpp
  src/ui/widget.cpp
  ${MOC_FILES}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} src /usr/share/drcsim-2.7/ros/atlas_msgs/msg_gen/cpp/include)

add_executable(supervisor ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(supervisor ${catkin_LIBRARIES} ${QT_LIBRARIES})


