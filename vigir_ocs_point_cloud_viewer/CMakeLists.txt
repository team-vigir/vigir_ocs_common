cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_point_cloud_viewer)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs rviz)

## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS roscpp rospy std_msgs rviz
)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/point_cloud_viewer.h
)
## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/point_cloud_viewer.cpp
  src/main.cpp
  ${MOC_FILES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(point_cloud_viewer ${SOURCE_FILES})
target_link_libraries(point_cloud_viewer ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
