cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_video_record_widget)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs)

## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy roslib std_msgs
)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

include(${LD_LIBRARY_PATH}/lib64)

qt4_wrap_cpp(MOC_FILES
  src/ui/video_record_widget.h
)

## Here we add QtCreator ui files
qt4_wrap_ui(UIC_FILES
  src/ui/video_record_widget.ui
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/ui/video_record_widget.cpp
  src/main.cpp
  ${MOC_FILES}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} src)

add_executable(video_record_widget ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(video_record_widget ${catkin_LIBRARIES} ${QT_LIBRARIES} )

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
