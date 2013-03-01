cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_status_window)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs )# vigir_perception_msgs)

## Declare things to be passed to other projects
catkin_package()

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES_WIDGET
  src/jointList.h
)

qt4_wrap_cpp(MOC_FILES
  src/status_window.h
  src/jointList.h
)

## Here we add QtCreator ui files
qt4_wrap_ui(UIC_FILES
  src/status_window.ui
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES_WIDGET
  src/jointList.cpp
  src/jointMain.cpp
  ${MOC_FILES_WIDGET}
)

set(SOURCE_FILES
  src/status_window.cpp
  src/jointList.cpp
  src/main.cpp
  ${MOC_FILES}
  ${UIC_FILES}
)
include_directories(include ${catkin_INCLUDE_DIRS} ../build/vigir_ocs_status_window src include)

add_executable(status_window ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(status_window ${catkin_LIBRARIES} ${QT_LIBRARIES} /opt/ros/groovy/lib/librviz.so)

add_executable(joint_list ${SOURCE_FILES_WIDGET})
target_link_libraries(joint_list ${catkin_LIBRARIES} ${QT_LIBRARIES} /opt/ros/groovy/lib/librviz.so)

#add_dependencies(camera_viewer_custom vigir_perception_msgs_gencpp)
