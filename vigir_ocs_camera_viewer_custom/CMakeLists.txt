cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_camera_viewer_custom)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs pluginlib vigir_ocs_rviz_plugins)

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
qt4_wrap_cpp(MOC_FILES
  src/camera_viewer_custom.h
)
## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/camera_viewer_custom.cpp
  src/main.cpp
  ${MOC_FILES}
)

include_directories(include ${catkin_INCLUDE_DIRS} /opt/vigir/vigir_perception/vigir_perception_msgs/msg_gen/cpp/include)

add_executable(camera_viewer_custom ${SOURCE_FILES})
target_link_libraries(camera_viewer_custom ${catkin_LIBRARIES} ${QT_LIBRARIES} /opt/ros/groovy/lib/librviz.so)


