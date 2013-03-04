cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_main_3d_view)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs rviz vigir_ocs_rviz_plugins vigir_ocs_template_loader)

## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS /opt/vigir/vigir_sandbox/vigir_ocs_msg2/msg_gen/cpp/include src
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs rviz vigir_ocs_rviz_plugins vigir_ocs_template_loader 
)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES_WIDGET
  src/main_3d_view.h
)

qt4_wrap_cpp(MOC_FILES
  src/main_3d_view.h
  src/ui/main_3d_view_widget.h
)

## Here we add QtCreator ui files
qt4_wrap_ui(UIC_FILES
  src/ui/main_3d_view_widget.ui
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES_WIDGET
  src/main_3d_view.cpp
  src/main_widget.cpp
  ${MOC_FILES_WIDGET}
)

set(SOURCE_FILES
  src/main_3d_view.cpp
  src/ui/main_3d_view_widget.cpp
  src/main.cpp
  ${MOC_FILES}
)

include_directories(/opt/vigir/vigir_sandbox/vigir_ocs_msg2/msg_gen/cpp/include ${catkin_INCLUDE_DIRS} ../build/vigir_ocs_main_3d_view src)

add_executable(main_3d_view ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(main_3d_view ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_executable(main_3d_view_widget ${SOURCE_FILES_WIDGET})
target_link_libraries(main_3d_view_widget ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
