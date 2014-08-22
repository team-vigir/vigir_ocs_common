cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_camera_viewer_custom)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs rviz pluginlib vigir_ocs_rviz_plugins flor_ocs_msgs flor_perception_msgs vigir_ocs_base_3d_view )

## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS ${catkin_INCLUDE_DIRS} src src/ui
	LIBRARIES ${PROJECT_NAME} 
        CATKIN_DEPENDS roscpp rospy std_msgs rviz pluginlib vigir_ocs_rviz_plugins flor_ocs_msgs flor_perception_msgs vigir_ocs_base_3d_view
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
  src/empty_view_controller.h
  src/camera_viewer_custom.h
)

qt4_wrap_cpp(MOC_FILES
  src/empty_view_controller.h
  src/camera_viewer_custom.h
  src/ui/camera_viewer_custom_widget.h
)

## Here we add QtCreator ui files
qt4_wrap_ui(UIC_FILES
  src/ui/camera_viewer_custom_widget.ui
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES_WIDGET
  src/empty_view_controller.cpp
  src/camera_viewer_custom.cpp
  src/main_widget.cpp
  ${MOC_FILES_WIDGET}
)

set(SOURCE_FILES
  src/empty_view_controller.cpp
  src/camera_viewer_custom.cpp
  src/ui/camera_viewer_custom_widget.cpp
  src/main.cpp
  ${MOC_FILES}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} src src/ui )

add_executable(camera_viewer_custom ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(camera_viewer_custom ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(camera_viewer_custom ${catkin_EXPORTED_TARGETS})

add_executable(camera_viewer_custom_widget ${SOURCE_FILES_WIDGET})
target_link_libraries(camera_viewer_custom_widget ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(camera_viewer_custom_widget ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
