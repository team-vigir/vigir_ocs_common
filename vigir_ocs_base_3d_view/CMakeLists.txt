cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_base_3d_view)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(PCL REQUIRED QUIET COMPONENTS common)

find_package(catkin REQUIRED COMPONENTS pcl_ros roscpp rospy std_msgs geometry_msgs sensor_msgs rviz vigir_ocs_rviz_plugins vigir_ocs_template_loader vigir_ocs_render_panel_custom vigir_ocs_mouse_event_handler flor_interactive_marker_server_custom moveit_core moveit_ros_planning flor_control_msgs vigir_ocs_robot_state_manager vigir_footstep_planning_msgs vigir_ocs_notification_system vigir_ocs_hotkey_manager)


## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME} 
        CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs geometry_msgs sensor_msgs rviz vigir_ocs_rviz_plugins vigir_ocs_template_loader vigir_ocs_render_panel_custom vigir_ocs_mouse_event_handler flor_interactive_marker_server_custom moveit_core moveit_ros_planning flor_control_msgs vigir_ocs_robot_state_manager vigir_footstep_planning_msgs vigir_ocs_notification_system vigir_ocs_hotkey_manager

)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  src/footstep_vis_manager.h
  src/base_3d_view.h
  src/base_context_menu.h
  src/context_menu_manager.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/footstep_vis_manager.cpp
  src/base_3d_view.cpp
  src/base_context_menu.cpp
  src/context_menu_manager.cpp
  ${MOC_FILES}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} src)

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

