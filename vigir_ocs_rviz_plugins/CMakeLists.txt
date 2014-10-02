cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_rviz_plugins)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs geometry_msgs flor_perception_msgs flor_ocs_msgs flor_interactive_marker_server_custom rviz pluginlib class_loader)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(OpenGL REQUIRED)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES
    vigir_ocs_rviz_plugin_camera_display_custom
    vigir_ocs_rviz_plugin_image_display_custom
    vigir_ocs_rviz_plugin_template_display_custom
    vigir_ocs_rviz_plugin_image_selection_tool_custom
    vigir_ocs_rviz_plugin_selection_3d_tool_custom
    vigir_ocs_rviz_plugin_selection_3d_display_custom
    vigir_ocs_rviz_plugin_bounding_object_display_custom
    vigir_ocs_rviz_plugin_path_display_custom
    vigir_ocs_rviz_plugin_octomap_display_custom
    vigir_ocs_rviz_plugin_grasp_display_custom
    vigir_ocs_rviz_plugin_robot_display_custom
    vigir_ocs_rviz_plugin_map_display_custom
    vigir_ocs_rviz_plugin_frustum_display_custom
    vigir_ocs_rviz_plugin_interaction_tool_custom
    vigir_ocs_rviz_plugin_ortho_view_controller_custom
    vigir_ocs_rviz_plugin_overlay_display_custom    
    ${OPENGL_LIBRARIES}
  INCLUDE_DIRS
    vigir_ocs_rviz_plugin_camera_display_custom/src
    vigir_ocs_rviz_plugin_image_display_custom/src
    vigir_ocs_rviz_plugin_template_display_custom/src
    vigir_ocs_rviz_plugin_image_selection_tool_custom/src
    vigir_ocs_rviz_plugin_selection_3d_tool_custom/src
    vigir_ocs_rviz_plugin_selection_3d_display_custom/src
    vigir_ocs_rviz_plugin_bounding_object_display_custom/src
    vigir_ocs_rviz_plugin_path_display_custom/src
    vigir_ocs_rviz_plugin_octomap_display_custom/src
    vigir_ocs_rviz_plugin_grasp_display_custom/src
    vigir_ocs_rviz_plugin_robot_display_custom/src
    vigir_ocs_rviz_plugin_map_display_custom/src
    vigir_ocs_rviz_plugin_frustum_display_custom/src
    vigir_ocs_rviz_plugin_interaction_tool_custom/src
    vigir_ocs_rviz_plugin_ortho_view_controller_custom/src
    vigir_ocs_rviz_plugin_overlay_display_custom/src
    ${OPENGL_INCLUDE_DIR}
  CATKIN_DEPENDS
    roscpp 
    rospy
    roslib
    std_msgs 
    geometry_msgs 
    flor_perception_msgs 
    flor_ocs_msgs
    flor_interactive_marker_server_custom
    rviz 
    pluginlib 
    class_loader
)

include_directories(
	vigir_ocs_rviz_plugin_camera_display_custom/src
	vigir_ocs_rviz_plugin_image_display_custom/src
	vigir_ocs_rviz_plugin_template_display_custom/src
	vigir_ocs_rviz_plugin_image_selection_tool_custom/src
	vigir_ocs_rviz_plugin_selection_3d_tool_custom/src
	vigir_ocs_rviz_plugin_selection_3d_display_custom/src
	vigir_ocs_rviz_plugin_bounding_object_display_custom/src
    vigir_ocs_rviz_plugin_path_display_custom/src
    vigir_ocs_rviz_plugin_octomap_display_custom/src
    vigir_ocs_rviz_plugin_grasp_display_custom/src
    vigir_ocs_rviz_plugin_robot_display_custom/src
    vigir_ocs_rviz_plugin_map_display_custom/src
    vigir_ocs_rviz_plugin_frustum_display_custom/src
    vigir_ocs_rviz_plugin_interaction_tool_custom/src
    vigir_ocs_rviz_plugin_ortho_view_controller_custom/src
    vigir_ocs_rviz_plugin_overlay_display_custom/src
	${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
				    ${OPENGL_INCLUDE_DIR}
                    ${QT_INCLUDE_DIR})
                    
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(vigir_ocs_rviz_plugin_camera_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_image_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_template_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_image_selection_tool_custom)
add_subdirectory(vigir_ocs_rviz_plugin_selection_3d_tool_custom)
add_subdirectory(vigir_ocs_rviz_plugin_selection_3d_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_bounding_object_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_path_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_octomap_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_grasp_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_robot_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_map_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_frustum_display_custom)
add_subdirectory(vigir_ocs_rviz_plugin_interaction_tool_custom)
add_subdirectory(vigir_ocs_rviz_plugin_ortho_view_controller_custom)
add_subdirectory(vigir_ocs_rviz_plugin_overlay_display_custom)

install(FILES
  camera_display_custom_plugin_description.xml
  image_display_custom_plugin_description.xml
  template_display_custom_plugin_description.xml
  image_selection_tool_custom_plugin_description.xml
  selection_3d_tool_custom_plugin_description.xml
  selection_3d_display_custom_plugin_description.xml
  bounding_object_display_custom_plugin_description.xml
  path_display_custom_plugin_description.xml
  octomap_display_custom_plugin_description.xml
  grasp_display_custom_plugin_description.xml
  robot_display_custom_plugin_description.xml
  map_display_custom_plugin_description.xml
  frustum_display_custom_plugin_description.xml
  interaction_tool_custom_plugin_description.xml
  ortho_view_controller_custom_plugin_description.xml
  overlay_display_custom_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
