cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_rqt_plugins)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin 
  REQUIRED 
  COMPONENTS 
    rqt_gui 
    rqt_gui_cpp 
    rviz 
    vigir_ocs_camera_viewer_custom
    vigir_ocs_main_3d_view
    vigir_ocs_map_view
    vigir_ocs_status_window)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## Define the package so catkin can find it
catkin_package(
  LIBRARIES
    vigir_ocs_rqt_plugin_camera_viewer_custom
    vigir_ocs_rqt_plugin_main_3d_view_custom
    vigir_ocs_rqt_plugin_map_view_custom
    vigir_ocs_rqt_plugin_status_view_custom
  INCLUDE_DIRS
    vigir_ocs_rqt_plugin_camera_viewer_custom/src
    vigir_ocs_rqt_plugin_main_3d_view_custom/src
    vigir_ocs_rqt_plugin_map_view_custom/src
    vigir_ocs_rqt_plugin_status_view_custom/src
  CATKIN_DEPENDS 
    rqt_gui
    rqt_gui_cpp
    rviz
    vigir_ocs_camera_viewer_custom
    vigir_ocs_main_3d_view
    vigir_ocs_map_view
    vigir_ocs_status_window)

## Setup the plugin
catkin_python_setup()

include_directories(
    vigir_ocs_rqt_plugin_camera_viewer_custom/src
    vigir_ocs_rqt_plugin_main_3d_view_custom/src
    vigir_ocs_rqt_plugin_map_view_custom/src
    vigir_ocs_rqt_plugin_status_view_custom/src
	${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${QT_INCLUDE_DIR})
                    
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(vigir_ocs_rqt_plugin_camera_viewer_custom)
add_subdirectory(vigir_ocs_rqt_plugin_main_3d_view_custom)
add_subdirectory(vigir_ocs_rqt_plugin_map_view_custom)
add_subdirectory(vigir_ocs_rqt_plugin_status_view_custom)

install(FILES
  camera_viewer_custom_plugin_description.xml
  main_3d_view_custom_plugin_description.xml
  map_view_custom_plugin_description.xml
  status_view_custom_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
