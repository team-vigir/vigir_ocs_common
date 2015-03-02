cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_main_view)

## Find catkin and any catkin packages on which
## this package depends at build time

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs rviz vigir_ocs_base_3d_view vigir_ocs_perspective_view vigir_ocs_ortho_view  vigir_ocs_status_bar vigir_ocs_grasp_widget vigir_ocs_notification_system )

## Declare things to be passed to other projects
catkin_package(	
        INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME} 

        CATKIN_DEPENDS roscpp rospy std_msgs rviz vigir_ocs_base_3d_view vigir_ocs_perspective_view vigir_ocs_ortho_view  vigir_ocs_status_bar vigir_ocs_grasp_widget vigir_ocs_notification_system

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
  src/ui/main_view_widget.h
  src/ui/footstep_config.h
  src/ui/main_context_menu.h
)

## Here we add QtCreator ui files
qt4_wrap_ui(UIC_FILES
  src/ui/main_view_widget.ui
  src/ui/footstep_config.ui
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.

set(SOURCE_FILES
  src/ui/main_view_widget.cpp
  src/ui/footstep_config.cpp
  src/ui/main_context_menu.cpp
  src/main.cpp
  ${MOC_FILES}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} src)

add_executable(main_view ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(main_view ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

install( TARGETS ${PROJECT_NAME} main_view
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
