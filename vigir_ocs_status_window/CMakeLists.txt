cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_status_window)

#set(ROS_BUILD_TYPE RelWithDebInfo)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs rviz vigir_ocs_msgs vigir_control_msgs vigir_ocs_robot_state_manager)# vigir_perception_msgs)

## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rospy std_msgs rviz vigir_ocs_msgs vigir_control_msgs vigir_ocs_robot_state_manager
)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_JOINT_WIDGET
  src/jointList.h
)
qt4_wrap_cpp(MOC_ROBOT_WIDGET
  src/robotStatus.h
)

qt4_wrap_cpp(MOC_FILES
  src/status_window.h
  src/jointList.h
  src/robotStatus.h
)


## Here we add QtCreator ui files
qt4_wrap_ui(UIC_FILES
  src/status_window.ui
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_JOINT_WIDGET
  src/jointList.cpp
  src/jointMain.cpp
  ${MOC_JOINT_WIDGET}
)

set(SOURCE_ROBOT_WIDGET
  src/robotStatus.cpp
  src/robotMain.cpp
  ${MOC_ROBOT_WIDGET}
)

set(SOURCE_FILES
  src/status_window.cpp
  src/jointList.cpp
  src/robotStatus.cpp
  src/main.cpp
  ${MOC_FILES}
  ${UIC_FILES}
)
include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} src)

add_executable(status_window ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(status_window ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(status_window ${catkin_EXPORTED_TARGETS})

add_executable(joint_list ${SOURCE_JOINT_WIDGET})
target_link_libraries(joint_list ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(joint_list ${catkin_EXPORTED_TARGETS})

add_executable(robot_status_widget ${SOURCE_ROBOT_WIDGET})
target_link_libraries(robot_status_widget ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(robot_status_widget ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install( TARGETS status_window joint_list robot_status_widget ${PROJECT_NAME} 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
