cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_interactive_marker_server_nodelet)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf pluginlib std_msgs geometry_msgs flor_ocs_msgs flor_grasp_msgs flor_interactive_marker_server_custom nodelet)

## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME} 
        CATKIN_DEPENDS roscpp rospy tf pluginlib std_msgs geometry_msgs flor_ocs_msgs flor_grasp_msgs flor_interactive_marker_server_custom nodelet
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/main.cpp
)

set(SOURCE_FILES_LIB
  src/interactive_marker_server_nodelet.cpp
)

include_directories(${catkin_INCLUDE_DIRS} src)

add_library(${PROJECT_NAME} ${SOURCE_FILES_LIB})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(interactive_marker_server_nodelet ${SOURCE_FILES})
target_link_libraries(interactive_marker_server_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(interactive_marker_server_nodelet ${catkin_EXPORTED_TARGETS})

install(FILES
  vigir_interactive_marker_server_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
