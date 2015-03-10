cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_template_nodelet)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf tf_conversions pluginlib std_msgs geometry_msgs moveit_core moveit_ros_planning shape_msgs flor_ocs_msgs flor_grasp_msgs nodelet cmake_modules)

find_package(TinyXML REQUIRED)

## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME} 
        CATKIN_DEPENDS roscpp rospy tf tf_conversions pluginlib std_msgs geometry_msgs moveit_core moveit_ros_planning shape_msgs flor_ocs_msgs flor_grasp_msgs nodelet
        DEPENDS TinyXML
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/main.cpp
)

set(SOURCE_FILES_LIB
  src/template_nodelet.cpp
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} src ${TinyXML_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${SOURCE_FILES_LIB})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(template_nodelet ${SOURCE_FILES})
target_link_libraries(template_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(template_nodelet ${catkin_EXPORTED_TARGETS})

install(FILES
  vigir_template_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install( TARGETS
    template_nodelet
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
