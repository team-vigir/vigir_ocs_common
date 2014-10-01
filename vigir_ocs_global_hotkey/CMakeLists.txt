cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_global_hotkey)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy pluginlib std_msgs flor_ocs_msgs)

## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME} 
        CATKIN_DEPENDS roscpp rospy pluginlib std_msgs flor_ocs_msgs
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/global_hotkey.cpp
  src/main.cpp
)

include_directories(${catkin_INCLUDE_DIRS} src)

add_executable(global_hotkey ${SOURCE_FILES})
target_link_libraries(global_hotkey ${catkin_LIBRARIES} X11)

add_dependencies(global_hotkey ${catkin_EXPORTED_TARGETS})

install( TARGETS
    global_hotkey
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

