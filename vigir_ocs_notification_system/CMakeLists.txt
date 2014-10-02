cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_notification_system)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(PCL REQUIRED QUIET COMPONENTS common)

find_package(catkin REQUIRED COMPONENTS pcl_ros roscpp rospy)


## Declare things to be passed to other projects
catkin_package(	
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME} 
    CATKIN_DEPENDS pcl_ros roscpp rospy flor_ocs_msgs

)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES

  src/notification_system.cpp
  ${MOC_FILES}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} src)

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

