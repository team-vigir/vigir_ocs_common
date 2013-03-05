cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_template_nodelet)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy pluginlib std_msgs geometry_msgs vigir_ocs_msg2 nodelet)

## Declare things to be passed to other projects
catkin_package()

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/main.cpp
)

set(SOURCE_FILES_LIB
  src/template_nodelet.cpp
)

include_directories(${catkin_INCLUDE_DIRS} src)

add_library(${PROJECT_NAME} ${SOURCE_FILES_LIB})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(template_nodelet ${SOURCE_FILES})
target_link_libraries(template_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME})

install(FILES
  vigir_template_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
