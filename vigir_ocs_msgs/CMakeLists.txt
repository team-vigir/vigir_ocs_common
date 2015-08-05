cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs message_generation vigir_grasp_msgs)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
    FILES  
    OCSSynchronize.msg
    OCSControlMode.msg
    OCSTemplateAdd.msg
    OCSTemplateRemove.msg
    OCSTemplateList.msg
    OCSTemplateUpdate.msg
    OCSWaypointAdd.msg
    OCSWaypointRemove.msg
    OCSWaypointUpdate.msg
    OCSFootstepList.msg
    OCSFootstepUpdate.msg
    OCSFootstepPlanGoal.msg
    OCSFootstepPlanGoalUpdate.msg
    OCSFootstepPlanParameters.msg
    OCSFootstepPlanRequest.msg
    OCSFootstepPlanUpdate.msg
    OCSFootstepStatus.msg
    OCSFootstepSyncStatus.msg
    OCSObjectSelection.msg
    OCSRobotStability.msg
    OCSRobotStatus.msg
    OCSReference.msg
    OCSLinkColor.msg    
    OCSRaycastRequest.msg
    OCSImageAdd.msg
    OCSImageList.msg
    OCSInteractiveMarkerAdd.msg
    OCSInteractiveMarkerUpdate.msg
    OCSSelectedObjectUpdate.msg
    OCSKeyEvent.msg
    TwoPoint.msg
    OCSAugmentRegions.msg
    OCSInverseReachability.msg
    OCSHotkeyRelay.msg
    OCSCameraTransform.msg
    OCSFootstepParamSetList.msg
    OCSOverlayText.msg
    OCSLogging.msg
    OCSBehaviorGoal.msg
    OCSGraspSync.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs 
   vigir_grasp_msgs
 )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vigir_ocs_msgs
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_runtime vigir_grasp_msgs
  #DEPENDS
)

###########
## Build ##
###########
## Specify additional locations of header files
set( HEADERS
    include/vigir_ocs_msgs/RobotStatusCodes.h
    include/vigir_ocs_msgs/WindowCodes.h
)

set( SOURCE
    src/RobotStatusCodes.cpp
)
## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCE})
#target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Declare a cpp library
# add_library(vigir_ocs_msgs
#   src/${PROJECT_NAME}/vigir_ocs_msgs.cpp
# )

## Declare a cpp executable
# add_executable(vigir_ocs_msgs_node src/vigir_ocs_msgs_node.cpp)

## Add dependencies to the executable
# add_dependencies(vigir_ocs_msgs_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
# target_link_libraries(vigir_ocs_msgs_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS vigir_ocs_msgs vigir_ocs_msgs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vigir_ocs_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
