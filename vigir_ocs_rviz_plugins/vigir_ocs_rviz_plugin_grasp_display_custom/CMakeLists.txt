# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/grasp_display_custom.h)
find_package(catkin REQUIRED COMPONENTS flor_perception_msgs)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} )

set(VIGIR_IMAGE_CUSTOM_LIB_NAME vigir_ocs_rviz_plugin_grasp_display_custom)

add_library(${VIGIR_IMAGE_CUSTOM_LIB_NAME}_core src/grasp_display_custom.cpp	${MOC_SOURCES})
target_link_libraries(${VIGIR_IMAGE_CUSTOM_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_library(${VIGIR_IMAGE_CUSTOM_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${VIGIR_IMAGE_CUSTOM_LIB_NAME} ${VIGIR_IMAGE_CUSTOM_LIB_NAME}_core ${catkin_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${VIGIR_IMAGE_CUSTOM_LIB_NAME} ${VIGIR_IMAGE_CUSTOM_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

