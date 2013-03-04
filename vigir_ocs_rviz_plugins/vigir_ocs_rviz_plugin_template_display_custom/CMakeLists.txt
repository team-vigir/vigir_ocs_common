# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/template_display_custom.h)

include_directories(${CMAKE_CURRENT_BINARY_DIR} )# /opt/vigir/vigir_perception/vigir_perception_msgs/msg_gen/cpp/include)

set(VIGIR_IMAGE_CUSTOM_LIB_NAME vigir_ocs_rviz_plugin_template_display_custom)

add_library(${VIGIR_IMAGE_CUSTOM_LIB_NAME}_core src/template_display_custom.cpp	${MOC_SOURCES})
target_link_libraries(${VIGIR_IMAGE_CUSTOM_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_library(${VIGIR_IMAGE_CUSTOM_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${VIGIR_IMAGE_CUSTOM_LIB_NAME} ${VIGIR_IMAGE_CUSTOM_LIB_NAME}_core ${catkin_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${VIGIR_IMAGE_CUSTOM_LIB_NAME} ${VIGIR_IMAGE_CUSTOM_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

