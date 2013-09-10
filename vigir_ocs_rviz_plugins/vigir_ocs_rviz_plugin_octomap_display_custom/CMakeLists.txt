# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/octomap_display_custom.h src/marker_display_custom.h)

include_directories(${CMAKE_CURRENT_BINARY_DIR} )# /opt/vigir/vigir_perception/vigir_perception_msgs/msg_gen/cpp/include)

set(VIGIR_OCTOMAP_LIB_NAME vigir_ocs_rviz_plugin_octomap_display_custom)

add_library(${VIGIR_OCTOMAP_LIB_NAME}_core src/octomap_display_custom.cpp src/marker_display_custom.cpp src/points_marker_custom.cpp src/point_cloud_custom.cpp src/marker_base_custom.cpp src/marker_selection_handler_custom.cpp ${MOC_SOURCES})
target_link_libraries(${VIGIR_OCTOMAP_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(${VIGIR_OCTOMAP_LIB_NAME}_core ${catkin_EXPORTED_TARGETS})

add_library(${VIGIR_OCTOMAP_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${VIGIR_OCTOMAP_LIB_NAME} ${VIGIR_OCTOMAP_LIB_NAME}_core ${catkin_LIBRARIES})

add_dependencies(${VIGIR_OCTOMAP_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${VIGIR_OCTOMAP_LIB_NAME} ${VIGIR_OCTOMAP_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

