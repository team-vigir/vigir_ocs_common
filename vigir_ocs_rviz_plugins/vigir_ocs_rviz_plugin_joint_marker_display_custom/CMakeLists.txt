# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/joint_marker_display_custom.h src/joint_visual_display_custom.h)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(VIGIR_BOUNDING_OBJECT_LIB_NAME vigir_ocs_rviz_plugin_joint_marker_display_custom)

add_library(${VIGIR_BOUNDING_OBJECT_LIB_NAME}_core src/joint_marker_display_custom.cpp src/joint_visual_display_custom.cpp ${MOC_SOURCES})
target_link_libraries(${VIGIR_BOUNDING_OBJECT_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(${VIGIR_BOUNDING_OBJECT_LIB_NAME}_core ${catkin_EXPORTED_TARGETS})

add_library(${VIGIR_BOUNDING_OBJECT_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${VIGIR_BOUNDING_OBJECT_LIB_NAME} ${VIGIR_BOUNDING_OBJECT_LIB_NAME}_core ${catkin_LIBRARIES})

add_dependencies(${VIGIR_BOUNDING_OBJECT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${VIGIR_BOUNDING_OBJECT_LIB_NAME} ${VIGIR_BOUNDING_OBJECT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

