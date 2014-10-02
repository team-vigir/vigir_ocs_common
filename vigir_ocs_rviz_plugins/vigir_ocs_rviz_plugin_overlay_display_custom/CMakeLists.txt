# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/overlay_text_display.h src/overlay_utils.h)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(VIGIR_OVERLAY_DISPLAY_LIB_NAME vigir_ocs_rviz_plugin_overlay_display_custom)

add_library(${VIGIR_OVERLAY_DISPLAY_LIB_NAME}_core src/overlay_text_display.cpp src/overlay_utils.cpp ${MOC_SOURCES})
target_link_libraries(${VIGIR_OVERLAY_DISPLAY_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(${VIGIR_OVERLAY_DISPLAY_LIB_NAME}_core ${catkin_EXPORTED_TARGETS})

add_library(${VIGIR_OVERLAY_DISPLAY_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${VIGIR_OVERLAY_DISPLAY_LIB_NAME} ${VIGIR_OVERLAY_DISPLAY_LIB_NAME}_core ${catkin_LIBRARIES})

add_dependencies(${VIGIR_OVERLAY_DISPLAY_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${VIGIR_OVERLAY_DISPLAY_LIB_NAME} ${VIGIR_OVERLAY_DISPLAY_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

