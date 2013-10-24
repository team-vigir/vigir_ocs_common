# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/robot_link_custom.h src/tf_link_updater_custom.h src/robot_custom.h src/robot_display_custom.h)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(VIGIR_ROBOT_DISPLAY_LIB_NAME vigir_ocs_rviz_plugin_robot_display_custom)

add_library(${VIGIR_ROBOT_DISPLAY_LIB_NAME}_core src/robot_link_custom.cpp src/tf_link_updater_custom.cpp src/robot_custom.cpp src/robot_display_custom.cpp ${MOC_SOURCES})
target_link_libraries(${VIGIR_ROBOT_DISPLAY_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(${VIGIR_ROBOT_DISPLAY_LIB_NAME}_core ${catkin_EXPORTED_TARGETS})

add_library(${VIGIR_ROBOT_DISPLAY_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${VIGIR_ROBOT_DISPLAY_LIB_NAME} ${VIGIR_ROBOT_DISPLAY_LIB_NAME}_core ${catkin_LIBRARIES})

add_dependencies(${VIGIR_ROBOT_DISPLAY_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${VIGIR_ROBOT_DISPLAY_LIB_NAME} ${VIGIR_ROBOT_DISPLAY_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

