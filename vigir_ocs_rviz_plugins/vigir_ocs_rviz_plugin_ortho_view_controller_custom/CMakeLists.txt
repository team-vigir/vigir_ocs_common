# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/ortho_view_controller_custom.h)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME vigir_ocs_rviz_plugin_ortho_view_controller_custom)

add_library(${VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME}_core src/ortho_view_controller_custom.cpp	${MOC_SOURCES})
target_link_libraries(${VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(${VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME}_core ${catkin_EXPORTED_TARGETS})

add_library(${VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME} ${VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME}_core ${catkin_LIBRARIES})

add_dependencies(${VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME} ${VIGIR_ORTHO_VIEW_CONTROLLER_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

