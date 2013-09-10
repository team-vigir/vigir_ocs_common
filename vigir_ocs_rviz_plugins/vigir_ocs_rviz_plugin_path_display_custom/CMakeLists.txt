# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/path_display_custom.h)
find_package(catkin REQUIRED COMPONENTS flor_perception_msgs )

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(VIGIR_PATH_LIB_NAME vigir_ocs_rviz_plugin_path_display_custom)

add_library(${VIGIR_PATH_LIB_NAME}_core src/path_display_custom.cpp	${MOC_SOURCES})
target_link_libraries(${VIGIR_PATH_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OPENGL_LIBRARIES})

add_dependencies(${VIGIR_PATH_LIB_NAME}_core ${catkin_EXPORTED_TARGETS})

add_library(${VIGIR_PATH_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${VIGIR_PATH_LIB_NAME} ${VIGIR_PATH_LIB_NAME}_core ${catkin_LIBRARIES} ${OPENGL_LIBRARIES})

add_dependencies(${VIGIR_PATH_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${VIGIR_PATH_LIB_NAME} ${VIGIR_PATH_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

