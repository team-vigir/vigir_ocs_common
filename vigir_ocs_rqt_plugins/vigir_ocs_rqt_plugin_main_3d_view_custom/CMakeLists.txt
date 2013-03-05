qt4_wrap_cpp(MOC_SOURCES src/main_3d_view_custom.h)

include_directories(${CMAKE_CURRENT_BINARY_DIR} src )

set(LIB_NAME vigir_ocs_rqt_plugin_main_3d_view_custom)

add_library(${LIB_NAME} src/main_3d_view_custom.cpp ${MOC_SOURCES})
target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${LIB_NAME})

install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/vigir_ocs_rqt_main_3d_view_custom
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


