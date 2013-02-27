cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_rqt_image_viewer_custom)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp rviz vigir_ocs_camera_viewer_custom)

catkin_package(	
	INCLUDE_DIRS ${vigir_ocs_rqt_image_viewer_custom_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp rviz vigir_ocs_camera_viewer_custom
)
catkin_python_setup()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(vigir_ocs_rqt_image_viewer_custom_SRCS
  src/vigir_ocs_rqt_image_viewer_custom/image_viewer_custom.cpp
)

set(vigir_ocs_rqt_image_viewer_custom_HDRS
  include/vigir_ocs_rqt_image_viewer_custom/image_viewer_custom.h
)

qt4_wrap_cpp(vigir_ocs_rqt_image_viewer_custom_MOCS ${vigir_ocs_rqt_image_viewer_custom_HDRS})

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${vigir_ocs_rqt_image_viewer_custom_SRCS} ${vigir_ocs_rqt_image_viewer_custom_MOCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/vigir_ocs_rqt_image_viewer_custom
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
