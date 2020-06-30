cmake_minimum_required(VERSION 2.6)

message(STATUS ">>> visualization subdirectory")

set(RGBD_RTK_VISUALIZATION_SRC reconstruction_visualizer.cpp)
set(RGBD_RTK_VISUALIZATION_INCLUDES reconstruction_visualizer.h)

add_library(rgbd_rtk_visualization SHARED ${RGBD_RTK_VISUALIZATION_SRC})
target_link_libraries(rgbd_rtk_visualization ${PCL_LIBRARIES})

set_target_properties(rgbd_rtk_visualization PROPERTIES OUTPUT_NAME "rgbd_rtk_visualization")
set_target_properties(rgbd_rtk_visualization PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_visualization
          EXPORT rgbd_rtkTargets
  		  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  		  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  		  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/rgbd_rtk_visualization"
    	  COMPONENT dev)

install(FILES ${RGBD_RTK_VISUALIZATION_INCLUDES} DESTINATION include)
