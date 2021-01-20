cmake_minimum_required(VERSION 2.6)

message(STATUS ">>> stereo subdirectory")

set(RGBD_RTK_STEREO_SRC stereo_cloud_generator.cpp)
set(RGBD_RTK_STEREO_INCLUDES stereo_cloud_generator.h)

add_library(rgbd_rtk_stereo SHARED ${RGBD_RTK_STEREO_SRC})
target_link_libraries(rgbd_rtk_stereo ${PCL_LIBS} ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_stereo PROPERTIES OUTPUT_NAME "rgbd_rtk_stereo")
set_target_properties(rgbd_rtk_stereo PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_stereo
          EXPORT rgbd_rtkTargets
  		  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  		  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  		  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/rgbd_rtk_stereo"
    	  COMPONENT dev)
install(FILES ${RGBD_RTK_STEREO_INCLUDES} DESTINATION include)
