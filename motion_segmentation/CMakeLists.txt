cmake_minimum_required(VERSION 2.6)

message(STATUS ">>> motion_segmentation subdirectory")

set(SRC_FILES motion_segmenter.cpp dnn/dnn_motion_segmenter.cpp geometric/geometric_motion_segmenter.cpp)
set(INCLUDE_FILES motion_segmenter.h dnn/dnn_motion_segmenter.h geometric/geometric_motion_segmenter.h)

set(RGBD_RTK_MOTION_SEGMENTATION_SRC ${SRC_FILES})
set(RGBD_RTK_MOTION_SEGMENTATION_INCLUDES ${INCLUDE_FILES}) 

add_library(rgbd_rtk_motion_segmentation SHARED ${RGBD_RTK_MOTION_SEGMENTATION_SRC})
target_link_libraries(rgbd_rtk_motion_segmentation ${RGBD_RTK_COMMON} ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_motion_segmentation PROPERTIES OUTPUT_NAME "rgbd_rtk_motion_segmentation")
set_target_properties(rgbd_rtk_motion_segmentation PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_motion_segmentation
          EXPORT rgbd_rtkTargets
  		  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  		  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  		  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/rgbd_rtk_motion_segmentation"
    	  COMPONENT dev)
install(FILES ${RGBD_RTK_MOTION_SEGMENTATION_INCLUDES} DESTINATION include)
