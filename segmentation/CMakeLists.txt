cmake_minimum_required(VERSION 2.6)

message(STATUS ">>> motion_segmentation subdirectory")

set(SRC_FILES dnn_based_mt.cpp mask_rcnn_dnn_mt.cpp feature_based_mt.cpp score_fbmt.cpp scored_mask_rcnnmt.cpp)
set(INCLUDE_FILES motion_segmenter.h dnn_based_mt.h mask_rcnn_dnn_mt.h feature_based_mt.h score_fbmt.h scored_mask_rcnnmt.h)

set(RGBD_RTK_SEGMENTATION_SRC ${SRC_FILES})
set(RGBD_RTK_SEGMENTATION_INCLUDES ${INCLUDE_FILES}) 

add_library(rgbd_rtk_segmentation SHARED ${RGBD_RTK_SEGMENTATION_SRC})
target_link_libraries(rgbd_rtk_segmentation PUBLIC ${RGBD_RTK_COMMON} ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_segmentation PROPERTIES OUTPUT_NAME "rgbd_rtk_segmentation")
set_target_properties(rgbd_rtk_segmentation PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_segmentation
          EXPORT rgbd_rtkTargets
  		  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  		  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  		  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/rgbd_rtk_segmentation"
    	  COMPONENT dev)
install(FILES ${RGBD_RTK_SEGMENTATION_INCLUDES} DESTINATION include)
