cmake_minimum_required(VERSION 2.6)

message(STATUS ">>> tracking subdirectory")

set(RGBD_RTK_TRACKING_SRC feature_tracker.cpp klt_tracker.cpp klttw_tracker.cpp klttcw_tracker.cpp kltatcw_tracker.cpp kltqt_tracker.cpp)
set(RGBD_RTK_TRACKING_INCLUDES feature_tracker.h klt_tracker.h klttw_tracker.h klttcw_tracker.h kltatcw_tracker.h kltqt_tracker.h)

add_library(rgbd_rtk_tracking SHARED ${RGBD_RTK_TRACKING_SRC})
target_link_libraries(rgbd_rtk_tracking ${RGBD_RTK_COMMON} ${RGBD_RTK_QUADTREE} ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_tracking PROPERTIES OUTPUT_NAME "rgbd_rtk_tracking")
set_target_properties(rgbd_rtk_tracking PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_tracking
          EXPORT rgbd_rtkTargets
  		  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  		  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  		  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/rgbd_rtk_tracking"
    	  COMPONENT dev)

install(FILES ${RGBD_RTK_TRACKING_INCLUDES} DESTINATION include)
