cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> tracking subdirectory")

set(RGBD_RTK_TRACKING_SRC klt_tracker.cpp)
set(RGBD_RTK_TRACKING_INCLUDES klt_tracker.h)

add_library(rgbd_rtk_tracking SHARED ${RGBD_RTK_TRACKING_SRC})
target_link_libraries(rgbd_rtk_tracking ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_tracking PROPERTIES OUTPUT_NAME "rgbd_rtk_tracking")
set_target_properties(rgbd_rtk_tracking PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_tracking
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_TRACKING_INCLUDES} DESTINATION include)