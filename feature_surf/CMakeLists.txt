cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> feature surf subdirectory")

set(RGBD_RTK_SURF_DETECTOR_SRC surf_detector.cpp)
set(RGBD_RTK_SURF_DETECTOR_INCLUDES  surf_detector.h )

add_library(rgbd_rtk_surf_detector SHARED ${RGBD_RTK_SURF_DETECTOR_SRC})
target_link_libraries(rgbd_rtk_surf_detector ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_surf_detector PROPERTIES OUTPUT_NAME "rgbd_rtk_surf_detector")
set_target_properties(rgbd_rtk_surf_detector PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_surf_detector
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_SURF_DETECTOR_INCLUDES} DESTINATION include)
