cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> flexible feature detector subdirectory")

set(RGBD_RTK_FLEXIBLE_FEATURE_DETECTOR_SRC flexible_feature_detector.cpp)
set(RGBD_RTK_FLEXIBLE_FEATURE_DETECTOR_INCLUDES flexible_feature_detector.h feature_surf.h)

add_library(rgbd_rtk_flexible_feature_detector SHARED ${RGBD_RTK_FLEXIBLE_FEATURE_DETECTOR_SRC})
target_link_libraries(rgbd_rtk_flexible_feature_detector ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_flexible_feature_detector PROPERTIES OUTPUT_NAME "rgbd_rtk_flexible_feature_detector")
set_target_properties(rgbd_rtk_flexible_feature_detector PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_flexible_feature_detector
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_FLEXIBLE_FEATURE_DETECTOR_INCLUDES} DESTINATION include)
