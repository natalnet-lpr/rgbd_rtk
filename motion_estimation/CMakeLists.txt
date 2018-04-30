cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> motion_estimation subdirectory")

set(RGBD_RTK_MOTION_ESTIMATION_SRC motion_estimator_ransac.cpp motion_estimator_icp.cpp)
set(RGBD_RTK_MOTION_ESTIMATION_INCLUDES motion_estimator_ransac.h motion_estimator_icp.h)

add_library(rgbd_rtk_motion_estimation SHARED ${RGBD_RTK_MOTION_ESTIMATION_SRC})
target_link_libraries(rgbd_rtk_motion_estimation ${RGBD_RTK_COMMON} ${PCL_LIBRARIES})

set_target_properties(rgbd_rtk_motion_estimation PROPERTIES OUTPUT_NAME "rgbd_rtk_motion_estimation")
set_target_properties(rgbd_rtk_motion_estimation PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_motion_estimation
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_MOTION_ESTIMATION_INCLUDES} DESTINATION include)