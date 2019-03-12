cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> visual_odometry subdirectory")

set(RGBD_RTK_VISUAL_ODOMETRY_SRC optical_flow_visual_odometry.cpp icp_odometry.cpp stereo_optical_flow_visual_odometry.cpp)
set(RGBD_RTK_VISUAL_ODOMETRY_INCLUDES optical_flow_visual_odometry.h icp_odometry.h stereo_optical_flow_visual_odometry.h)
add_library(rgbd_rtk_visual_odometry SHARED ${RGBD_RTK_VISUAL_ODOMETRY_SRC})
target_link_libraries(rgbd_rtk_visual_odometry ${RGBD_RTK_COMMON} ${RGBD_RTK_TRACKING} ${RGBD_RTK_MOTION_ESTIMATION} ${PCL_LIBRARIES})

set_target_properties(rgbd_rtk_visual_odometry PROPERTIES OUTPUT_NAME "rgbd_rtk_visual_odometry")
set_target_properties(rgbd_rtk_visual_odometry PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_visual_odometry
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_VISUAL_ODOMETRY_INCLUDES} DESTINATION include)
