cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> slam subdirectory")

set(RGBD_RTK_SLAM_SRC marker_finder.cpp)
set(RGBD_RTK_SLAM_INCLUDES marker_finder.h)

add_library(rgbd_rtk_slam SHARED ${RGBD_RTK_SLAM_SRC})
target_link_libraries(rgbd_rtk_slam ${aruco_LIBS})

set_target_properties(rgbd_rtk_slam PROPERTIES OUTPUT_NAME "rgbd_rtk_slam")
set_target_properties(rgbd_rtk_slam PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_slam
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_SLAM_INCLUDES} DESTINATION include)