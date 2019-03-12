cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> common subdirectory")

set(RGBD_RTK_COMMON_SRC geometry.cpp kitti_cloud_generator.cpp)
set(RGBD_RTK_COMMON_INCLUDES common_types.h geometry.h kitti_cloud_generator.h)

add_library(rgbd_rtk_common SHARED ${RGBD_RTK_COMMON_SRC})
target_link_libraries(rgbd_rtk_common ${PCL_LIBS} ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_common PROPERTIES OUTPUT_NAME "rgbd_rtk_common")
set_target_properties(rgbd_rtk_common PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_common
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_COMMON_INCLUDES} DESTINATION include)
