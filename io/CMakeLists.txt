cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> io subdirectory")

set(RGBD_RTK_IO_SRC sequence_loader.cpp rgbd_loader.cpp kitti_stereo_loader.cpp velodyne_loader.cpp)
set(RGBD_RTK_IO_INCLUDES sequence_loader.h rgbd_loader.h kitti_stereo_loader.h velodyne_loader.h)

add_library(rgbd_rtk_io SHARED ${RGBD_RTK_IO_SRC})
target_link_libraries(rgbd_rtk_io ${OpenCV_LIBS} ${PCL_LIBRARIES})

set_target_properties(rgbd_rtk_io PROPERTIES OUTPUT_NAME "rgbd_rtk_io")
set_target_properties(rgbd_rtk_io PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_io
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_IO_INCLUDES} DESTINATION include)
