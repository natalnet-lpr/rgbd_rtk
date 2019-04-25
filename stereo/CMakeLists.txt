cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> stereo subdirectory")

set(RGBD_RTK_STEREO_SRC stereo_cloud_generator.cpp)
set(RGBD_RTK_STEREO_INCLUDES stereo_cloud_generator.h)

add_library(rgbd_rtk_stereo SHARED ${RGBD_RTK_STEREO_SRC})
target_link_libraries(rgbd_rtk_stereo ${PCL_LIBS} ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_stereo PROPERTIES OUTPUT_NAME "rgbd_rtk_stereo")
set_target_properties(rgbd_rtk_stereo PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_stereo
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_STEREO_INCLUDES} DESTINATION include)
