cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> visualization subdirectory")

set(RGBD_RTK_VISUALIZATION_SRC reconstruction_visualizer.cpp)
set(RGBD_RTK_VISUALIZATION_INCLUDES reconstruction_visualizer.h)

add_library(rgbd_rtk_visualization SHARED ${RGBD_RTK_VISUALIZATION_SRC})
target_link_libraries(rgbd_rtk_visualization ${PCL_LIBRARIES})

set_target_properties(rgbd_rtk_visualization PROPERTIES OUTPUT_NAME "rgbd_rtk_visualization")
set_target_properties(rgbd_rtk_visualization PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_visualization
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_VISUALIZATION_INCLUDES} DESTINATION include)