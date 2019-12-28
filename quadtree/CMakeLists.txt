cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> quadtree subdirectory")

set(RGBD_RTK_QUADTREE_SRC quad_tree.cpp)
set(RGBD_RTK_QUADTREE_INCLUDES quad_tree.h)

add_library(rgbd_rtk_quadtree SHARED ${RGBD_RTK_QUADTREE_SRC})
target_link_libraries(rgbd_rtk_quadtree ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_quadtree PROPERTIES OUTPUT_NAME "rgbd_rtk_quadtree")
set_target_properties(rgbd_rtk_quadtree PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_quadtree
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_QUADTREE_INCLUDES} DESTINATION include)
