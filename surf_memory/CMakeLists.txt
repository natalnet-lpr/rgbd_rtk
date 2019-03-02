cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> feature surf subdirectory")

set(RGBD_RTK_SURF_MEMORY_SRC surf_BOW.cpp)
set(RGBD_RTK_SURF_MEMORY_INCLUDES surf_h.BOW)

add_library(rgbd_rtk_surf_memory SHARED ${RGBD_RTK_SURF_MEMORY_SRC})
target_link_libraries(rgbd_rtk_surf_memory ${OpenCV_LIBS})

set_target_properties(rgbd_rtk_surf_memory PROPERTIES OUTPUT_NAME "rgbd_rtk_surf_memory")
set_target_properties(rgbd_rtk_surf_memory PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_surf_memory
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_SURF_MEMORY_INCLUDES} DESTINATION include)
