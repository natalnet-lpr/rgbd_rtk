cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> visual memory subdirectory")


add_subdirectory(fabMap)


set(RGBD_RTK_VISUAL_MEMORY_SRC visual_memory.cpp fabMap/fabmap_memory.cpp)

set(RGBD_RTK_VISUAL_MEMORY_INCLUDES visual_memory.h fabMap/fabmap_memory.h)

add_library(rgbd_rtk_visual_memory SHARED ${RGBD_RTK_VISUAL_MEMORY_SRC})

target_link_libraries(rgbd_rtk_visual_memory ${OPEN_FAB_MAP_LIBRARIES} ${OpenCV_LIBS} ${RGBD_RTK_TRACKING} )
include_directories(rgbd_rtk_visual_memory ${OPEN_FAB_MAP_INCLUDE_DIRS})
message("enock " ${OPEN_FAB_MAP_INCLUDE_DIRS})

set_target_properties(rgbd_rtk_visual_memory PROPERTIES OUTPUT_NAME "rgbd_rtk_visual_memory")
set_target_properties(rgbd_rtk_visual_memory PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_visual_memory
        LIBRARY DESTINATION lib)

install(FILES ${RGBD_RTK_VISUAL_MEMORY_INCLUDES} DESTINATION include)
