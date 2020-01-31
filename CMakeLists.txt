cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#set the root path of openfabmap
#../rootfs/include/*.hpp and ../rootfs/include/*.h
#../rootfs/lib/openFABMAP.a

list(APPEND CMAKE_FIND_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../rootfs")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-march=native -mno-avx -O3") # WITH -march=native: seg_fault. on Mac OS
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

SET(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

message(STATUS "Project source directory: " ${PROJECT_SOURCE_DIR})
message(STATUS "Build directory: " ${PROJECT_BINARY_DIR})

find_package(OpenCV REQUIRED)
find_package(openFABMAP REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message("Found OpenMP")
endif(OPENMP_FOUND)


if(OpenCV_FOUND)
	message(STATUS "OpenCV version: " ${OpenCV_VERSION})
	if(OpenCV_VERSION VERSION_LESS "3.0.0")
  		message(FATAL_ERROR "ERROR: OpenCV version >= 3.0 required")
	endif()
else()
	message(FATAL_ERROR "Error: OpenCV was not found in your system")
endif()

find_package(PCL 1.8 REQUIRED)
if(PCL_FOUND)
	message(STATUS "PCL version: " ${PCL_VERSION})
else()
	message(FATAL_ERROR "Error: PCL was not found in your system")
endif()





#set(OpenCV_INCLUDE_DIRS "/home/bruno/Lib/opencv-3.4.1/local/include")
# Force OpenCV versions if ROS is installed
message("test " ${OPEN_FAB_MAP_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OPEN_FAB_MAP_INCLUDE_DIRS} common io tracking motion_estimation
                    visualization visual_odometry visual_memory)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OPEN_FAB_MAP_LIBRARIES})
#add_definitions(${PCL_DEFINITIONS}) #apparently not needed
add_subdirectory(io)
set(RGBD_RTK_IO rgbd_rtk_io)
add_subdirectory(tracking)
set(RGBD_RTK_TRACKING rgbd_rtk_tracking)
add_subdirectory(common)
set(RGBD_RTK_COMMON rgbd_rtk_common)
add_subdirectory(visualization)
set(RGBD_RTK_VISUALIZATION rgbd_rtk_visualization)
add_subdirectory(motion_estimation)
set(RGBD_RTK_MOTION_ESTIMATION rgbd_rtk_motion_estimation)
add_subdirectory(visual_odometry)
set(RGBD_RTK_VISUAL_ODOMETRY rgbd_rtk_visual_odometry)
add_subdirectory(visual_memory)
set(RGBD_RTK_VISUAL_MEMORY rgbd_rtk_visual_memory)

# Build SLAM module only if ARUCO was found
#find_package(aruco 2.0 QUIET)
#if(aruco_FOUND)
#	message(STATUS "Aruco found")
#	message(STATUS "Aruco INCLUDES: " ${aruco_INCLUDE_DIRS})
#	message(STATUS "Aruco LIBS: " ${aruco_LIBS})
#	include_directories(slam)
#	add_subdirectory(slam)
#	set(RGBD_RTK_SLAM rgbd_rtk_slam)
#else()
#	message(WARNING "Aruco was not found in your system. SLAM module will not be built.")
#endif()

add_subdirectory(applications)

