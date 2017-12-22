cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS "Project source directory: " ${PROJECT_SOURCE_DIR})
message(STATUS "Build directory: " ${PROJECT_BINARY_DIR})

find_package(OpenCV REQUIRED)
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

# Force OpenCVPCL versions if ROS is installed
#set(OpenCV_INCLUDE_DIRS "/home/bruno/Lib/opencv-3.3.1/local/include")

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} common io tracking motion_estimation
                    visualization visual_odometry)
link_directories(${PCL_LIBRARY_DIRS})
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
add_subdirectory(applications)