cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-march=native -mno-avx -O3") # WITH -march=native: seg_fault. on Mac OS
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(DEF_INSTALL_CMAKE_DIR CMake)
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")


# Make relative paths absolute (needed to create rgbd_rtkConfig.cmake)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# set up include-directories
include_directories(
  "${PROJECT_SOURCE_DIR}"   
  "${PROJECT_BINARY_DIR}")  

message(STATUS "Project source directory: " ${PROJECT_SOURCE_DIR})
message(STATUS "Build directory: " ${PROJECT_BINARY_DIR})
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})


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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(G2O_ROOT /usr/local/include/g2o)

find_package(G2O REQUIRED)
message(STATUS "G2O Found: " ${g2o_FOUND})

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} common io quadtree tracking visualization motion_estimation stereo visual_odometry)
link_directories(${PCL_LIBRARY_DIRS})

add_subdirectory(common)
set(RGBD_RTK_COMMON rgbd_rtk_common)
add_subdirectory(io)
set(RGBD_RTK_IO rgbd_rtk_io)
add_subdirectory(quadtree)
set(RGBD_RTK_QUADTREE rgbd_rtk_quadtree)
add_subdirectory(tracking)
set(RGBD_RTK_TRACKING rgbd_rtk_tracking)
add_subdirectory(visualization)
set(RGBD_RTK_VISUALIZATION rgbd_rtk_visualization)
add_subdirectory(motion_estimation)
set(RGBD_RTK_MOTION_ESTIMATION rgbd_rtk_motion_estimation)
add_subdirectory(stereo)
set(RGBD_RTK_STEREO rgbd_rtk_stereo)
add_subdirectory(visual_odometry)
set(RGBD_RTK_VISUAL_ODOMETRY rgbd_rtk_visual_odometry)

# Export the package for use from the build tree (registers the build-tree with a global CMake-registry)
export(TARGETS rgbd_rtk_common rgbd_rtk_io rgbd_rtk_quadtree rgbd_rtk_tracking  rgbd_rtk_visualization rgbd_rtk_motion_estimation rgbd_rtk_stereo rgbd_rtk_visual_odometry
FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")

# Build SLAM module only if ARUCO was found
find_package(aruco QUIET)
if(aruco_FOUND)
  message(STATUS "aruco version: " ${aruco_VERSION})
  if(aruco_VERSION VERSION_LESS "2.0")
    message(WARNING "WARNING: aruco version >= 2.0 required")
  else()
    include_directories(slam)
    add_subdirectory(slam)
    set(RGBD_RTK_SLAM rgbd_rtk_slam)

    export(TARGETS rgbd_rtk_slam
    FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
  endif()
endif()

export(PACKAGE rgbd_rtk)

add_subdirectory(applications)

# Create the rgbd_rtkConfig.cmake 
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")
# For the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(rgbd_rtkConfig.cmake.in
  "${PROJECT_BINARY_DIR}/rgbd_rtkConfig.cmake" @ONLY)
# For the install tree
set(CONF_INCLUDE_DIRS "\${rgbd_rtk_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(rgbd_rtkConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/rgbd_rtkConfig.cmake" @ONLY)


# Install the rgbd_rtkConfig.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/rgbd_rtkConfig.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT rgbd_rtkTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)
