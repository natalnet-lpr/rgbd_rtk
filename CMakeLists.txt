cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra ")#-fsanitize=address -fsanitize=leak")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3") # WITH -march=native: seg_fault. on Mac OS
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

# Offer the user the choice of overriding the installation directories
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
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
add_definitions(${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
message(STATUS "G2O Found: " ${G2O_FOUND})

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 Found: " ${EIGEN3_FOUND})

find_package(CSparse REQUIRED)
message(STATUS "CSparse Found: " ${CSPARSE_FOUND})

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR} ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR}
                    common io tracking motion_estimation visualization visual_odometry
                    quadtree stereo segmentation)
link_directories(${PCL_LIBRARY_DIRS})

LIST(APPEND G2O_LIBS
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_CLI_LIBRARY} ${G2O_SOLVER_CHOLMOD}
    ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_SOLVER_STRUCTURE_ONLY}
    ${G2O_SOLVER_EIGEN} ${G2O_TYPES_DATA} ${G2O_TYPES_ICP} ${G2O_TYPES_SBA}
    ${G2O_TYPES_SCLAM2D} ${G2O_TYPES_SIM3} ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D}
    ${CSPARSE_LIBRARY}
)

message(STATUS "G2O libs: " ${G2O_LIBS})

option(WITH_IO "Build with io module" ON)
option(WITH_QUADTREE "Build with quadtree module" ON)
option(WITH_TRACKING "Build with tracking module" ON)
option(WITH_VISUALIZATION "Build with visualization module" ON)
option(WITH_MOTION_ESTIMATION "Build with motion estimation module" ON)
option(WITH_STEREO "Build with stereo module" ON)
option(WITH_VISUAL_ODOMETRY "Build with odometry module" ON)
option(WITH_SLAM "Build with slam module" ON)

if(${OpenCV_VERSION_MAJOR} GREATER_EQUAL 3 )
	option(WITH_SEGMENTATION "Build with motion segmentation module" ON)
else()
	option(WITH_SEGMENTATION "Build without motion segmentation module" OFF)
endif()

# Needed for every other module
add_subdirectory(common)
set(RGBD_RTK_COMMON rgbd_rtk_common)
export(TARGETS rgbd_rtk_common APPEND FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")

if(WITH_IO)
	add_subdirectory(io)
	set(RGBD_RTK_IO rgbd_rtk_io)
	export(TARGETS rgbd_rtk_io APPEND FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
endif()

if(WITH_QUADTREE)
	add_subdirectory(quadtree)
	set(RGBD_RTK_QUADTREE rgbd_rtk_quadtree)
	export(TARGETS rgbd_rtk_quadtree APPEND FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
endif()

## Quadtree needed for kltqt_tracker
if(WITH_TRACKING AND WITH_QUADTREE) 
	add_subdirectory(tracking)
	set(RGBD_RTK_TRACKING rgbd_rtk_tracking)
	export(TARGETS rgbd_rtk_tracking APPEND
    FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
endif()
if(WITH_TRACKING AND (NOT WITH_QUADTREE))
	message(SEND_ERROR "Can't build Tracking without QuadTree" )
	SET(WITH_TRACKING OFF)
endif()

if(WITH_VISUALIZATION)
	add_subdirectory(visualization)
	set(RGBD_RTK_VISUALIZATION rgbd_rtk_visualization)
	export(TARGETS rgbd_rtk_visualization APPEND FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
endif()

if(WITH_MOTION_ESTIMATION)
	add_subdirectory(motion_estimation)
	set(RGBD_RTK_MOTION_ESTIMATION rgbd_rtk_motion_estimation)
	export(TARGETS rgbd_rtk_motion_estimation APPEND FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
endif()

if(WITH_STEREO)
	add_subdirectory(stereo)
	set(RGBD_RTK_STEREO rgbd_rtk_stereo)
	export(TARGETS rgbd_rtk_stereo APPEND FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
endif()

#Motion estimation needed for icp_odometry and wide_baseline_visual_odometry
##Tracking needed for optical_flow_visual_odometry
##Stereo needed for stereo_optical_flow_visual_odometry
if(WITH_VISUAL_ODOMETRY AND WITH_MOTION_ESTIMATION AND WITH_TRACKING AND WITH_STEREO)
	add_subdirectory(visual_odometry)
	set(RGBD_RTK_VISUAL_ODOMETRY rgbd_rtk_visual_odometry)
	export(TARGETS rgbd_rtk_visual_odometry APPEND
    FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
endif()
if(WITH_VISUAL_ODOMETRY AND ((NOT WITH_MOTION_ESTIMATION) OR (NOT WITH_TRACKING) OR (NOT WITH_STEREO)))
	message(SEND_ERROR "Can't build Visual Odometry without motion_estimation, tracking and stereo modules" )
	set(WITH_VISUAL_ODOMETRY OFF)
endif()

#
# Build SLAM module only if ARUCO was found
if(WITH_SLAM)
	find_package(aruco QUIET)
	if(aruco_FOUND)
	message(STATUS "aruco version: " ${aruco_VERSION})
	if(aruco_VERSION VERSION_LESS "3.0")
		message(WARNING "WARNING: aruco version >= 3.0 required")
	else()
		include_directories(slam ${aruco_INCLUDE_DIRS})
		add_subdirectory(slam)
		set(RGBD_RTK_SLAM rgbd_rtk_slam)
		export(TARGETS rgbd_rtk_slam APPEND
		FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
	endif()
	else()
		message(WARNING "ARUCO NOT FOUND")
	endif()
endif()

if(WITH_SEGMENTATION)
	add_subdirectory(segmentation)
	set(RGBD_RTK_SEGMENTATION rgbd_rtk_segmentation)
	export(TARGETS rgbd_rtk_segmentation APPEND FILE "${PROJECT_BINARY_DIR}/rgbd_rtkTargets.cmake")
endif()

add_subdirectory(applications)

export(PACKAGE rgbd_rtk)

# Create the rgbd_rtkConfig.cmake 
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")
# For the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(rgbd_rtkConfig.cmake.in "${PROJECT_BINARY_DIR}/rgbd_rtkConfig.cmake" @ONLY)
# For the install tree
set(CONF_INCLUDE_DIRS "\${rgbd_rtk_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(rgbd_rtkConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/rgbd_rtkConfig.cmake" @ONLY)


# Install the rgbd_rtkConfig.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/rgbd_rtkConfig.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT rgbd_rtkTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)
