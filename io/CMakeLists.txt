cmake_minimum_required(VERSION 2.6)

message(STATUS ">>> io subdirectory")

set(RGBD_RTK_IO_SRC sequence_loader.cpp rgbd_loader.cpp config_loader.cpp 
                    kitti_stereo_loader.cpp kitti_velodyne_loader.cpp)
set(RGBD_RTK_IO_INCLUDES sequence_loader.h rgbd_loader.h config_loader.h
                         kitti_stereo_loader.h kitti_velodyne_loader.h)

add_library(rgbd_rtk_io SHARED ${RGBD_RTK_IO_SRC})
target_link_libraries(rgbd_rtk_io ${RGBD_RTK_COMMON} ${OpenCV_LIBS} ${PCL_LIBRARIES})

set_target_properties(rgbd_rtk_io PROPERTIES OUTPUT_NAME "rgbd_rtk_io")
set_target_properties(rgbd_rtk_io PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_io
          EXPORT rgbd_rtkTargets
  		  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  		  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  		  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/rgbd_rtk_io"
    	  COMPONENT dev)

install(FILES ${RGBD_RTK_IO_INCLUDES} DESTINATION include)
