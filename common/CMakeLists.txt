cmake_minimum_required(VERSION 2.6)

message(STATUS ">>> common subdirectory")

set(RGBD_RTK_COMMON_SRC geometry.cpp event_logger.cpp)
set(RGBD_RTK_COMMON_INCLUDES common_types.h geometry.h event_logger.h)

add_library(rgbd_rtk_common SHARED ${RGBD_RTK_COMMON_SRC})
target_link_libraries(rgbd_rtk_common ${PCL_LIBRARIES})

set_target_properties(rgbd_rtk_common PROPERTIES OUTPUT_NAME "rgbd_rtk_common")
set_target_properties(rgbd_rtk_common PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS rgbd_rtk_common
          EXPORT rgbd_rtkTargets
  		  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  		  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  		  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/rgbd_rtk_common"
    	  COMPONENT dev)

install(FILES ${RGBD_RTK_COMMON_INCLUDES} DESTINATION include)