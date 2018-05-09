cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

message(STATUS ">>> applications subdirectory")

add_executable(monocular_calibration monocular_calibration.cpp)
add_executable(monocular_image_grabber monocular_image_grabber.cpp)
add_executable(sequence_loader_test sequence_loader_test.cpp)
add_executable(rgbd_loader_test rgbd_loader_test.cpp)
add_executable(klt_tracker_test klt_tracker_test.cpp)
add_executable(klttw_tracker_test klttw_tracker_test.cpp)
add_executable(klttcw_tracker_test klttcw_tracker_test.cpp)
add_executable(kltatcw_tracker_test kltatcw_tracker_test.cpp)
add_executable(motion_estimator_test motion_estimator_test.cpp)
add_executable(optical_flow_visual_odometry_test optical_flow_visual_odometry_test.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

find_library(RGBD_RTK_IO NAMES rgbd_rtk_io)
find_library(RGBD_RTK_TRACKING NAMES rgbd_rtk_tracking)
find_library(RGBD_RTK_COMMON NAMES rgbd_rtk_common)
find_library(RGBD_RTK_VISUALIZATION NAMES rgbd_rtk_visualization)
find_library(RGBD_RTK_MOTION_ESTIMATION NAMES rgbd_rtk_motion_estimation)
find_library(RGBD_RTK_VISUAL_ODOMETRY NAMES rgbd_rtk_visual_odometry)

target_link_libraries(monocular_calibration ${OpenCV_LIBRARIES})
target_link_libraries(monocular_image_grabber ${OpenCV_LIBRARIES})
target_link_libraries(sequence_loader_test ${RGBD_RTK_IO})
target_link_libraries(rgbd_loader_test ${RGBD_RTK_IO})
target_link_libraries(klt_tracker_test ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING})
target_link_libraries(klttw_tracker_test ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING})
target_link_libraries(klttcw_tracker_test ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING})
target_link_libraries(kltatcw_tracker_test ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING})
target_link_libraries(motion_estimator_test  ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
	                                         ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION}
	                                         ${RGBD_RTK_MOTION_ESTIMATION})
target_link_libraries(optical_flow_visual_odometry_test ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
	                                                    ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION}
	                                                    ${RGBD_RTK_MOTION_ESTIMATION}
	                                                    ${RGBD_RTK_VISUAL_ODOMETRY})

install(TARGETS monocular_calibration
	        monocular_image_grabber
	        sequence_loader_test
                rgbd_loader_test
                klt_tracker_test
                klttw_tracker_test
                klttcw_tracker_test
                kltatcw_tracker_test
                motion_estimator_test
                optical_flow_visual_odometry_test
        RUNTIME DESTINATION bin)