cmake_minimum_required(VERSION 2.6)

message(STATUS ">>> applications subdirectory")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

find_library(RGBD_RTK_COMMON NAMES rgbd_rtk_common)

if(WITH_IO)
    ##Install applications that only needs IO
    add_executable(sequence_loader_test sequence_loader_test.cpp)
    add_executable(rgbd_loader_test rgbd_loader_test.cpp)
    add_executable(event_logger_test event_logger_test.cpp)
    add_executable(kitti_loader_test kitti_loader_test.cpp)

    find_library(RGBD_RTK_IO NAMES rgbd_rtk_io)
    target_link_libraries(sequence_loader_test ${RGBD_RTK_IO})
    target_link_libraries(rgbd_loader_test ${RGBD_RTK_COMMON} ${RGBD_RTK_IO})
    target_link_libraries(event_logger_test ${RGBD_RTK_COMMON} ${RGBD_RTK_IO})
    target_link_libraries(kitti_loader_test ${RGBD_RTK_IO})
    install(TARGETS sequence_loader_test rgbd_loader_test event_logger_test kitti_loader_test
            EXPORT rgbd_rtkTargets
            RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
endif()

if(WITH_TRACKING)
    find_library(RGBD_RTK_TRACKING NAMES rgbd_rtk_tracking)
endif()

if(WITH_VISUALIZATION)
    ##Install applications that only needs visualization
    add_executable(kinect_grabber kinect_grabber.cpp)
    find_library(RGBD_RTK_VISUALIZATION NAMES rgbd_rtk_visualization)
    target_link_libraries(kinect_grabber ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION})
    install(TARGETS kinect_grabber
            EXPORT rgbd_rtkTargets
            RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
endif()

if(WITH_QUADTREE)
    find_library(RGBD_RTK_QUADTREE NAMES rgbd_rtk_quadtree)
endif()

if(WITH_VISUAL_ODOMETRY)
    find_library(RGBD_RTK_VISUAL_ODOMETRY NAMES rgbd_rtk_visual_odometry)
endif()

if(WITH_MOTION_ESTIMATION)
    find_library(RGBD_RTK_MOTION_ESTIMATION NAMES rgbd_rtk_motion_estimation)
endif()

if(WITH_STEREO)
    find_library(RGBD_RTK_STEREO NAMES rgbd_rtk_stereo)
endif()

##Install applications that only needs opencv
add_executable(monocular_calibration monocular_calibration.cpp)
add_executable(monocular_image_grabber monocular_image_grabber.cpp)
target_link_libraries(monocular_calibration ${OpenCV_LIBRARIES})
target_link_libraries(monocular_image_grabber ${OpenCV_LIBRARIES})
install(TARGETS monocular_calibration monocular_image_grabber
            EXPORT rgbd_rtkTargets
            RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)


if(WITH_IO AND WITH_TRACKING)
    ##Install applications that only needs IO and Tracking
    add_executable(klt_tracker_test klt_tracker_test.cpp)
    add_executable(klttw_tracker_test klttw_tracker_test.cpp)
    add_executable(klttcw_tracker_test klttcw_tracker_test.cpp)
    add_executable(kltatcw_tracker_test kltatcw_tracker_test.cpp)
    add_executable(wide_baseline_tracker_test wide_baseline_tracker_test.cpp)

    target_link_libraries(klt_tracker_test ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING})
    target_link_libraries(klttw_tracker_test ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING})
    target_link_libraries(klttcw_tracker_test ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING})
    target_link_libraries(kltatcw_tracker_test ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING})
    target_link_libraries(wide_baseline_tracker_test ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING})
    install(TARGETS klt_tracker_test klttw_tracker_test klttcw_tracker_test kltatcw_tracker_test wide_baseline_tracker_test
            EXPORT rgbd_rtkTargets
            RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)

    if(WITH_QUADTREE)
        ##Install applications that needs IO, Tracking and Quadtree
        add_executable(kltqt_tracker_test kltqt_tracker_test.cpp)
        target_link_libraries(kltqt_tracker_test ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING} ${RGBD_RTK_QUADTREE})
        install(TARGETS kltqt_tracker_test
                EXPORT rgbd_rtkTargets
                RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
    endif()

    if(WITH_VISUALIZATION AND WITH_MOTION_ESTIMATION)
        ##Install applications that needs IO, Tracking, Visualization and Motion Estimation
        add_executable(motion_estimator_test motion_estimator_test.cpp)
        target_link_libraries(motion_estimator_test  ${PCL_LIBRARIES} ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING} 
                                                     ${RGBD_RTK_VISUALIZATION}
	                                                 ${RGBD_RTK_MOTION_ESTIMATION})
        install(TARGETS motion_estimator_test
                EXPORT rgbd_rtkTargets
                RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)

        if(WITH_VISUAL_ODOMETRY)
            ##Install applications that needs IO,Tracking, Visualization, Motion Estimation and Visual Odometry
            add_executable(optical_flow_visual_odometry_test optical_flow_visual_odometry_test.cpp)
            add_executable(wide_baseline_visual_odometry_test wide_baseline_visual_odometry_test.cpp)
            add_executable(icp_odometry_test icp_odometry_test.cpp)

            target_link_libraries(optical_flow_visual_odometry_test ${PCL_LIBRARIES} ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
                                                                    ${RGBD_RTK_VISUALIZATION}
                                                                    ${RGBD_RTK_MOTION_ESTIMATION}
                                                                    ${RGBD_RTK_VISUAL_ODOMETRY})
                                                                    
            target_link_libraries(wide_baseline_visual_odometry_test ${PCL_LIBRARIES} ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
                                                                     ${RGBD_RTK_VISUALIZATION}
                                                                     ${RGBD_RTK_MOTION_ESTIMATION}
                                                                     ${RGBD_RTK_VISUAL_ODOMETRY})
                                                                    
            target_link_libraries(icp_odometry_test ${PCL_LIBRARIES} ${RGBD_RTK_COMMON} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
                                                    ${RGBD_RTK_VISUALIZATION}
                                                    ${RGBD_RTK_MOTION_ESTIMATION}
                                                    ${RGBD_RTK_VISUAL_ODOMETRY})

            install(TARGETS optical_flow_visual_odometry_test wide_baseline_visual_odometry_test icp_odometry_test
                EXPORT rgbd_rtkTargets
                RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
        endif()
    endif()
endif()

if(WITH_IO AND WITH_VISUALIZATION)
    ##Install applications that needs IO and Visualization
    add_executable(kitti_velodyne_loader_test kitti_velodyne_loader_test.cpp)
    target_link_libraries(kitti_velodyne_loader_test ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_VISUALIZATION})
    install(TARGETS kitti_velodyne_loader_test
                EXPORT rgbd_rtkTargets
                RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
    
    if(WITH_STEREO)
        ##Install applications that needs IO, Visualization and Stereo
        add_executable(stereo_cloud_generator_test stereo_cloud_generator_test.cpp)
        target_link_libraries(stereo_cloud_generator_test ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_VISUALIZATION}
                                                  ${RGBD_RTK_STEREO})
        install(TARGETS stereo_cloud_generator_test
                EXPORT rgbd_rtkTargets
                RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)

        if(WITH_VISUAL_ODOMETRY AND WITH_MOTION_ESTIMATION)
            ##Install applications that needs IO, Visualization, Stereo, Visual Odometry and Motion Estimation
            add_executable(stereo_optical_flow_visual_odometry_test stereo_optical_flow_visual_odometry_test.cpp)
            target_link_libraries(stereo_optical_flow_visual_odometry_test ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_VISUALIZATION} ${RGBD_RTK_STEREO}                                   
                                                                ${RGBD_RTK_VISUAL_ODOMETRY}
                                                                ${RGBD_RTK_MOTION_ESTIMATION})
            
            install(TARGETS stereo_optical_flow_visual_odometry_test
                EXPORT rgbd_rtkTargets
                RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
        endif()
    endif()
endif()       

# Build SLAM related applications only if ARUCO was found
if(aruco_FOUND AND WITH_IO AND WITH_TRACKING AND WITH_VISUALIZATION AND WITH_MOTION_ESTIMATION AND WITH_VISUAL_ODOMETRY)
    add_executable(marker_finder_test marker_finder_test.cpp)
    add_executable(slam_solver_test slam_solver_test.cpp)
    find_library(RGBD_RTK_SLAM NAMES rgbd_rtk_slam)
    target_link_libraries(marker_finder_test ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
                                         ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION}
                                         ${RGBD_RTK_MOTION_ESTIMATION}
                                         ${RGBD_RTK_VISUAL_ODOMETRY}
                                         ${RGBD_RTK_SLAM})
    target_link_libraries(slam_solver_test ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
                                         ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION}
                                         ${RGBD_RTK_MOTION_ESTIMATION}
                                         ${RGBD_RTK_VISUAL_ODOMETRY}
                                         ${RGBD_RTK_SLAM})                                         
    install(TARGETS marker_finder_test 
                    slam_solver_test
        EXPORT rgbd_rtkTargets
        RUNTIME DESTINATION bin)
endif()
