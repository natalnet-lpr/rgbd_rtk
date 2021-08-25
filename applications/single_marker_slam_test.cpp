/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2021, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Authors:
 *
 *  Rodrigo Sarmento
 *  Bruno Silva
 */

// C++
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

// C++ Third Party
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// RGBD RTK
#include <config_loader.h>
#include <event_logger.h>
#include <geometry.h>
#include <marker_finder.h>
#include <optical_flow_visual_odometry.h>
#include <reconstruction_visualizer.h>
#include <rgbd_loader.h>
#include <pose_graph_slam.h>
#include <single_marker_slam.h>

using namespace std;
using namespace cv;
using namespace aruco;
using namespace g2o;

int main(int argc, char** argv)
{
    EventLogger& logger = EventLogger::getInstance();
    logger.activateLoggingOnlyFor(EventLogger::M_SLAM);
    logger.activateLoggingFor(EventLogger::M_COMMON);
    logger.activateLoggingFor(EventLogger::M_VISUAL_ODOMETRY);
    logger.setVerbosityLevel(EventLogger::L_DEBUG);

    RGBDLoader loader;
    ReconstructionVisualizer visualizer;
    VisualOdometry::Parameters vo_param;
    MarkerFinder::Parameters mf_param;
    SingleMarkerSLAM::Parameters slam_param;
    string index_file;
    Mat frame, depth;

    // Slam solver will start when the marker is found for the first time
    if (argc != 2)
    {
        logger.print(EventLogger::L_ERROR, "[single_marker_slam_test.cpp] Usage: %s <path/to/config_file.yaml>\n", argv[0]);
        exit(0);
    }

    ConfigLoader param_loader(argv[1]);

    param_loader.checkAndGetString("index_file", index_file);
    //tracker parameters
    param_loader.checkAndGetString("tracker_type", vo_param.tracker_param_.type_);
    param_loader.checkAndGetInt("min_pts", vo_param.tracker_param_.min_pts_);
    param_loader.checkAndGetInt("max_pts", vo_param.tracker_param_.max_pts_);
    param_loader.checkAndGetBool("log_stats", vo_param.tracker_param_.log_stats_);
    //motion estimator parameters
    param_loader.checkAndGetFloat("ransac_distance_threshold", vo_param.ransac_thr_);
    vo_param.intr_ = Intrinsics(0); //TODO: load this parameter from file
    //marker detection parameters
    param_loader.checkAndGetString("camera_calibration_file", mf_param.calib_file_);
    param_loader.checkAndGetFloat("aruco_marker_size", mf_param.marker_size_);
    param_loader.checkAndGetFloat("aruco_max_distance", mf_param.max_marker_dist_);
    param_loader.checkAndGetString("aruco_dic", mf_param.aruco_dict_);
    //slam with markers parameters
    slam_param.opt_iterations_ = 10; //TODO: load this parameter from file
    param_loader.checkAndGetInt("marker_id",
                                slam_param.marker_id_);
    param_loader.checkAndGetInt("optimization_loop_closures",
                                slam_param.opt_loop_closures_);
    param_loader.checkAndGetFloat("minimum_distance_between_keyframes",
                                  slam_param.min_dist_bw_keyframes_);

    SingleMarkerSLAM sm_slam(vo_param, mf_param, slam_param);

    loader.processFile(index_file);

    // Compute SLAM using a single marker with each image
    for (int i = 0; i < loader.num_images_; i++)
    {
        MLOG_DEBUG(EventLogger::M_SLAM, "@single_marker_slam_test: processing image %i\n", i);

        // Load RGB-D image
        loader.getNextImage(frame, depth);

        // Process the RGB-D image using the supplied marker id as reference
        sm_slam.processImage(frame, depth, slam_param.marker_id_); 

        // Update visualization
        visualizer.viewReferenceFrame(sm_slam.visualOdometryPose());
        visualizer.viewReferenceFrame(sm_slam.markerPose(), "AR_pose");
        visualizer.viewQuantizedPointCloud(sm_slam.visualOdometryPointCloud(), 0.02, sm_slam.visualOdometryPose());
        visualizer.viewKeyframes(sm_slam.keyframes());
        visualizer.spinOnce();

        imshow("Image view", frame);

        char key = waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            logger.print(EventLogger::L_INFO, "[single_marker_slam_test.cpp] Exiting.\n");
            break;
        }
    }

    visualizer.spin();

    return 0;
}