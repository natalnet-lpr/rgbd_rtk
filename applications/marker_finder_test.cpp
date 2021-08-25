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
 *  Author:
 *
 *  Rodrigo Sarmento
 *  Bruno Silva
 */

#include <Eigen/Geometry>
#include <aruco/cvdrawingutils.h>
#include <cstdio>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <config_loader.h>
#include <event_logger.h>
#include <geometry.h>
#include <marker_finder.h>
#include <optical_flow_visual_odometry.h>
#include <reconstruction_visualizer.h>
#include <rgbd_loader.h>

using namespace std;
using namespace cv;
using namespace aruco;

/**
 * This program shows the use of ARUCO marker detection.
 * @param .yml config. file (used fields: index_file, aruco_marker_size, camera_calibration_file, aruco_max_distance)
 */

int main(int argc, char** argv)
{
    EventLogger& logger = EventLogger::getInstance();
    logger.setVerbosityLevel(EventLogger::L_DEBUG);

    ReconstructionVisualizer visualizer;
    RGBDLoader loader;
    FeatureTracker::Parameters tracking_param;
    Intrinsics intr(0);

    Mat frame, depth;
    float marker_size, aruco_max_distance, ransac_thr;
    string camera_calibration_file, aruco_dic, index_file;

    if (argc != 2)
    {
        logger.print(
            EventLogger::L_ERROR, "[marker_finder_test.cpp] ERROR: Usage: %s <path/to/config_file.yaml>\n", argv[0]);
        exit(0);
    }

    ConfigLoader param_loader(argv[1]);
    //file parameters
    param_loader.checkAndGetString("index_file", index_file);
    //tracker parameters
    param_loader.checkAndGetString("tracker_type", tracking_param.type_);
    param_loader.checkAndGetInt("min_pts", tracking_param.min_pts_);
    param_loader.checkAndGetInt("max_pts", tracking_param.max_pts_);
    param_loader.checkAndGetBool("log_stats", tracking_param.log_stats_);
    //motion estimator parameters
    param_loader.checkAndGetFloat("ransac_distance_threshold", ransac_thr);
    //marker detection parameters
    param_loader.checkAndGetFloat("aruco_marker_size", marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", aruco_max_distance);
    param_loader.checkAndGetString("camera_calibration_file", camera_calibration_file);
    param_loader.checkAndGetString("aruco_dic", aruco_dic);

    MarkerFinder marker_finder(camera_calibration_file, marker_size, aruco_dic);
    OpticalFlowVisualOdometry vo(intr, tracking_param, ransac_thr);

    visualizer.addReferenceFrame(vo.pose_, "origin");

    loader.processFile(index_file);

    // Compute visual odometry and find markers on each image
    for(int i = 0; i < loader.num_images_; i++)
    {
        // Load RGB-D image
        loader.getNextImage(frame, depth);

        // Estimate current camera pose
        vo.computeCameraPose(frame, depth);

        // Find ARUCO markers and compute their poses
        marker_finder.detectMarkersPoses(frame, vo.pose_, aruco_max_distance);
        for(size_t j = 0; j < marker_finder.markers_.size(); j++)
        {
            marker_finder.markers_[j].draw(frame, Scalar(0, 0, 255), 1.5);
            CvDrawingUtils::draw3dAxis(frame, marker_finder.markers_[j], marker_finder.camera_params_, 2);
            string marker_name = "m" + to_string(marker_finder.markers_[j].id);
            visualizer.viewReferenceFrame(marker_finder.marker_poses_[j], marker_name);
        }

        if (i == 0) visualizer.addReferenceFrame(vo.pose_, "origin");
        visualizer.addQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);
        visualizer.viewReferenceFrame(vo.pose_);
        // visualizer.viewPointCloud(vo.curr_dense_cloud_, vo.pose_);
        visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.02, vo.pose_);

        visualizer.spinOnce();

        // Show RGB-D image
        imshow("Image view", frame);
        char key = waitKey(100);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            printf("Exiting.\n");
            break;
        }
    }

    visualizer.close();

    return 0;
}