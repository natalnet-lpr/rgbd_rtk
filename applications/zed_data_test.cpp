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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author:
 *
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

int main(int argc, char** argv)
{
    EventLogger& logger = EventLogger::getInstance();
    logger.setVerbosityLevel(EventLogger::L_DEBUG);

    ReconstructionVisualizer visualizer;
    RGBDLoader loader;
    VisualOdometry::Parameters vo_param;
    Intrinsics intr;

    Mat frame, depth;
    float marker_size, aruco_max_distance;
    string camera_calibration_file, aruco_dic, index_file;

    if (argc != 2)
    {
        logger.print(
            EventLogger::L_ERROR, "[zed_data_test.cpp] ERROR: Usage: %s <path/to/config_file.yaml>\n", argv[0]);
        exit(0);
    }
    
    ConfigLoader param_loader(argv[1]);
    //file parameters
    param_loader.checkAndGetString("index_file", index_file);
    intr.loadFromFile(camera_calibration_file);
    intr.setScale(1000.0); //transform zed point cloud from milimeters to meters
    //tracker parameters
    param_loader.checkAndGetString("tracker_type", vo_param.tracker_param_.type_);
    param_loader.checkAndGetInt("min_pts", vo_param.tracker_param_.min_pts_);
    param_loader.checkAndGetInt("max_pts", vo_param.tracker_param_.max_pts_);
    param_loader.checkAndGetBool("log_stats", vo_param.tracker_param_.log_stats_);
    //motion estimator parameters
    param_loader.checkAndGetFloat("ransac_distance_threshold", vo_param.ransac_thr_);
    vo_param.intr_ = intr;
    //marker detection parameters
    param_loader.checkAndGetFloat("aruco_marker_size", marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", aruco_max_distance);
    param_loader.checkAndGetString("camera_calibration_file", camera_calibration_file);
    param_loader.checkAndGetString("aruco_dic", aruco_dic);

    MarkerFinder marker_finder(camera_calibration_file, marker_size, aruco_dic);
    OpticalFlowVisualOdometry vo(vo_param);

    loader.processFile(index_file);

    //pcl::PointCloud<PointT>::Ptr cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    // Compute visual odometry and find markers on each image
    for (int i = 0; i < loader.num_images_; i++)
    {
        // Load RGB-D image
        loader.getNextImage(frame, depth);

        // Estimate current camera pose
        vo.computeCameraPose(frame, depth); //resulting in ransac exception (no keypoints)
        //*cloud = getPointCloud(frame, depth, intr);

        // Find ARUCO markers and compute their poses
        /*
        marker_finder.detectMarkersPoses(frame, vo.pose_, aruco_max_distance);
        for (size_t j = 0; j < marker_finder.markers_.size(); j++)
        {
            if (250 == marker_finder.markers_[j].id)
            {
                if (i == 0)
                {
                    first = marker_finder.marker_poses_[j];
                    marker_finder.markers_[j].draw(frame, Scalar(0, 0, 255), 1.5);
                    CvDrawingUtils::draw3dAxis(frame, marker_finder.markers_[j], marker_finder.camera_params_, 2);
                    stringstream ss;
                    ss << "m" << marker_finder.markers_[j].id;
                    visualizer.viewReferenceFrame(marker_finder.marker_poses_[j], ss.str());
                }
            }
        }
        */

        if (i == 0) visualizer.addReferenceFrame(vo.pose_, "origin");

        //visualizer.viewPointCloud(cloud, vo.pose_);
        //visualizer.viewQuantizedPointCloud(cloud, 0.02, vo.pose_);
        visualizer.addQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);
        visualizer.viewReferenceFrame(vo.pose_);
        // visualizer.viewPointCloud(vo.curr_dense_cloud_, vo.pose_);
        visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.02, vo.pose_);

        visualizer.spinOnce();

        // Show RGB-D image
        imshow("Image view", frame);
        char key = waitKey(0);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            printf("Exiting.\n");
            break;
        }
    }

    visualizer.close();

    return 0;
}