/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or
 *     promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author:
 *
 *  Bruno Silva
 */

// C++
#include <cstdio>
#include <cstdlib>
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
#include <slam_solver.h>

using namespace std;
using namespace cv;
using namespace aruco;

struct Pose
{
    int id;
    float x;
    float y;
    float z;
    float x_rotation;
    float y_rotation;
    float z_rotation;
    float w_rotation;
};

EventLogger& logger = EventLogger::getInstance();
RGBDLoader loader;
Intrinsics intr(0);
OpticalFlowVisualOdometry vo(intr);
ReconstructionVisualizer visualizer;
Mat frame, depth;
SLAM_Solver slam_solver;
float marker_size, aruco_max_distance;
vector<Pose> all_markers; // List of marker struct
string camera_calibration_file, aruco_dic, index_file;
MarkerFinder marker_finder;
int num_keyframes = 0;

/**
 * Check if the aruco is already found, if it is we do nothing, if it was not we add
 * in the aruco vector
 */
void isArucoFound(int i);
/**
 * This program shows the use of camera pose estimation using optical flow visual odometry.
 * @param .yml config. file (from which index_file is used)
 */
int main(int argc, char** argv)
{
    logger.setVerbosityLevel(EventLogger::L_ERROR);
    if (argc != 2)
    {
        logger.print(
            EventLogger::L_INFO,
            "[optical_flow_visual_odometry_test.cpp] Usage: %s <path/to/config_file.yaml>\n",
            argv[0]);
        exit(0);
    }

    ConfigLoader param_loader(argv[1]);
    param_loader.checkAndGetString("index_file", index_file);
    param_loader.checkAndGetFloat("aruco_marker_size", marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", aruco_max_distance);
    param_loader.checkAndGetString("camera_calibration_file", camera_calibration_file);
    param_loader.checkAndGetString("index_file", index_file);
    param_loader.checkAndGetString("aruco_dic", aruco_dic);

    marker_finder.markerParam(camera_calibration_file, marker_size, aruco_dic);
    loader.processFile(index_file);
    visualizer.addReferenceFrame(vo.pose_, "origin");

    // Compute visual odometry on each image
    for (int i = 0; i < loader.num_images_; i++)
    {
        // Load RGB-D image
        loader.getNextImage(frame, depth);

        // Estimate current camera pose
        bool is_kf = vo.computeCameraPose(frame, depth);

        // View tracked points
        for (size_t k = 0; k < vo.tracker_.curr_pts_.size(); k++)
        {
            Point2i pt1 = vo.tracker_.prev_pts_[k];
            Point2i pt2 = vo.tracker_.curr_pts_[k];
            Scalar color;

            is_kf ? color = CV_RGB(255, 0, 0) : color = CV_RGB(0, 0, 255);

            circle(frame, pt1, 1, color, -1);
            circle(frame, pt2, 3, color, -1);
            line(frame, pt1, pt2, color);
        }

        visualizer.viewReferenceFrame(vo.pose_);
        visualizer.viewPointCloud(vo.curr_dense_cloud_, vo.pose_);
        visualizer.addQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);
        visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);
        visualizer.addCameraPath(vo.pose_);

        if (is_kf)
        {
            slam_solver.addVertexAndEdge(vo.getLastKeyframe().pose_, num_keyframes);
            visualizer.addReferenceFrame(vo.getLastKeyframe().pose_, to_string(num_keyframes));
            num_keyframes++; // Increment the number of keyframes found
        }

        visualizer.spinOnce();

        imshow("Image view", frame);
        // imshow("Depth view", depth);
        char key = waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            logger.print(EventLogger::L_INFO, "[slam_solver_test.cpp] Exiting\n", argv[0]);
            break;
        }
        // Find ARUCO markers and compute their poses
        marker_finder.detectMarkersPoses(frame, vo.pose_, aruco_max_distance);
        for (size_t i = 0; i < marker_finder.markers_.size(); i++) { isArucoFound(i); }
    }
    slam_solver.optimizeGraph(10);
    return 0;
}

void isArucoFound(int i)
{
    Pose aruco_pose;
    bool marker_found = false;
    Eigen::Quaternionf q_aruco; // Quaternion of aruco pose

    // I'm pushing the pose in a vector because we can use tha position to create a threashold
    // when we see a marker already seen, something like "if I'm closer to the marker I'll update the position"

    if (all_markers.size() == 0)
    {
        aruco_pose.id = marker_finder.markers_[i].id;
        aruco_pose.x = marker_finder.marker_poses_[i](0, 3);
        aruco_pose.y = marker_finder.marker_poses_[i](1, 3);
        aruco_pose.z = marker_finder.marker_poses_[i](2, 3);
        q_aruco = marker_finder.marker_poses_[i].rotation();
        aruco_pose.x_rotation = q_aruco.x();
        aruco_pose.y_rotation = q_aruco.y();
        aruco_pose.z_rotation = q_aruco.z();
        aruco_pose.w_rotation = q_aruco.w();
        all_markers.push_back(aruco_pose);
        slam_solver.addLoopClosingEdge(marker_finder.marker_poses_[i], num_keyframes);
        visualizer.addReferenceFrame(marker_finder.marker_poses_[i], to_string(num_keyframes));
        num_keyframes++; // Increment the number of keyframes found
    }
    else
    {
        // If it is not empty, we need to be sure that the marker we have already exists
        for (auto pose : all_markers)
        {
            if (marker_finder.markers_[i].id == pose.id)
            {
                marker_found = true;
                break;
            }
            // If exists we do nothing
        }
        // If do not exists we add
        if (marker_found == false)
        {
            aruco_pose.id = marker_finder.markers_[i].id;
            aruco_pose.x = marker_finder.marker_poses_[i](0, 3);
            aruco_pose.y = marker_finder.marker_poses_[i](1, 3);
            aruco_pose.z = marker_finder.marker_poses_[i](2, 3);
            q_aruco = marker_finder.marker_poses_[i].rotation();
            aruco_pose.x_rotation = q_aruco.x();
            aruco_pose.y_rotation = q_aruco.y();
            aruco_pose.z_rotation = q_aruco.z();
            aruco_pose.w_rotation = q_aruco.w();
            all_markers.push_back(aruco_pose);
            slam_solver.addLoopClosingEdge(marker_finder.marker_poses_[i], num_keyframes);
            visualizer.addReferenceFrame(marker_finder.marker_poses_[i], to_string(num_keyframes));
            num_keyframes++; // Increment the number of keyframes found
        }
    }
}