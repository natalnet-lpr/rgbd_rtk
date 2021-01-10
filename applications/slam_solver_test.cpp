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

void addVertixAndEdge(
    Eigen::Affine3f new_keyframe_pose,
    Eigen::Affine3f& last_keyframe_pose,
    Eigen::Affine3f& first_keyframe_pose,
    int& num_keyframes,
    SLAM_Solver& slam_solver,
    ReconstructionVisualizer& visualizer,
    bool isLoopClosure);

/**
 * This program shows the use of camera pose estimation using optical flow visual odometry.
 * @param .yml config. file (from which index_file is used)
 */
int main(int argc, char** argv)
{
    int num_keyframes = 0;
    bool slam_solver_started = false;
    float marker_size, aruco_max_distance;

    string camera_calibration_file, aruco_dic, index_file;
    EventLogger& logger = EventLogger::getInstance();
    logger.setVerbosityLevel(EventLogger::L_ERROR);

    SLAM_Solver slam_solver;
    ReconstructionVisualizer visualizer;
    Intrinsics intr(0);
    OpticalFlowVisualOdometry vo(intr);
    MarkerFinder marker_finder;
    RGBDLoader loader;
    Mat frame, depth;
    Eigen::Affine3f last_keyframe;
    Eigen::Affine3f first_keyframe;
    // Slam solver will start when the marker is found for the first time
    if (argc != 2)
    {
        logger.print(
            EventLogger::L_ERROR,
            "[slam_solver_test.cpp] Usage: %s <path/to/config_file.yaml>\n",
            argv[0]);
        exit(0);
    }

    ConfigLoader param_loader(argv[1]);
    param_loader.checkAndGetString("index_file", index_file);
    param_loader.checkAndGetFloat("aruco_marker_size", marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", aruco_max_distance);
    param_loader.checkAndGetString("camera_calibration_file", camera_calibration_file);
    param_loader.checkAndGetString("aruco_dic", aruco_dic);

    marker_finder.markerParam(camera_calibration_file, marker_size, aruco_dic);
    loader.processFile(index_file);

    // Compute visual odometry on each image
    for (int i = 0; i < loader.num_images_; i++)
    {
        // Load RGB-D image
        loader.getNextImage(frame, depth);

        // If aruco is not found yet
        if (slam_solver_started == false)
        {
            marker_finder.detectMarkersPoses(
                frame, Eigen::Affine3f::Identity(), aruco_max_distance);
            for (size_t i = 0; i < marker_finder.markers_.size(); i++)
            {
                // THIS SHOULD NOT BE HERE, THE ONLY REASON IS HERE IS BECAUSE I'M TESTING
                // A PROGRAM THAT SHOULD USE ONLY ONE MARKER IN A DATASET WITH MULTIPLE MARKERS
                if (147 == marker_finder.markers_[i].id)
                {
                    // If we found the mark for the first time, we will add the marker as the origin
                    // of the system Then we add the first camera pose related to the marker pose.
                    // Getting Camera Pose
                    vo.pose_ = marker_finder.marker_poses_[i];
                    // Adding first pose to slam_solver and visualizer
                    slam_solver.addVertexAndEdge(vo.pose_, num_keyframes);
                    visualizer.addReferenceFrame(vo.pose_, to_string(num_keyframes));
                    num_keyframes++; // Increment the number of keyframes found
                    // Set slam solver started to true since we found the marker for the first time
                    slam_solver_started = true;

                    first_keyframe = vo.pose_;
                    last_keyframe = vo.pose_;
                }
            }
            continue;
        }

        // If we have already found the marker we start the keyframe process
        else
        {
            marker_finder.detectMarkersPoses(frame, vo.pose_, aruco_max_distance, false);
            for (size_t i = 0; i < marker_finder.markers_.size(); i++)
            {
                if (147 == marker_finder.markers_[i].id)
                {
                    addVertixAndEdge(
                        marker_finder.marker_poses_[i],
                        last_keyframe,
                        first_keyframe,
                        num_keyframes,
                        slam_solver,
                        visualizer,
                        true);
                }
            }
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
            visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);
            visualizer.addQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);

            // If we found a keyframe we will added to slam solver and visualizer
            if (is_kf)
            {
                addVertixAndEdge(
                    vo.pose_,
                    last_keyframe,
                    first_keyframe,
                    num_keyframes,
                    slam_solver,
                    visualizer,
                    false);
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
        }
    }
    visualizer.spin();
    slam_solver.optimizeGraph(10);
    return 0;
}

void addVertixAndEdge(
    Eigen::Affine3f new_keyframe_pose,
    Eigen::Affine3f& last_keyframe_pose,
    Eigen::Affine3f& first_keyframe_pose,
    int& num_keyframes,
    SLAM_Solver& slam_solver,
    ReconstructionVisualizer& visualizer,
    bool isLoopClosure)
{
    double x = pow(new_keyframe_pose(0, 3) - last_keyframe_pose(0, 3), 2);
    double y = pow(new_keyframe_pose(1, 3) - last_keyframe_pose(1, 3), 2);
    double z = pow(new_keyframe_pose(2, 3) - last_keyframe_pose(2, 3), 2);

    cout << sqrt(x + y + z) << endl;
    if (sqrt(x + y + z) >= 0.2)
    {
        visualizer.addReferenceFrame(new_keyframe_pose, to_string(num_keyframes));
        // Add the pose given by marker and creating an edge to the last vertex
        slam_solver.addVertexAndEdge(new_keyframe_pose, num_keyframes);

        Edge ed(
            num_keyframes - 2,
            num_keyframes - 1,
            Eigen::Vector3d(
                last_keyframe_pose(0, 3), last_keyframe_pose(1, 3), last_keyframe_pose(2, 3)),
            Eigen::Vector3d(
                new_keyframe_pose(0, 3), new_keyframe_pose(1, 3), new_keyframe_pose(2, 3)),
            to_string(num_keyframes - 2) + " to " + to_string(num_keyframes - 1));
        visualizer.addEdge(ed);

        if (isLoopClosure == true)
        {
            // Adding the loop closing edge that is an edge from this vertex to the initial
            slam_solver.addLoopClosingEdge(new_keyframe_pose, num_keyframes);
            Edge ed_loop_closure(
                0,
                num_keyframes - 1,
                Eigen::Vector3d(
                    first_keyframe_pose(0, 3),
                    first_keyframe_pose(1, 3),
                    first_keyframe_pose(2, 3)),
                Eigen::Vector3d(
                    new_keyframe_pose(0, 3), new_keyframe_pose(1, 3), new_keyframe_pose(2, 3)),
                "loop_" + to_string(num_keyframes));
            visualizer.addEdge(ed_loop_closure, Eigen::Vector3f(1.0, 0.0, 0.0));
        }

        num_keyframes++; // Increment the number of keyframes found
        last_keyframe_pose = new_keyframe_pose;
    }
}