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

struct Aruco
{
    int id;
    Eigen::Affine3f pose;
};

/**
 * Adds vertix and edges in slam_solver and visualizer
 * @param new_keyframe_pose new keyframe that should be added
 * @param last_keyframe_pose the last keyframe added in graph, this is update this in this function
 * @param first_keyframe_pose the first keyframe
 * @param num_keyframes number of keyframes in graph, this is update in this function
 * @param slam_solver slam_solver reference
 * @param visualizer visualizer reference
 * @param is_loop_closure if the vertex added is a loop_closure as well
 */
void addVertixAndEdge(
    Eigen::Affine3f cam_pose,
    Eigen::Affine3f& last_keyframe_pose,
    Eigen::Affine3f aruco_pose,
    int& num_keyframes,
    SLAM_Solver& slam_solver,
    ReconstructionVisualizer& visualizer,
    bool is_loop_closure);
/**
 * This program shows the use of camera pose estimation using optical flow visual odometry.
 * @param .yml config. file (from which index_file is used)
 */
int main(int argc, char** argv)
{
    int num_keyframes = 0;
    int num_keyframes_local = 0;
    bool slam_solver_started = false;
    bool pose_has_been_added = false;
    float marker_size, aruco_max_distance;

    string camera_calibration_file, aruco_dic, index_file;
    EventLogger& logger = EventLogger::getInstance();
    logger.setVerbosityLevel(EventLogger::L_ERROR);

    SLAM_Solver slam_solver;
    SLAM_Solver slam_solver_local;
    ReconstructionVisualizer visualizer;
    ReconstructionVisualizer visualizer_local;
    Intrinsics intr(0);
    OpticalFlowVisualOdometry vo(intr);
    MarkerFinder marker_finder;
    RGBDLoader loader;
    Mat frame, depth;
    Eigen::Affine3f last_keyframe_pose;
    Aruco first_keyframe_pose;
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
            if (marker_finder.markers_.size() > 0)
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

                first_keyframe_pose.pose = vo.pose_;
                first_keyframe_pose.id = marker_finder.markers_[i].id;
                last_keyframe_pose = vo.pose_;
            }
            continue;
        }

        // If the markers has already be found then start keyframe process
        else
        {
            // Reseting local graph
            pose_has_been_added = false;
            num_keyframes_local = 0;
            slam_solver_local.resetGraph();

            marker_finder.detectMarkersPoses(
                frame, Eigen::Affine3f::Identity(), aruco_max_distance);
            // Iterate over all markers that has been found
            for (size_t i = 0; i < marker_finder.markers_.size(); i++)
            {
                // If the origin marker has been found again add a edge from cam to origin
                if (first_keyframe_pose.id == marker_finder.markers_[i].id)
                {
                    addVertixAndEdge(
                        vo.pose_,
                        last_keyframe_pose,
                        vo.pose_ * marker_finder.marker_poses_[i],
                        num_keyframes,
                        slam_solver,
                        visualizer,
                        true);
                }
                // If is another marker, create a subgraph to optimize the cam pose
                else
                {

                    // Only create a subgraph if there is more than two markers
                    if (marker_finder.markers_.size() > 2)
                    {

                        if (!pose_has_been_added) // Add cam pose to subgraph
                        {
                            visualizer_local.addReferenceFrame(
                                vo.pose_, to_string(num_keyframes_local));
                            slam_solver_local.addVertexAndEdge(vo.pose_, num_keyframes_local);
                            pose_has_been_added = true;
                            num_keyframes_local++;
                        }
                        else
                        {
                            // Adding markers to subgraph
                            visualizer_local.addReferenceFrame(
                                vo.pose_ * marker_finder.marker_poses_[i],
                                to_string(num_keyframes_local));

                            slam_solver_local.addVertexAndEdge(
                                vo.pose_ * marker_finder.marker_poses_[i], num_keyframes_local);

                            visualizer_local.addEdge(slam_solver_local.odometry_edges_.back());
                            num_keyframes_local++;
                            // If we are in the last iteration optimize the subgraph
                            // TODO If the origin marker is the last one found this will not be
                            // called
                            if (i == marker_finder.markers_.size() - 1)
                            {
                                slam_solver_local.optimizeGraph(10);
                                visualizer_local.addOptimizedEdges(
                                    slam_solver_local.odometry_edges_,
                                    Eigen::Vector3f(1.0, 0.0, 1.0));
                                visualizer_local.addOptimizedEdges(
                                    slam_solver_local.loop_edges_, Eigen::Vector3f(0.0, 1.0, 1.0));
                                visualizer_local.addReferenceFrame(
                                    slam_solver_local.optimized_estimates_[0], "NOVA POSE");
                            }
                        }
                    }
                }
            }
            if (slam_solver_local.optimized_estimates_.size() > 2)
                vo.pose_ = slam_solver_local.optimized_estimates_[0];

            visualizer_local.spinOnce();
            visualizer_local.resetVisualizer();
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
            visualizer_local.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);
            visualizer_local.addQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);
            // If we found a keyframe we will added to slam solver and visualizer
            if (is_kf)
            {
                addVertixAndEdge(
                    vo.pose_,
                    last_keyframe_pose,
                    Eigen::Affine3f::Identity(),
                    num_keyframes,
                    slam_solver,
                    visualizer,
                    false);
            }
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
    slam_solver.optimizeGraph(10);
    visualizer.addOptimizedEdges(slam_solver.odometry_edges_, Eigen::Vector3f(1.0, 0.0, 1.0));
    visualizer.addOptimizedEdges(slam_solver.loop_edges_, Eigen::Vector3f(0.0, 1.0, 1.0));
    visualizer.spin();
    return 0;
}

void addVertixAndEdge(
    Eigen::Affine3f cam_pose,
    Eigen::Affine3f& last_keyframe_pose,
    Eigen::Affine3f aruco_pose,
    int& num_keyframes,
    SLAM_Solver& slam_solver,
    ReconstructionVisualizer& visualizer,
    bool is_loop_closure)

{
    double x = pow(cam_pose(0, 3) - last_keyframe_pose(0, 3), 2);
    double y = pow(cam_pose(1, 3) - last_keyframe_pose(1, 3), 2);
    double z = pow(cam_pose(2, 3) - last_keyframe_pose(2, 3), 2);
    // We will only add a new keyframe if they have at least 5cm of distance between each other
    if (sqrt(x + y + z) >= 0.05)
    {
        // Adding keyframe in visualizer
        visualizer.addReferenceFrame(cam_pose, to_string(num_keyframes));
        // Add the keyframe and creating an edge to the last vertex
        slam_solver.addVertexAndEdge(cam_pose, num_keyframes);
        visualizer.addEdge(slam_solver.odometry_edges_.back());

        // If the keyframe is a loop closure we will create a loop closing edge
        if (is_loop_closure == true)
        {
            // Adding the loop closing edge that is an edge from this vertex to the initial
            // If we want to change the system coord from A to B -> A.inverse * B
            slam_solver.addLoopClosingEdge(cam_pose.inverse() * aruco_pose, num_keyframes);
            visualizer.addEdge(slam_solver.loop_edges_.back(), Eigen::Vector3f(1.0, 0.0, 0.0));
            // Make a optimization in the graph from every 5 loop edges
            if (slam_solver.odometry_edges_.size() % 10 == 0)
            {
                // visualizer.removeEdges(slam_solver.odometry_edges_);
                // visualizer.removeEdges(slam_solver.loop_edges_);
                // for (int i = 0; i < slam_solver.optimized_estimates_.size(); i++)
                // {
                //  visualizer.addReferenceFrame(
                //        slam_solver.optimized_estimates_[i], "optimized " + i);
                //}
                slam_solver.optimizeGraph(10);
                visualizer.addOptimizedEdges(
                    slam_solver.odometry_edges_, Eigen::Vector3f(1.0, 0.0, 1.0));
                visualizer.addOptimizedEdges(
                    slam_solver.loop_edges_, Eigen::Vector3f(0.0, 1.0, 1.0));
            }
        }

        num_keyframes++;               // Increment the number of keyframes found
        last_keyframe_pose = cam_pose; // Updating the pose of last keyframe
    }
}