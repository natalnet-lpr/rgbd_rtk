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
#include <fstream>
#include <iostream> // std::cout
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

using namespace std;
using namespace cv;
using namespace aruco;

struct Aruco
{
    int id;
    Eigen::Affine3f pose;
};

struct Pose
{
    int id;
    float x;
    float y;
    float z;
};

vector<Pose> markers_found; // List of marker structs

/**
 * Load all markers
 */
void loadMarkersFound(string aruco_poses_file);

/**
 * Adds vertex and edges in single_marker_slam and visualizer
 * @param new_keyframe_pose new keyframe that should be added
 * @param last_keyframe_pose_odometry the last keyframe added in graph, this is update this in this function
 * @param last_keyframe_pose_aruco the last keyframe added in graph, this is update this in this function
 * @param aruco_pose aruco_pose
 * @param num_keyframes number of keyframes in graph, this is update in this function
 * @param single_marker_slam single_marker_slam reference
 * @param visualizer visualizer reference
 * @param is_loop_closure if the vertex added is a loop_closure as well
 */
void addVertexAndEdge(
    Eigen::Affine3f cam_pose,
    Eigen::Affine3f& last_keyframe_pose_odometry,
    Eigen::Affine3f& last_keyframe_pose_aruco,
    Eigen::Affine3f aruco_pose,
    int& num_keyframes,
    PoseGraphSLAM& single_marker_slam,
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
    bool marker_found;
    float marker_size, aruco_max_distance;

    string camera_calibration_file, aruco_dic, index_file, aruco_poses_file;
    EventLogger& logger = EventLogger::getInstance();
    logger.setVerbosityLevel(EventLogger::L_ERROR);

    PoseGraphSLAM single_marker_slam;
    PoseGraphSLAM slam_solver_local;
    ReconstructionVisualizer visualizer;
    ReconstructionVisualizer visualizer_local;
    Intrinsics intr(0);
    OpticalFlowVisualOdometry vo(intr);
    MarkerFinder marker_finder;
    RGBDLoader loader;
    Mat frame, depth;
    Eigen::Affine3f last_keyframe_pose_odometry;
    Eigen::Affine3f last_keyframe_pose_aruco;
    Eigen::Affine3f first_aruco_pose;
    Aruco aruco_origin_pose_uco_reference;
    // Slam solver will start when the marker is found for the first time
    if (argc != 2)
    {
        logger.print(EventLogger::L_ERROR, "[slam_solver_test.cpp] Usage: %s <path/to/config_file.yaml>\n", argv[0]);
        exit(0);
    }

    ConfigLoader param_loader(argv[1]);
    param_loader.checkAndGetString("index_file", index_file);
    param_loader.checkAndGetFloat("aruco_marker_size", marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", aruco_max_distance);
    param_loader.checkAndGetString("camera_calibration_file", camera_calibration_file);
    param_loader.checkAndGetString("aruco_dic", aruco_dic);
    param_loader.checkAndGetString("aruco_poses_file", aruco_poses_file);
    marker_finder.markerParam(camera_calibration_file, marker_size, aruco_dic);
    loader.processFile(index_file);

    loadMarkersFound(aruco_poses_file);

    // Compute visual odometry on each image
    for (int i = 0; i < loader.num_images_; i++)
    {
        // Load RGB-D image
        loader.getNextImage(frame, depth);

        // If aruco is not found yet
        if (slam_solver_started == false)
        {
            marker_finder.detectMarkersPoses(frame, Eigen::Affine3f::Identity(), aruco_max_distance);
            cout << marker_finder.markers_.size();
            if (marker_finder.markers_.size() > 0)
            {
                // If we found the mark for the first time, we will add the marker as the origin
                // of the system Then we add the first camera pose related to the marker pose.
                // Getting Camera Pose
                vo.pose_ = marker_finder.marker_poses_[i].inverse();
                // Adding first pose to single_marker_slam and visualizer
                single_marker_slam.addVertexAndEdge(vo.pose_, num_keyframes);
                visualizer.addReferenceFrame(vo.pose_, "first pose");
                num_keyframes++; // Increment the number of keyframes found
                // Set slam solver started to true since we found the marker for the first time
                slam_solver_started = true;

                first_aruco_pose = vo.pose_;
                last_keyframe_pose_odometry = vo.pose_;
                last_keyframe_pose_aruco = vo.pose_;

                // TODO Need to change this
                for (auto marker : markers_found)
                {
                    if (marker.id == marker_finder.markers_[i].id)
                    {
                        aruco_origin_pose_uco_reference.id = marker.id;
                        aruco_origin_pose_uco_reference.pose = Eigen::Affine3f::Identity();
                        aruco_origin_pose_uco_reference.pose(0, 3) = marker.x;
                        aruco_origin_pose_uco_reference.pose(1, 3) = marker.y;
                        aruco_origin_pose_uco_reference.pose(2, 3) = marker.z;
                        break;
                    }
                }
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
            // Reset marker finding boolean
            marker_found = false;

            marker_finder.detectMarkersPoses(frame, Eigen::Affine3f::Identity(), aruco_max_distance);
            // Iterate over all markers that has been found
            for (size_t i = 0; i < marker_finder.markers_.size(); i++)
            {

                marker_finder.markers_[i].draw(frame, Scalar(0, 0, 255), 1.5);
                CvDrawingUtils::draw3dAxis(frame, marker_finder.markers_[i], marker_finder.camera_params_, 2);
                // If the origin marker has been found again add a edge from cam to origin
                if (aruco_origin_pose_uco_reference.id == marker_finder.markers_[i].id)
                {
                    addVertexAndEdge(
                        vo.pose_,
                        last_keyframe_pose_odometry,
                        last_keyframe_pose_aruco,
                        marker_finder.marker_poses_[i],
                        num_keyframes,
                        single_marker_slam,
                        visualizer,
                        true);
                    marker_found = true;
                }
                // If is another marker, create a subgraph to optimize the cam pose
                /*
               else
               {
                   // Only create a subgraph if there is more than two markers
                   if (marker_finder.markers_.size() > 2)
                   {
                       for (auto marker : markers_found)
                       {
                           if (marker.id == marker_finder.markers_[i].id)
                           {

                               if (!pose_has_been_added) // Add cam pose to subgraph
                               {
                                   visualizer_local.addReferenceFrame(vo.pose_, to_string(num_keyframes_local));
                                   slam_solver_local.addVertexAndEdge(vo.pose_, num_keyframes_local);
                                   pose_has_been_added = true;
                                   num_keyframes_local++;
                               }
                               else
                               {
                                   Aruco pose_to_add;
                                   pose_to_add.id = marker.id;
                                   pose_to_add.pose = marker_finder.marker_poses_[i];
                                   pose_to_add.pose(0, 3) = marker.x;
                                   pose_to_add.pose(1, 3) = marker.y;
                                   pose_to_add.pose(2, 3) = marker.z;
                                   pose_to_add.pose = pose_to_add.pose;

                                   Eigen::Affine3f tmp = pose_to_add.pose;

                                   printf(
                                       "pose antiga id %i: %f %f %f\n",
                                       marker_finder.markers_[i].id,
                                       marker_finder.marker_poses_[i](0, 3),
                                       marker_finder.marker_poses_[i](1, 3),
                                       marker_finder.marker_poses_[i](2, 3));
                                   printf(
                                       "pose nova id %i: %f %f %f\n", pose_to_add.id, tmp(0, 3), tmp(1, 3), tmp(2, 3));


                                   // Adding markers to subgraph
                                   visualizer_local.addReferenceFrame(
                                       vo.pose_ * marker_finder.marker_poses_[i], to_string(marker.id));

                                   slam_solver_local.addVertexAndEdge(
                                       vo.pose_ * marker_finder.marker_poses_[i], num_keyframes_local);
                                   slam_solver_local.addLoopClosingEdge(
                                       vo.pose_.inverse() * marker_finder.marker_poses_[i], num_keyframes_local);
                                   visualizer_local.addEdge(
                                       slam_solver_local.loop_edges_.back(), Eigen::Vector3f(1.0, 0.0, 0.0));
                                   visualizer_local.addEdge(slam_solver_local.odometry_edges_.back());

                                   num_keyframes_local++;

                                   // Adding previously marker to subgraph
                                   visualizer_local.viewReferenceFrame(tmp, to_string(marker.id) + "_FFFFF");

                                   slam_solver_local.addVertexAndEdge(
                                       vo.pose_ * pose_to_add.pose, num_keyframes_local);
                                   slam_solver_local.addLoopClosingEdge(
                                       vo.pose_.inverse() * pose_to_add.pose, num_keyframes_local);
                                   visualizer_local.addEdge(
                                       slam_solver_local.loop_edges_.back(), Eigen::Vector3f(1.0, 0.0, 0.0));
                                   visualizer_local.addEdge(slam_solver_local.odometry_edges_.back());

                                   num_keyframes_local++;

                                   // If we are in the last iteration optimize the subgraph
                                   // TODO If the origin num_keyframesarker is the last one found this will not be
                                   // called
                                   if (i == marker_finder.markers_.size() - 1)
                                   {
                                       printf(
                                           "pose antiga %f %f %f\n", vo.pose_(0, 3), vo.pose_(1, 3), vo.pose_(2, 3));
                                       slam_solver_local.optimizeGraph(10);
                                       visualizer_local.addOptimizedEdges(
                                           slam_solver_local.odometry_edges_, Eigen::Vector3f(1.0, 0.0, 1.0));
                                       visualizer_local.addOptimizedEdges(
                                           slam_solver_local.loop_edges_, Eigen::Vector3f(0.0, 1.0, 1.0));
                                       visualizer_local.addReferenceFrame(
                                           slam_solver_local.optimized_estimates_[0], "NOVA POSE");
                                       vo.pose_ = slam_solver_local.optimized_estimates_[0];
                                   }
                               }
                           }
                       }
                   }
               }*/
            }
            // if (slam_solver_local.optimized_estimates_.size() > 2) vo.pose_ =
            // slam_solver_local.optimized_estimates_[0];

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
            if (is_kf and !marker_found)
            {
                addVertexAndEdge(
                    vo.pose_,
                    last_keyframe_pose_odometry,
                    last_keyframe_pose_aruco,
                    Eigen::Affine3f::Identity(),
                    num_keyframes,
                    single_marker_slam,
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
    single_marker_slam.optimizeGraph(10);
    // visualizer.addOptimizedEdges(single_marker_slam.odometry_edges_, Eigen::Vector3f(1.0, 0.0, 1.0));
    // visualizer.addOptimizedEdges(single_marker_slam.loop_edges_, Eigen::Vector3f(0.0, 1.0, 1.0));
    visualizer.spin();
    return 0;
}

void addVertexAndEdge(
    Eigen::Affine3f cam_pose,
    Eigen::Affine3f& last_keyframe_pose_odometry,
    Eigen::Affine3f& last_keyframe_pose_aruco,
    Eigen::Affine3f aruco_pose,
    int& num_keyframes,
    PoseGraphSLAM& single_marker_slam,
    ReconstructionVisualizer& visualizer,
    bool is_loop_closure)

{
    double x = pow(cam_pose(0, 3) - last_keyframe_pose_odometry(0, 3), 2);
    double y = pow(cam_pose(1, 3) - last_keyframe_pose_odometry(1, 3), 2);
    double z = pow(cam_pose(2, 3) - last_keyframe_pose_odometry(2, 3), 2);

    // We will only add a new keyframe if they have at least 5cm of distance between each other
    if (sqrt(x + y + z) >= 0.1 and is_loop_closure == false)
    {
        // Adding keyframe in visualizer
        visualizer.viewReferenceFrame(cam_pose, to_string(num_keyframes));
        // Add the keyframe and creating an edge to the last vertex
        single_marker_slam.addVertexAndEdge(cam_pose, num_keyframes);
        // visualizer.addEdge(single_marker_slam.odometry_edges_.back());

        last_keyframe_pose_odometry = cam_pose;
        num_keyframes++; // Increment the number of keyframes found
    }
    // If the keyframe is a loop closure we will create a loop closing edge
    x = pow(cam_pose(0, 3) - last_keyframe_pose_aruco(0, 3), 2);
    y = pow(cam_pose(1, 3) - last_keyframe_pose_aruco(1, 3), 2);
    z = pow(cam_pose(2, 3) - last_keyframe_pose_aruco(2, 3), 2);
    if (sqrt(x + y + z) >= 0.1 and is_loop_closure == true)
    {
        // Adding keyframe in visualizer
        visualizer.viewReferenceFrame(cam_pose, to_string(num_keyframes));
        // Add the keyframe and creating an edge to the last vertex
        single_marker_slam.addVertexAndEdge(cam_pose, num_keyframes);
        // visualizer.addEdge(single_marker_slam.getLastEdge("odometry"));
        // Adding the loop closing edge that is an edge from this vertex to the initial
        // If we want to change the system coord from A to B -> A.inverse * B
        single_marker_slam.addLoopClosingEdge(cam_pose.inverse() * aruco_pose, num_keyframes);
        // visualizer.addEdge(single_marker_slam.loop_edges_.back(), Eigen::Vector3f(1.0, 0.0, 0.0));
        // Make a optimization in the graph from every 20 loop edges
        // if (single_marker_slam.loop_edges_.size() % 20 == 0)
        //{
        // visualizer.removeEdges(single_marker_slam.odometry_edges_);
        // visualizer.removeEdges(single_marker_slam.loop_edges_);
        // visualizer.resetVisualizer();

        //   single_marker_slam.optimizeGraph(10);
        // visualizer.addOptimizedEdges(single_marker_slam.odometry_edges_, Eigen::Vector3f(1.0, 0.0, 1.0));
        // visualizer.addOptimizedEdges(single_marker_slam.loop_edges_, Eigen::Vector3f(0.0, 1.0, 1.0));
        // }
        last_keyframe_pose_aruco = cam_pose;
        num_keyframes++; // Increment the number of keyframes found
    }
}

void loadMarkersFound(string aruco_poses_file)
{
    printf("Loading markers previously mapped\n");
    Pose pose;
    ifstream load_file;
    load_file.open(aruco_poses_file);
    if (!load_file)
    {
        printf("Unable to open file, check if the file exists");
        exit(1);
    }
    int size = 0;
    // fill waypoints list
    load_file >> size;
    for (int i = 0; i < size; i++)
    {
        load_file >> pose.id;
        load_file >> pose.x;
        load_file >> pose.y;
        load_file >> pose.z;
        printf("Marker number %i -> x: %f y: %f z: %f\n", pose.id, pose.x, pose.y, pose.z);
        markers_found.push_back(pose);
    }
    load_file.close();
}