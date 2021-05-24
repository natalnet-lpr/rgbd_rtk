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
#include <single_marker_slam.h>

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

struct ConfigParams
{
    string index_file;
    string camera_calibration_file;
    string aruco_dic;
    float aruco_marker_size;
    float aruco_max_distance = 4;
    float minimum_distance_between_keyframes = 0.05;
    int local_optimization_threshold = 20; // How many loop edges to make a optimization
};
Eigen::Vector3d edge_from, edge_to;
string edge_name;

void addOptimizedEdges(SingleMarkerSlam& single_marker_slam, ReconstructionVisualizer& visualizer);
void removeEdges(SingleMarkerSlam& single_marker_slam, ReconstructionVisualizer& visualizer);
bool isOrientationCorrect(Eigen::Affine3f first, Eigen::Affine3f newone);
void updatePointCloud(
    ReconstructionVisualizer& visualizer,
    OpticalFlowVisualOdometry& vo,
    SingleMarkerSlam& single_marker_slam);
/**
 * Adds vertix and edges in single_marker_slam and visualizer
 * @param new_keyframe_pose new keyframe that should be added
 * @param last_keyframe_pose_odometry the last keyframe added in graph, this is update this in this function
 * @param last_keyframe_pose_aruco the last keyframe added in graph, this is update this in this function
 * @param first_keyframe_pose the first keyframe
 * @param num_keyframes number of keyframes in graph, this is update in this function
 * @param single_marker_slam single_marker_slam reference
 * @param visualizer visualizer reference
 * @param is_loop_closure if the vertex added is a loop_closure as well
 */
void addVertixAndEdge(
    OpticalFlowVisualOdometry& vo,
    Eigen::Affine3f& last_keyframe_pose_odometry,
    Eigen::Affine3f& last_keyframe_pose_aruco,
    Eigen::Affine3f aruco_pose,
    int& num_keyframes,
    SingleMarkerSlam& single_marker_slam,
    ReconstructionVisualizer& visualizer,
    bool is_loop_closure,
    ConfigParams config_params);
/**
 * This program shows the use of camera pose estimation using optical flow visual odometry.
 * @param .yml config. file (from which index_file is used)
 */
int main(int argc, char** argv)
{
    int num_keyframes = 0;
    bool slam_solver_started = false;
    bool marker_found;
    ConfigParams config_params;
    EventLogger& logger = EventLogger::getInstance();
    logger.setVerbosityLevel(EventLogger::L_ERROR);

    ReconstructionVisualizer visualizer;
    SingleMarkerSlam single_marker_slam;
    Intrinsics intr(0);
    OpticalFlowVisualOdometry vo(intr);
    MarkerFinder marker_finder;
    RGBDLoader loader;
    Mat frame, depth;
    Eigen::Affine3f last_keyframe_pose_odometry; // Last odometry keyframe
    Eigen::Affine3f last_keyframe_pose_aruco;    // Last Aruco Keyframe
    Eigen::Affine3f first_aruco_pose;            // First Keyframe

    // Slam solver will start when the marker is found for the first time
    if (argc != 2)
    {
        logger.print(EventLogger::L_ERROR, "[slam_solver_test.cpp] Usage: %s <path/to/config_file.yaml>\n", argv[0]);
        exit(0);
    }

    ConfigLoader param_loader(argv[1]);
    param_loader.checkAndGetFloat("aruco_marker_size", config_params.aruco_marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", config_params.aruco_max_distance);
    param_loader.checkAndGetInt("local_optimization_threshold", config_params.local_optimization_threshold);
    param_loader.checkAndGetString("index_file", config_params.index_file);
    param_loader.checkAndGetFloat(
        "minimum_distance_between_keyframes", config_params.minimum_distance_between_keyframes);
    param_loader.checkAndGetString("camera_calibration_file", config_params.camera_calibration_file);
    param_loader.checkAndGetString("aruco_dic", config_params.aruco_dic);

    marker_finder.markerParam(
        config_params.camera_calibration_file, config_params.aruco_marker_size, config_params.aruco_dic);

    loader.processFile(config_params.index_file);

    // Compute visual odometry on each image
    for (int i = 0; i < loader.num_images_; i++)
    {
        // Load RGB-D image
        loader.getNextImage(frame, depth);

        // If aruco is not found yet
        if (slam_solver_started == false)
        {
            marker_finder.detectMarkersPoses(frame, Eigen::Affine3f::Identity(), config_params.aruco_max_distance);
            if (marker_finder.markers_.size() > 0)
            {
                if (250 == marker_finder.markers_[i].id)
                {
                    // If we found the mark for the first time, we will add the marker as the origin
                    // of the system Then we add the first camera pose related to the marker pose.
                    // Getting Camera Pose

                    vo.pose_ = marker_finder.marker_poses_[i].inverse();
                    first_aruco_pose = marker_finder.marker_poses_[i];
                    last_keyframe_pose_odometry = vo.pose_;
                    last_keyframe_pose_aruco = vo.pose_;

                    // Adding first pose to single_marker_slam and visualizer
                    single_marker_slam.addVertexAndEdge(vo.pose_, num_keyframes);
                    visualizer.addReferenceFrame(vo.pose_, to_string(num_keyframes));
                    num_keyframes++; // Increment the number of keyframes found
                    // Set slam solver started to true since we found the marker for the first time
                    slam_solver_started = true;
                }
            }
            continue;
        }

        // If we have already found the marker we start the keyframe process
        else
        {
            // Reset marker finding boolean
            marker_found = false;

            marker_finder.detectMarkersPoses(frame, Eigen::Affine3f::Identity(), config_params.aruco_max_distance);
            for (size_t j = 0; j < marker_finder.markers_.size(); j++)
            {

                marker_finder.markers_[j].draw(frame, Scalar(0, 0, 255), 1.5);
                CvDrawingUtils::draw3dAxis(frame, marker_finder.markers_[j], marker_finder.camera_params_, 2);

                if (250 == marker_finder.markers_[j].id)
                {
                    if (isOrientationCorrect(first_aruco_pose, marker_finder.marker_poses_[j]))
                    {
                        vo.computeCameraPose(frame, depth, true) 
                        addVertixAndEdge(
                            vo,
                            last_keyframe_pose_odometry,
                            last_keyframe_pose_aruco,
                            marker_finder.marker_poses_[j],
                            num_keyframes,
                            single_marker_slam,
                            visualizer,
                            true,
                            config_params);
                        marker_found = true;
                    }
                }
            }

            if(!marker_found){
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
                visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.02, vo.pose_);

                // If we found a keyframe we will added to slam solver and visualizer
                if (is_kf)
                {
                    addVertixAndEdge(
                        vo,
                        last_keyframe_pose_odometry,
                        last_keyframe_pose_aruco,
                        Eigen::Affine3f::Identity(),
                        num_keyframes,
                        single_marker_slam,
                        visualizer,
                        false,
                        config_params);
                    // Adding a Keyframe to visualizer, this will save the point cloud
                    visualizer.addKeyFrame(vo.keyframes_[vo.keyframes_.size()], to_string(vo.keyframes_.size()));
                }

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

    removeEdges(single_marker_slam, visualizer);
    visualizer.removeAllVertexesAndEdges();
    single_marker_slam.optimizeGraph(10);
    addOptimizedEdges(single_marker_slam, visualizer);
    visualizer.spin();
    return 0;
}

void addVertixAndEdge(
    OpticalFlowVisualOdometry& vo,
    Eigen::Affine3f& last_keyframe_pose_odometry,
    Eigen::Affine3f& last_keyframe_pose_aruco,
    Eigen::Affine3f aruco_pose,
    int& num_keyframes,
    SingleMarkerSlam& single_marker_slam,
    ReconstructionVisualizer& visualizer,
    bool is_loop_closure,
    ConfigParams config_params)

{
    double x = pow(vo.pose_(0, 3) - last_keyframe_pose_odometry(0, 3), 2);
    double y = pow(vo.pose_(1, 3) - last_keyframe_pose_odometry(1, 3), 2);
    double z = pow(vo.pose_(2, 3) - last_keyframe_pose_odometry(2, 3), 2);

    // We will only add a new keyframe if they have at least 5cm of distance between each other
    if (sqrt(x + y + z) >= config_params.minimum_distance_between_keyframes and is_loop_closure == false)
    {
        // Adding keyframe in visualizer
        visualizer.viewReferenceFrame(vo.pose_, to_string(num_keyframes));
        // Add the keyframe and creating an edge to the last vertex
        single_marker_slam.addVertexAndEdge(vo.pose_, num_keyframes);
        single_marker_slam.getEdge(
            single_marker_slam.num_vertices_ - 2, single_marker_slam.num_vertices_ - 1, edge_from, edge_to, edge_name);
        visualizer.addEdge(edge_from, edge_to, edge_name);

        last_keyframe_pose_odometry = vo.pose_;
        num_keyframes++; // Increment the number of keyframes found
    }
    // If the keyframe is a loop closure we will create a loop closing edge
    x = pow(vo.pose_(0, 3) - last_keyframe_pose_aruco(0, 3), 2);
    y = pow(vo.pose_(1, 3) - last_keyframe_pose_aruco(1, 3), 2);
    z = pow(vo.pose_(2, 3) - last_keyframe_pose_aruco(2, 3), 2);
    if (sqrt(x + y + z) >= config_params.minimum_distance_between_keyframes and is_loop_closure == true)
    {
        // Adding keyframe in visualizer
        visualizer.viewReferenceFrame(vo.pose_, to_string(num_keyframes));
        // Add the keyframe and creating an edge to the last vertex
        single_marker_slam.addVertexAndEdge(vo.pose_, num_keyframes);
        single_marker_slam.getEdge(
            single_marker_slam.num_vertices_ - 2, single_marker_slam.num_vertices_ - 1, edge_from, edge_to, edge_name);
        visualizer.addEdge(edge_from, edge_to, edge_name);

        // Adding the loop closing edge that is an edge from this vertex to the initial
        // If we want to change the system coord from A to B -> A.inverse * B
        // A = cam pose no sistema de coordenadas do Aruco = P
        // B = origem
        single_marker_slam.addLoopClosingEdge(vo.pose_.inverse() * aruco_pose, num_keyframes);
        single_marker_slam.getEdge(single_marker_slam.num_vertices_ - 1, 0, edge_from, edge_to, edge_name);
        visualizer.addEdge(edge_from, edge_to, edge_name, Eigen::Vector3f(1.0, 0.0, 0.0));

        single_marker_slam.loop_closure_edges_name_.push_back(
            edge_name); // Saving the edge name of the loop closure edges
        // Make a optimization in the graph from every 20 loop edges
        if (single_marker_slam.num_loop_edges_ % config_params.local_optimization_threshold == 0)
        {
            removeEdges(single_marker_slam, visualizer);
            visualizer.removeAllVertexesAndEdges();

            single_marker_slam.optimizeGraph(10);
            addOptimizedEdges(single_marker_slam, visualizer);
            updatePointCloud(visualizer, vo, single_marker_slam);
        }

        last_keyframe_pose_aruco = vo.pose_;
        num_keyframes++; // Increment the number of keyframes found
    }
}

/**
 * Check if the orientation of the aruco pose found is correct, if the orientation changes a lot related with the first
 * one detected, we ignore
 * @param first First pose of aruco
 * @param new_pose new aruco pose
 */
bool isOrientationCorrect(Eigen::Affine3f first, Eigen::Affine3f new_pose)
{
    double angle =
        acos((first(0, 2) * new_pose(0, 2)) + (first(1, 2) * new_pose(1, 2)) + (first(2, 2) * new_pose(2, 2))) * 180.0 /
        3.1315;

    return angle > 10 ? false : true;
}

/**
 * Remove edges from visualizer
 * @param single_marker_slam
 * @param visualizer
 */
void removeEdges(SingleMarkerSlam& single_marker_slam, ReconstructionVisualizer& visualizer)
{
    // Removing the edges
    for (signed i = 0; i < single_marker_slam.num_vertices_; i++)
    {
        single_marker_slam.getEdge(i, i + 1, edge_from, edge_to, edge_name);
        visualizer.removeEdge(edge_name);
    }
    for (size_t i = 0; i < single_marker_slam.loop_closure_edges_name_.size(); i++)
    {
        visualizer.removeEdge(single_marker_slam.loop_closure_edges_name_[i]);
    }
}

/**
 * Iterate over the number of vertices to add odometry optimization edges
 * and Iterate over the loop edges to add loop edges odometry
 * @param single_marker_slam
 * @param visualizer
 */
void addOptimizedEdges(SingleMarkerSlam& single_marker_slam, ReconstructionVisualizer& visualizer)
{
    // Iterate over number of vertices and get the optimized edge and adding to visualizer
    for (signed i = 0; i < single_marker_slam.num_vertices_ - 1; i++)
    {
        single_marker_slam.getOptimizedEdge(i, i + 1, edge_from, edge_to, edge_name);
        visualizer.addEdge(edge_from, edge_to, edge_name, Eigen::Vector3f(1.0, 0.0, 1.0));
    }
    // Iterate over number of vertices and get the optimized loop edge and adding to visualizer
    for (size_t i = 0; i < single_marker_slam.loop_closure_edges_name_.size() - 1; i++)
    {
        // name of edge is saved as "edge_fromid_toid"
        string copy =
            single_marker_slam.loop_closure_edges_name_[i].erase(0, 5); // Removing the first 5 strings "edge_"

        string from_id = copy.substr(0, copy.find('_'));
        string to_id = copy.substr(copy.find('_') + 1, 1);
        if (from_id != "" && to_id != "")
        {
            single_marker_slam.getOptimizedEdge(stoi(from_id), stoi(to_id), edge_from, edge_to, edge_name);
            visualizer.addEdge(edge_from, edge_to, edge_name, Eigen::Vector3f(0.0, 1.0, 1.0));
        }
    }
}

void updatePointCloud(
    ReconstructionVisualizer& visualizer,
    OpticalFlowVisualOdometry& vo,
    SingleMarkerSlam& single_marker_slam)
{
    vector<Keyframe> v;

    for (auto& x : vo.keyframes_)
    {
        // Iterate over the vector of keyframes, then get the id of each keyframe and get the optimized vertix of this
        // keyframe id
        Eigen::Affine3f vertixPose = single_marker_slam.getVertix(x.second.idx_);
        Eigen::Affine3f new_estimate = Eigen::Affine3f::Identity();

        // Update the pose of the keyframe with the new pose and adding to the new vector of keyframes
        new_estimate(0, 0) = vertixPose(0, 0);
        new_estimate(0, 1) = vertixPose(0, 1);
        new_estimate(0, 2) = vertixPose(0, 2);
        new_estimate(0, 3) = vertixPose(0, 3);
        new_estimate(1, 0) = vertixPose(1, 0);
        new_estimate(1, 1) = vertixPose(1, 1);
        new_estimate(1, 2) = vertixPose(1, 2);
        new_estimate(1, 3) = vertixPose(1, 3);
        new_estimate(2, 0) = vertixPose(2, 0);
        new_estimate(2, 1) = vertixPose(2, 1);
        new_estimate(2, 2) = vertixPose(2, 2);
        new_estimate(2, 3) = vertixPose(2, 3);

        x.second.pose_ = new_estimate;
        v.push_back(x.second);
    }
    visualizer.updateKeyframes(v); // Updating keyframes in visualizer
}
