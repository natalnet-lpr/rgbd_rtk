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
#include <math.h>
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

void addOptimizedEdges(SLAM_Solver& slam_solver, ReconstructionVisualizer visualizer);
void removeEdges(SLAM_Solver& slam_solver, ReconstructionVisualizer visualizer);
bool isOrientationCorrect(Eigen::Affine3f& first, Eigen::Affine3f newone);

/**
 * Adds vertix and edges in slam_solver and visualizer
 * @param new_keyframe_pose new keyframe that should be added
 * @param last_keyframe_pose_odometry the last keyframe added in graph, this is update this in this function
 * @param last_keyframe_pose_aruco the last keyframe added in graph, this is update this in this function
 * @param first_keyframe_pose the first keyframe
 * @param num_keyframes number of keyframes in graph, this is update in this function
 * @param slam_solver slam_solver reference
 * @param visualizer visualizer reference
 * @param is_loop_closure if the vertex added is a loop_closure as well
 */
void addVertixAndEdge(
    Eigen::Affine3f cam_pose,
    Eigen::Affine3f& last_keyframe_pose_odometry,
    Eigen::Affine3f& last_keyframe_pose_aruco,
    Eigen::Affine3f aruco_pose,
    int& num_keyframes,
    SLAM_Solver& slam_solver,
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
    SLAM_Solver slam_solver;
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

                    // Adding first pose to slam_solver and visualizer
                    slam_solver.addVertexAndEdge(vo.pose_, num_keyframes);
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
                        addVertixAndEdge(
                            vo.pose_,
                            last_keyframe_pose_odometry,
                            last_keyframe_pose_aruco,
                            marker_finder.marker_poses_[j],
                            num_keyframes,
                            slam_solver,
                            visualizer,
                            true,
                            config_params);
                        marker_found = true;
                    }
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
            visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.02, vo.pose_);
            visualizer.addQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);

            // If we found a keyframe we will added to slam solver and visualizer
            if (is_kf and !marker_found)
            {
                addVertixAndEdge(
                    vo.pose_,
                    last_keyframe_pose_odometry,
                    last_keyframe_pose_aruco,
                    Eigen::Affine3f::Identity(),
                    num_keyframes,
                    slam_solver,
                    visualizer,
                    false,
                    config_params);
            }
        }

        visualizer.spinOnce();
        imshow("Image view", frame);
        // imshow("Depth view", depth);
        char key = waitKey(10);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            logger.print(EventLogger::L_INFO, "[slam_solver_test.cpp] Exiting\n", argv[0]);
            break;
        }
    }

    slam_solver.optimizeGraph(10);
    // visualizer.addOptimizedEdges(slam_solver.odometry_edges_, Eigen::Vector3f(1.0, 0.0, 1.0));
    //  visualizer.addOptimizedEdges(slam_solver.loop_edges_, Eigen::Vector3f(0.0, 1.0, 1.0));
    visualizer.spin();
    return 0;
}

void addVertixAndEdge(
    Eigen::Affine3f cam_pose,
    Eigen::Affine3f& last_keyframe_pose_odometry,
    Eigen::Affine3f& last_keyframe_pose_aruco,
    Eigen::Affine3f aruco_pose,
    int& num_keyframes,
    SLAM_Solver& slam_solver,
    ReconstructionVisualizer& visualizer,
    bool is_loop_closure,
    ConfigParams config_params)

{
    double x = pow(cam_pose(0, 3) - last_keyframe_pose_odometry(0, 3), 2);
    double y = pow(cam_pose(1, 3) - last_keyframe_pose_odometry(1, 3), 2);
    double z = pow(cam_pose(2, 3) - last_keyframe_pose_odometry(2, 3), 2);

    // We will only add a new keyframe if they have at least 5cm of distance between each other
    if (sqrt(x + y + z) >= config_params.minimum_distance_between_keyframes and is_loop_closure == false)
    {
        // Adding keyframe in visualizer
        visualizer.viewReferenceFrame(cam_pose, to_string(num_keyframes));
        // Add the keyframe and creating an edge to the last vertex
        slam_solver.addVertexAndEdge(cam_pose, num_keyframes);
        slam_solver.getEdge(
            slam_solver.num_vertices_ - 2, slam_solver.num_vertices_ - 1, edge_from, edge_to, edge_name);
        visualizer.addEdge(edge_from, edge_to, edge_name);

        last_keyframe_pose_odometry = cam_pose;
        num_keyframes++; // Increment the number of keyframes found
    }
    // If the keyframe is a loop closure we will create a loop closing edge
    x = pow(cam_pose(0, 3) - last_keyframe_pose_aruco(0, 3), 2);
    y = pow(cam_pose(1, 3) - last_keyframe_pose_aruco(1, 3), 2);
    z = pow(cam_pose(2, 3) - last_keyframe_pose_aruco(2, 3), 2);
    if (sqrt(x + y + z) >= config_params.minimum_distance_between_keyframes and is_loop_closure == true)
    {
        // Adding keyframe in visualizer
        visualizer.viewReferenceFrame(cam_pose, to_string(num_keyframes));
        // Add the keyframe and creating an edge to the last vertex
        slam_solver.addVertexAndEdge(cam_pose, num_keyframes);
        slam_solver.getEdge(
            slam_solver.num_vertices_ - 2, slam_solver.num_vertices_ - 1, edge_from, edge_to, edge_name);
        visualizer.addEdge(edge_from, edge_to, edge_name);

        // Adding the loop closing edge that is an edge from this vertex to the initial
        // If we want to change the system coord from A to B -> A.inverse * B
        // A = cam pose no sistema de coordenadas do Aruco = P
        // B = origem
        slam_solver.addLoopClosingEdge(cam_pose.inverse() * aruco_pose, num_keyframes);
        slam_solver.getEdge(slam_solver.num_vertices_ - 1, 0, edge_from, edge_to, edge_name);
        visualizer.addEdge(edge_from, edge_to, edge_name, Eigen::Vector3f(1.0, 0.0, 0.0));

        slam_solver.loop_closure_edges_name.push_back(edge_name); // Saving the edge name of the loop closure edges
        // Make a optimization in the graph from every 20 loop edges
        if (slam_solver.num_loop_edges_ % config_params.local_optimization_threshold == 0)
        {
            removeEdges(slam_solver, visualizer);
            visualizer.resetVisualizer();

            slam_solver.optimizeGraph(10);
            addOptimizedEdges(slam_solver, visualizer);
        }

        last_keyframe_pose_aruco = cam_pose;
        num_keyframes++; // Increment the number of keyframes found
    }
}

bool isOrientationCorrect(Eigen::Affine3f& first, Eigen::Affine3f newone)
{
    double angle = acos((first(0, 2) * newone(0, 2)) + (first(1, 2) * newone(1, 2)) + (first(2, 2) * newone(2, 2))) *
                   180.0 / 3.1315;

    return angle > 10 ? false : true;
}

void removeEdges(SLAM_Solver& slam_solver, ReconstructionVisualizer visualizer)
{
    for (int i = 0; i < slam_solver.num_vertices_; i++)
    {
        slam_solver.getEdge(i, i + 1, edge_from, edge_to, edge_name);
        visualizer.removeEdge(edge_name);
    }
    for (int i = 0; i < slam_solver.loop_closure_edges_name.size(); i++)
    {
        visualizer.removeEdge(string("edge_") + slam_solver.loop_closure_edges_name[i]);
    }
}

void addOptimizedEdges(SLAM_Solver& slam_solver, ReconstructionVisualizer visualizer)
{
    for (int i = 0; i < slam_solver.num_vertices_; i++)
    {
        slam_solver.getOptimizedEdge(i, i + 1, edge_from, edge_to, edge_name);
        // visualizer.addOptimizedEdges(edge_from, edge_to, edge_name, Eigen::Vector3f(1.0, 0.0, 1.0));
    }
    for (int i = 0; i < slam_solver.loop_closure_edges_name.size(); i++)
    {
        string copy = edge_name.erase(0, 15);
        cout << "loop closure edge first Id: " << copy.substr(0, copy.find('_'))
             << " second id:" << copy.substr(copy.find('_') + 1, 1) << endl;
        /*
        slam_solver.getOptimizedEdge(
            copy.substr(0, copy.find('_')),
            copy.substr(copy.find('_'), copy.size()),
            edge_from,
            edge_to,
            edge_name.erase(0, 15));
         visualizer.addOptimizedEdges(edge_from, edge_to, edge_name, Eigen::Vector3f(0.0, 1.0, 1.0));
         */
    }
}
