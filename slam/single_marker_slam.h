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
 *  Bruno Silva
 *  Rodrigo Xavier
 */

#ifndef INCLUDE_SINGLEMARKERSLAM_H_
#define INCLUDE_SINGLEMARKERSLAM_H_

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>

#include <common_types.h>
#include <optical_flow_visual_odometry.h>
#include <marker_finder.h>
#include <pose_graph_slam.h>

class SingleMarkerSLAM
{
private:
    
    // Informs if the initial ref. frame was set
    //(i.e. if the marker was found for the first time)
    bool initialized_;

    // Informs if the marker was found in the last image
    bool marker_found_;

    // Number of loop closures in the current optimization cycle
    size_t curr_loop_closures_;

    // Algorithm parameter: max. valid distance bw. the marker
    //and the camera (results in marker poses of better quality)
    const float max_marker_dist_;

    // Algorithm parameter: min. distance bw. consecutive keyframes
    //(avoids creating a keyframe close to another)
    const float min_kf_dist_;

    // Algorithm parameter: number of optimization iterations
    const size_t opt_iterations_;

    // Algorithm parameter: number of loop closures to trigger an
    // optimization
    const size_t opt_loop_closures_;

    // Container with pairs index <-> keyframe
    std::map<size_t, Keyframe> keyframes_;

    // Last odom. keyframe pose
    Eigen::Affine3f last_odom_kf_pose_;

    // Last loop closure (marker found) keyframe pose
    Eigen::Affine3f last_lc_kf_pose_;

    // Fist camera pose from the AR marker
    Eigen::Affine3f first_pose_from_AR_;

    // Last camera pose from the AR marker
    Eigen::Affine3f last_pose_from_AR_;

    // Transformation from visual odometry coord. system to SLAM coord. system
    Eigen::Affine3f vo_correction_;

    // Computes visual odometry from RGB-D images
    OpticalFlowVisualOdometry vo_;

    // Finds the artificial marker to build graph restrictions
    MarkerFinder marker_finder_;

    // Used to solve the optimization
    PoseGraphSLAM slam_;

    /**
     * Creates and stores a Keyframe.
     *
     */
    void _addKeyframe();

    /**
     * Updates all keyframes with the poses
     * obtained from the graph vertices after optimization.
     */
    void _updateKeyframes();

    /**
     * Gets the pose of the last node added to the
     * graph (which should be equal to the visual odometry pose
     * if errors were not present).
     */
    Eigen::Affine3f _lastVertexPose();

    /**
     * Applies a transformation to the visual odometry
     * ref. frame to align it with the SLAM ref. frame.
     */
    void _correctVisualOdometry();

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //Utility struct to aid loading parameters from config. files
    struct Parameters
    {
        int marker_id_; //id of the marker to be used in the optimization
        int opt_iterations_; //number of iterations in each optimization
        float min_dist_bw_keyframes_; //minimum distance between keyframes to consider a new keyframe
        int opt_loop_closures_; //number of loop closures to trigger an optimization
    };

    /**
     * Builds a SingleMarkerSLAM with
     * default parameters.
     */
//    SingleMarkerSLAM();

    /**
     * TODO: add tracking and visual odometry parameters
     * Builds a SingleMarkerSLAM with
     * the given parameters.
     * @parameter tracker_param: FeatureTracker parameters
     * @parameter mf_param: MarkerFinder parameters
     * @parameter slam_param: SingleMarkerSLAM parameters
     */
    SingleMarkerSLAM(const FeatureTracker::Parameters &tracker_param,
                     const MarkerFinder::Parameters &mf_param,
                     const SingleMarkerSLAM::Parameters &slam_param);

    /**
     * Adds a vertex with the given pose and
     * an edge from the previous vertex to the
     * new vertex.
     * The edge transform is the odometry transform
     * (relative transformation between the two vertices).
     * @parameter pose: vertex pose
     */
    void addVertexAndEdge(const Eigen::Affine3f& pose);

    /**
     * Adds a loop closing edge connecting
     * the last added vertex to the first vertex.
     * The edge transform is the transform computed by the
     * AR marker pose
     * (relative transformation between the two vertices as
     *  measured by the marker).
     * @parameter transform: tranformation bw. the last and first
     *                       vertices
     */
    void addLoopClosingEdge(const Eigen::Affine3f &transform);

    /**
     * Main member function: process a RGB-D image
     * considering the marker with given id as reference.
     * @parameter rgb: RGB image
     * @parameter depth: depth image
     * @parameter marker_id: id of the ref. marker
     * @return true if the processed image is a keyframe (e.g. a
               vertex was created)
     */
    bool processImage(const cv::Mat &rgb, const cv::Mat &depth,
                      const int &marker_id);

    /**
     * Returns true if the sought marker
     * was found in the image passed to
     * processImage method
     * (processImage must be called first).
     */
    bool isMarkerFound();

    /**
     * Returns a read-only reference
     * to the stored keyframes.
     */
    const std::map<size_t, Keyframe>& keyframes();

    /**
     * Returns a read-only reference
     * to the last keyframe.
     */
    const Keyframe& lastKeyframe();

    /**
     * Returns the current
     * visual odometry pose.
     */
    Eigen::Affine3f visualOdometryPose();

    /**
     * Returns the last computed
     * pose from the AR marker.
     */
    Eigen::Affine3f markerPose();

    /**
     * Returns the current
     * visual odometry point cloud.
     */
    pcl::PointCloud<PointT>::Ptr visualOdometryPointCloud();
};

#endif /* INCLUDE_SINGLEMARKERSLAM_H_ */
