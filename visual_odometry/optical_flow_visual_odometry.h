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

#ifndef INCLUDE_OPTICAL_FLOW_VISUAL_ODOMETRY_H_
#define INCLUDE_OPTICAL_FLOW_VISUAL_ODOMETRY_H_

#include <Eigen/Geometry>
#include <map>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>

#include <common_types.h>
#include <motion_estimator_ransac.h>
#include <wide_baseline_tracker.h>

class OpticalFlowVisualOdometry
{
private:
    // Current frame index
    unsigned long int frame_idx_;

    // Adds a new keyframe to the internal container of keyframes
    void addKeyFrame(const cv::Mat& rgb);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Previous dense point cloud
    pcl::PointCloud<PointT>::Ptr prev_dense_cloud_;

    // Current dense point cloud
    pcl::PointCloud<PointT>::Ptr curr_dense_cloud_;

    // Feature tracker
    WideBaselineTracker tracker_;

    // Motion estimator
    MotionEstimatorRANSAC motion_estimator_;

    // Current camera pose
    Eigen::Affine3f pose_;

    // Container with pairs idx <-> keyframe
    std::map<size_t, Keyframe> keyframes_;

    /**
     * Default constructor
     * @param initialPose start pose of the camera, identity if nothing is passed
     */
    OpticalFlowVisualOdometry(const Eigen::Affine3f& initialPose = Eigen::Affine3f::Identity());

    /**
     * Constructor with the matrix of intrinsic parameters
     * @param intr camera parameters
     * @param initialPose start pose of the camera, identity if nothing is passed
     */
    OpticalFlowVisualOdometry(const Intrinsics& intr, const Eigen::Affine3f& initialPose = Eigen::Affine3f::Identity());

    /**
     * Main member function: computes the current camera pose.
     * Returns true if the current frame is a keyframe.
     * @param rgb image @param depth image
     * @param return a boolean
     */
    bool computeCameraPose(const cv::Mat& rgb, const cv::Mat& depth);

    /**
     * @param Keyframe return the last detected keyframe.
     */
    Keyframe getLastKeyframe();
};

#endif /* INCLUDE_OPTICAL_FLOW_VISUAL_ODOMETRY_H_ */