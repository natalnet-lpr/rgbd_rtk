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
 *  Authors:
 *
 *  Bruno Silva
 */

#ifndef INCLUDE_VISUAL_ODOMETRY_H_
#define INCLUDE_VISUAL_ODOMETRY_H_

#include <vector>
#include <map>
#include <fstream>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>

#include <common_types.h>
#include <feature_tracker.h>
#include <motion_estimator_ransac.h>

/**
 *  Abstract Base Class for Visual Odometry.
 *  Contains members that are comon to all implementations
 *  and virtual member functions that should be implemented
 *  by each concrete Visual Odometry class.
 */ 
class VisualOdometry
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //Struct with Visual Odometry parameters
    struct Parameters
    {
        FeatureTracker::Parameters tracker_param_;
        Intrinsics intr_;
        float ransac_thr_;
    };

    // Previous dense point cloud
    pcl::PointCloud<PointT>::Ptr prev_dense_cloud_;

    // Current dense point cloud
    pcl::PointCloud<PointT>::Ptr curr_dense_cloud_;

    // Feature tracker
    cv::Ptr<FeatureTracker> tracker_ptr_;

    // Motion estimator
    MotionEstimatorRANSAC motion_estimator_;

    // Current camera pose
    Eigen::Affine3f pose_;

    /**
     * Builds a Visual Odometry algorithm with the
     * given parameters
     * @param vo_param: VisualOdometry parameters
     * @param initialPose: Visual Odometry initial pose
     */
    VisualOdometry(const VisualOdometry::Parameters &vo_param,
                   const Eigen::Affine3f &initialPose = Eigen::Affine3f::Identity());

    virtual ~VisualOdometry()
    {
        poses_file_.close();
    }

    /**
     * Main member function: computes the current camera pose
     * @param rgb: RGB image
     * @param depth: depth image
     * @param time_stamp: the time that the pose was computed
     */ 
    virtual bool computeCameraPose(const cv::Mat &rgb, const cv::Mat &depth, const std::string &time_stamp = "") = 0;

    /**
     * Utility function: creates and returns a Keyframe
     * with the given id.
     * computePose must be called first, so
     * the last supplied data to computePose is
     * used to create a keyframe.
     * @param kf_id: id of the keyframe
     */
    Keyframe createKeyframe(const size_t &kf_id);

protected:

    // Current frame index
    size_t frame_idx_;

    // Copy of the current RGB image (for Keyframe creation)
    cv::Mat rgb_;

    // Copy of the current depth image (for Keyframe creation)
    cv::Mat depth_;

    // Output file computed poses
    std::ofstream poses_file_;

    /**
     * Writes the last computed visual odometry pose to file.
     * @param time_stamp, the time that the pose was computed. 
     */
    void _writePoseToFile(const std::string &time_stamp);

};

#endif /* INCLUDE_VISUAL_ODOMETRY_H_ */