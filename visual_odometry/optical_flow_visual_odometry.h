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

#ifndef INCLUDE_OPTICAL_FLOW_VISUAL_ODOMETRY_H_
#define INCLUDE_OPTICAL_FLOW_VISUAL_ODOMETRY_H_

#include <Eigen/Geometry>
#include <map>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>


#include <common_types.h>
#include <motion_estimator_ransac.h>
#include <klt_tracker.h>
#include <klttw_tracker.h>

class OpticalFlowVisualOdometry
{
private:
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

public:
    void writePoseToFile(const std::string& time_stamp);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
     * Constructs a Visual Odometry given the matrix of intrinsic parameters,
     * tracker parameters, motion estimation parameter (RANSAC) and initial pose.
     * @param intr intrinsic camera parameters
     * @param tracking_param tracking parameters
     * @param ransac_thr threshold for RANSAC (a correspondence is accepted if < thr)
     * @param initialPose initial pose of the camera, identity if nothing is passed
     */
    OpticalFlowVisualOdometry(const Intrinsics& intr, const FeatureTracker::Parameters& tracking_param,
                              const float& ransac_thr,
                              const Eigen::Affine3f& initialPose = Eigen::Affine3f::Identity());

    /**
     * Main member function: computes the current camera pose.
     * Returns true if the current frame is a keyframe.
     * @param rgb image @param depth image
     * @param return a boolean
     */

    ~OpticalFlowVisualOdometry()
    {
        poses_file_.close();
    }

     /**
     * Main member function: computes the current camera pose
     * @param rgb and @param depth images.
     * @param time_stamp, the time that the pose was computed.
     */ 
    bool computeCameraPose(const cv::Mat& rgb, const cv::Mat& depth, const std::string& time_stamp = "");

    /**
     * Utility function: creates and returns a Keyframe
     * with the given id.
     * computeCameraPose must be called first, so
     * the last supplied data to computeCameraPose is
     * used to create a keyframe.
     * @param kf_id: id of the keyframe
     */
    Keyframe createKeyframe(const size_t &kf_id);
};

#endif /* INCLUDE_OPTICAL_FLOW_VISUAL_ODOMETRY_H_ */