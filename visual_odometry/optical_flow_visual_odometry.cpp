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
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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

#include <geometry.h>
#include <optical_flow_visual_odometry.h>

using namespace std;
using namespace cv;

OpticalFlowVisualOdometry::OpticalFlowVisualOdometry(const VisualOdometry::Parameters &vo_param,
                                                     const Eigen::Affine3f &initialPose):
    VisualOdometry(vo_param, initialPose)
{

}

bool OpticalFlowVisualOdometry::computeCameraPose(const cv::Mat& rgb, const cv::Mat& depth, const std::string& time_stamp)
{
    bool is_kf;

    MLOG_INFO(EventLogger::M_VISUAL_ODOMETRY,
              "@VisualOdometry::computeCameraPose: "
              "processing frame %lu\n", frame_idx_);

    Eigen::Affine3f trans = Eigen::Affine3f::Identity();

    // Copy RGB-D data to internal buffers
    rgb.copyTo(rgb_);
    depth.copyTo(depth_);

    // Get dense point cloud from RGB-D data
    *curr_dense_cloud_ = getPointCloud(rgb, depth, motion_estimator_.intr_);

    // Track keypoints
    is_kf = tracker_ptr_->track(rgb);

    // Estimate motion between the current and the previous point clouds
    if (frame_idx_ > 0)
    {
        trans = motion_estimator_.estimate(tracker_ptr_->prev_pts_,
                                           prev_dense_cloud_,
                                           tracker_ptr_->curr_pts_,
                                           curr_dense_cloud_);
        pose_ = pose_ * trans;
    }

    // Write computed odometry pose to file
    _writePoseToFile(time_stamp);

    // Let the prev. cloud in the next frame be the current cloud
    *prev_dense_cloud_ = *curr_dense_cloud_;

    // Increment the frame index
    frame_idx_++;

    return is_kf;
}