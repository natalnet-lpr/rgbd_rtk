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

#include <algorithm>

#include <geometry.h>
#include <optical_flow_visual_odometry.h>

using namespace std;
using namespace cv;

void OpticalFlowVisualOdometry::writePoseToFile(const std::string& time_stamp)
{
    Eigen::Matrix3f Rot;
	Rot(0,0) = pose_(0,0); Rot(0,1) = pose_(0,1); Rot(0,2) = pose_(0,2);
	Rot(1,0) = pose_(1,0); Rot(1,1) = pose_(1,1); Rot(1,2) = pose_(1,2);
	Rot(2,0) = pose_(2,0); Rot(2,1) = pose_(2,1); Rot(2,2) = pose_(2,2);
	
    Eigen::Quaternionf q(Rot);
	
    poses_file_ << time_stamp <<  " "  << pose_(0,3) << " "
								  	   << pose_(1,3) << " "
								  	   << pose_(2,3) << " "
								  	   << q.x() << " "
								  	   << q.y() << " "
								  	   << q.z() << " "
								  	   << q.w() << "\n";
}

OpticalFlowVisualOdometry::OpticalFlowVisualOdometry(const Intrinsics& intr,
                                                     const FeatureTracker::Parameters& tracking_param,
                                                     const float& ransac_thr,
                                                     const Eigen::Affine3f& initialPose):
frame_idx_(0), motion_estimator_(intr, ransac_thr, 0)
{
    pose_ = initialPose;

    prev_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    curr_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    MLOG_DEBUG(EventLogger::M_VISUAL_ODOMETRY,
               "@OpticalFlowVisualOdometry: "
               "building with tracker %s\n",
               tracking_param.type_.c_str());

    if(FeatureTracker::strToType(tracking_param.type_) == FeatureTracker::TRACKER_KLT)
    {
        tracker_ptr_ = cv::Ptr<FeatureTracker>(new KLTTracker(tracking_param));
    }
    else if(FeatureTracker::strToType(tracking_param.type_) == FeatureTracker::TRACKER_KLTTW)
    {
        tracker_ptr_ = cv::Ptr<FeatureTracker>(new KLTTWTracker(tracking_param));
    }
    else
    {
        MLOG_ERROR(EventLogger::M_VISUAL_ODOMETRY, "@OpticalFlowVisualOdometry: "
                                                   "unknown feature tracker "
                                                   "informed: %s\n",
                                                   tracking_param.type_.c_str());
        exit(-1);
    }

    poses_file_.open("optical_flow_visual_odometry_poses.txt");
    if (!poses_file_.is_open())
    {
        MLOG_ERROR(EventLogger::M_VISUAL_ODOMETRY, "@OpticalFlowVisualOdometry: "
                                                   "there is a problem opening "
                                                   "the file with the computed poses.\n");
        exit(-1);
    }
}

bool OpticalFlowVisualOdometry::computeCameraPose(const cv::Mat& rgb, const cv::Mat& depth, const std::string& time_stamp)
{
    bool is_kf;

    MLOG_INFO(EventLogger::M_VISUAL_ODOMETRY, "Processing frame %lu\n",
              frame_idx_);

    Eigen::Affine3f trans = Eigen::Affine3f::Identity();

    // Copy RGB-D data to internal buffers
    rgb.copyTo(rgb_);
    depth.copyTo(depth_);

    // Get dense point cloud from RGB-D data
    *curr_dense_cloud_ = getPointCloud(rgb, depth, motion_estimator_.intr_);

    // Track keypoints using KLT optical flow
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
    writePoseToFile(time_stamp);

    // Let the prev. cloud in the next frame be the current cloud
    *prev_dense_cloud_ = *curr_dense_cloud_;

    // Increment the frame index
    frame_idx_++;

    return is_kf;
}

Keyframe OpticalFlowVisualOdometry::createKeyframe(const size_t &kf_id)
{
    Keyframe kf;
    kf.idx_ = kf_id;
    kf.pose_ = pose_;
    rgb_.copyTo(kf.img_);
    *kf.local_cloud_ = *curr_dense_cloud_;
    kf.keypoints_.resize(tracker_ptr_->curr_pts_.size());
    copy(tracker_ptr_->curr_pts_.begin(), tracker_ptr_->curr_pts_.end(), kf.keypoints_.begin());

    return kf;
}
