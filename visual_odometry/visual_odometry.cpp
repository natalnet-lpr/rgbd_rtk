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

#include <cstdio>
#include <algorithm>

#include <event_logger.h>
#include <visual_odometry.h>
#include <klt_tracker.h>
#include <klttw_tracker.h>

using namespace std;

void VisualOdometry::_writePoseToFile(const std::string &time_stamp)
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

VisualOdometry::VisualOdometry(const VisualOdometry::Parameters &vo_param,
                               const Eigen::Affine3f &initialPose):
	motion_estimator_(vo_param.intr_, vo_param.ransac_thr_, 0),
	frame_idx_(0)
{
    pose_ = initialPose;

    prev_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    curr_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    MLOG_DEBUG(EventLogger::M_VISUAL_ODOMETRY,
               "@VisualOdometry: "
               "building with tracker %s\n",
               vo_param.tracker_param_.type_.c_str());

    if(FeatureTracker::strToType(vo_param.tracker_param_.type_) == FeatureTracker::TRACKER_KLT)
    {
        tracker_ptr_ = cv::Ptr<FeatureTracker>(new KLTTracker(vo_param.tracker_param_));
    }
    else if(FeatureTracker::strToType(vo_param.tracker_param_.type_) == FeatureTracker::TRACKER_KLTTW)
    {
        tracker_ptr_ = cv::Ptr<FeatureTracker>(new KLTTWTracker(vo_param.tracker_param_));
    }
    else
    {
        MLOG_ERROR(EventLogger::M_VISUAL_ODOMETRY, "@VisualOdometry: "
                                                   "unknown feature tracker "
                                                   "informed: %s\n",
                                                   vo_param.tracker_param_.type_.c_str());
        exit(-1);
    }

    poses_file_.open("visual_odometry_poses.txt");
    if (!poses_file_.is_open())
    {
        MLOG_ERROR(EventLogger::M_VISUAL_ODOMETRY, "@VisualOdometry: "
                                                   "there is a problem opening "
                                                   "the file that will store the "
                                                   "computed poses.\n");
        exit(-1);
    }
}

Keyframe VisualOdometry::createKeyframe(const size_t &kf_id)
{
	MLOG_DEBUG(EventLogger::M_VISUAL_ODOMETRY,
		      "@VisualOdometry::createKeyframe: "
			  "creating key frame with index %lu\n", kf_id);

	Keyframe kf;
    kf.idx_ = kf_id;
    kf.pose_ = pose_;
    rgb_.copyTo(kf.img_);
    *kf.local_cloud_ = *curr_dense_cloud_;
    kf.keypoints_.resize(tracker_ptr_->curr_pts_.size());
    copy(tracker_ptr_->curr_pts_.begin(), tracker_ptr_->curr_pts_.end(), kf.keypoints_.begin());

    return kf;
}