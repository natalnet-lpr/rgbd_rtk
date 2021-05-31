/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Natalnet Laboratory for Perceptual Robotics
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

void OpticalFlowVisualOdometry::addKeyFrame(const Mat& rgb)
{
    Keyframe kf;
    kf.idx_ = frame_idx_;
    kf.pose_ = pose_;
    rgb.copyTo(kf.img_);
    *kf.local_cloud_ = *curr_dense_cloud_;
    kf.keypoints_.resize(tracker_.curr_pts_.size());
    copy(tracker_.curr_pts_.begin(), tracker_.curr_pts_.end(), kf.keypoints_.begin());

    MLOG_DEBUG(EventLogger::M_VISUAL_ODOMETRY, "@OpticalFlowVisualOdometry::addKeyFrame: \
                                                adding %lu as keyframe\n", frame_idx_);
    
    keyframes_.insert(pair<size_t, Keyframe>(kf.idx_, kf));
}

OpticalFlowVisualOdometry::OpticalFlowVisualOdometry(const Eigen::Affine3f& initialPose)
{
    frame_idx_ = 0;
    pose_ = initialPose;

    prev_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    curr_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    poses_file_.open("optical_flow_visual_odometry_poses.txt");
    if (!poses_file_.is_open())
    {
        MLOG_ERROR(EventLogger::M_VISUAL_ODOMETRY, "@OpticalFlowVisualOdometry: \
                                                    there is a problem opening \
                                                    the file with the computed poses.\n");
        exit(-1);
    }
}

OpticalFlowVisualOdometry::OpticalFlowVisualOdometry(const Intrinsics& intr, const Eigen::Affine3f& initialPose)
{
    frame_idx_ = 0;
    pose_ = initialPose;
    motion_estimator_.intr_ = intr;

    prev_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    curr_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    poses_file_.open("optical_flow_visual_odometry_poses.txt");
    if (!poses_file_.is_open())
    {
        MLOG_ERROR(EventLogger::M_VISUAL_ODOMETRY, "@OpticalFlowVisualOdometry: \
                                                    there is a problem opening \
                                                    the file with the computed poses.\n");
        exit(-1);
    }
}

bool OpticalFlowVisualOdometry::computeCameraPose(const cv::Mat& rgb, const cv::Mat& depth, const std::string& time_stamp)
{
    bool is_kf;

    Eigen::Affine3f trans = Eigen::Affine3f::Identity();

    // Get dense point cloud from RGB-D data
    *curr_dense_cloud_ = getPointCloud(rgb, depth, motion_estimator_.intr_);

    // Track keypoints using KLT optical flow
    is_kf = tracker_.track(rgb);

    // If a new keyframe is found, add it to the internal buffer
    if (is_kf) addKeyFrame(rgb);

    // Estimate motion between the current and the previous point clouds
    if (frame_idx_ > 0)
    {
        trans =
            motion_estimator_.estimate(tracker_.prev_pts_, prev_dense_cloud_, tracker_.curr_pts_, curr_dense_cloud_);
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

Keyframe OpticalFlowVisualOdometry::getLastKeyframe()
{
    map<size_t, Keyframe>::iterator it;

    it = keyframes_.end();


    MLOG_DEBUG(EventLogger::M_VISUAL_ODOMETRY, "@OpticalFlowVisualOdometry::getLastKeyframe: \
                                                last keyframe has id %lu\n", it->first);
    
    return prev(it)->second; // prev from std
}
