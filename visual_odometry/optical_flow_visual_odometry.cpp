/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2017, Natalnet Laboratory for Perceptual Robotics
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
 */

#include <optical_flow_visual_odometry.h>
#include <geometry.h>

using namespace std;
using namespace cv;

OpticalFlowVisualOdometry::OpticalFlowVisualOdometry()
{
	frame_idx_ = 0;
	pose_ = Eigen::Affine3f::Identity();

	prev_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	curr_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
}

OpticalFlowVisualOdometry::OpticalFlowVisualOdometry(const Intrinsics intr)
{
	frame_idx_ = 0;
	pose_ = Eigen::Affine3f::Identity();
	motion_estimator_.intr_ = intr;

	prev_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	curr_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
}

void OpticalFlowVisualOdometry::computeCameraPose(cv::Mat rgb, cv::Mat depth)
{
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();

	//Get dense point cloud from RGB-D data
	*curr_dense_cloud_ = getPointCloud(rgb, depth, motion_estimator_.intr_);

	//Track keypoints using KLT optical flow
	tracker_.track(rgb);

	//Estimate motion between the current and the previous point clouds
	if(frame_idx_ > 0)
	{
		trans = motion_estimator_.estimate(tracker_.prev_pts_, prev_dense_cloud_,
			                               tracker_.curr_pts_, curr_dense_cloud_);
		pose_ = pose_*trans;
	}

	//Let the prev. cloud in the next frame be the current cloud
	*prev_dense_cloud_ = *curr_dense_cloud_;

	//Increment the frame index
	frame_idx_++;
}