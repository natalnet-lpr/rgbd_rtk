/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2019, Natalnet Laboratory for Perceptual Robotics
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
 *  Felipe Ferreira Barbosa
 *  Vanessa Dantas de Souto Costa
 *  Bruno Silva
 */

#include <stereo_optical_flow_visual_odometry.h>
#include <geometry.h>

using namespace std;
using namespace cv;

StereoOpticalFlowVisualOdometry::StereoOpticalFlowVisualOdometry(const Intrinsics& intr):
frame_idx_(0), cloud_generator_(intr, 1), motion_estimator_(intr, 0.3, 0)
{
	pose_ = Eigen::Affine3f::Identity();

	prev_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	curr_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
}

void StereoOpticalFlowVisualOdometry::computeCameraPose(const cv::Mat& left, const cv::Mat& right)
{
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();

	//Get dense point cloud from the stereo point cloud
	cloud_generator_.generatePointCloud(left, right);
	*curr_dense_cloud_ = *cloud_generator_.cloud_;

	MLOG_DEBUG(EventLogger::M_STEREO, "Generated cloud has %lu points\n",
		       curr_dense_cloud_->size());

	//Track keypoints using KLT optical flow
	tracker_.track(left);

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