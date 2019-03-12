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


#include <hybrid_optical_flow_visual_odometry.h>
#include <geometry.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



using namespace std;


HybridOpticalFLowVisualOdometry::HybridOpticalFLowVisualOdometry()
{
	frame_idx_ = 0;
	pose_ = Eigen::Affine3f::Identity(); 
	prev_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	curr_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
}

HybridOpticalFLowVisualOdometry::HybridOpticalFLowVisualOdometry(const Intrinsics intr)
{
	frame_idx_ = 0;
	pose_ = Eigen::Affine3f::Identity(); 
	motion_estimator_.intr_ = intr;

	prev_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	curr_dense_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
}

void HybridOpticalFLowVisualOdometry::computeCameraPose(cv::Mat rgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_stereo, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_lidar)
{
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	curr_dense_cloud_->clear(); 
	//Filter the Lidar Cloud with Stereo Cloud
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  	kdtree.setInputCloud (cloud_lidar);
  	for(int i=0;i<cloud_stereo->points.size();i++){
        vector<int> pointIdxNKNSearch(1);
        vector<float> pointNKNSquaredDistance(1);
        if(kdtree.nearestKSearch(cloud_stereo->points[i],1,pointIdxNKNSearch,pointNKNSquaredDistance)>0){
       	 	curr_dense_cloud_->points.push_back(cloud_lidar->points[pointIdxNKNSearch[0]]);
    	}
    	else{
			curr_dense_cloud_->points.push_back(cloud_lidar->points[pointIdxNKNSearch[0]]);
    	}

  	}
  	curr_dense_cloud_->width=cloud_stereo->width;
  	curr_dense_cloud_->height=cloud_stereo->height;

	//Track keypoints using KLT optical flow
	tracker_.track(rgb);

	//Estimate motion between the current and the previous point clouds
	if(frame_idx_ > 0)
	{	
		//
		//
		/*O problema esta aqui */
		trans = motion_estimator_.estimate(tracker_.prev_pts_, prev_dense_cloud_,
			                               tracker_.curr_pts_, curr_dense_cloud_);

		pose_ = pose_*trans;/* */
	}

	//Let the prev. cloud in the next frame be the current cloud
	*prev_dense_cloud_ = *curr_dense_cloud_;

	//Increment the frame index
	frame_idx_++;
}