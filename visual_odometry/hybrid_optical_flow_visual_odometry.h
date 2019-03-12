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
#ifndef INCLUDE_HYBRID_OPTICAL_FLOW_VISUAL_ODOMETRY_H_
#define INCLUDE_HYBRID_OPTICAL_FLOW_VISUAL_ODOMETRY_H_


#include <iostream>
#include <Eigen/Geometry>
#include <pcl/filters/frustum_culling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>


#include <common_types.h>
#include <klt_tracker.h>
#include <motion_estimator_ransac.h>

using namespace std;
//using namespace cv;


class HybridOpticalFLowVisualOdometry
{
private:
	//Current frame index
	unsigned long int frame_idx_;

public:
	//Previous dense point cloud
	pcl::PointCloud<PointT>::Ptr prev_dense_cloud_;

	//Current dense point cloud
	pcl::PointCloud<PointT>::Ptr curr_dense_cloud_;
	
	//Feature tracker
	KLTTracker tracker_;

	//Motion estimator
	MotionEstimatorRANSAC motion_estimator_;

	//Current camera pose
	Eigen::Affine3f pose_;

	//Default constructor
	HybridOpticalFLowVisualOdometry();

	//Constructor with the matrix of intrinsic parameters
	HybridOpticalFLowVisualOdometry(const Intrinsics intr);

	//Main member function: computes the current camera pose
	void computeCameraPose(cv::Mat rgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_stereo, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_lidar);

};

#endif /* INCLUDE_HYBRID_OPTICAL_FLOW_VISUAL_ODOMETRY_H_ */
