/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Natalnet Laboratory for Perceptual Robotics
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

#ifndef INCLUDE_MOTION_ESTIMATOR_RANSAC_H_
#define INCLUDE_MOTION_ESTIMATOR_RANSAC_H_

#include <vector>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <common_types.h>

#include <pcl/correspondence.h>

class MotionEstimatorRANSAC
{

protected:

	//Set of intrinsic parameters
	Intrinsics intr_;

	//Sets the data of the source and target point clouds with the 3D coords. of each 2D point.
	//The function assumes that tgt_points and src_points have the same size and also that
	//the 3D points related to tgt_points[i] and src_points[i] are added to the point clouds
	//only if neither of them are invalid.
	void setDataFromCorrespondences(const std::vector<cv::Point2f> tgt_points, const pcl::PointCloud<PointT>::Ptr tgt_dense_cloud,
		                            const std::vector<cv::Point2f> src_points, const pcl::PointCloud<PointT>::Ptr src_dense_cloud);

public:

	//DEBUG
	pcl::Correspondences src_to_tgt; 

	//Target point cloud (formely the previous point cloyd)
	pcl::PointCloud<PointT>::Ptr tgt_cloud_;

	//Source point cloud (formely the current point cloud)
	pcl::PointCloud<PointT>::Ptr src_cloud_;

	//Number of inlier correspondences after motion estimation
	long unsigned num_inliers_;

	//Vector telling if each correspondence is an inlier or not
	std::vector<unsigned char> is_inlier_;

	//Default constructor
	MotionEstimatorRANSAC();

	//Constructor with the matrix of intrinsic parameters
	MotionEstimatorRANSAC(const Intrinsics intr);

	/* Main member function: estimates the motion between two point clouds as the registration transformation
	 * between two sparse clouds of visual features. The sparse clouds are given as two vectors of 2D points,
	 * from which the corresponding 3D points are extracted.
	 */
	Eigen::Matrix4f estimate(const std::vector<cv::Point2f> tgt_points, const pcl::PointCloud<PointT>::Ptr tgt_dense_cloud,
		                     const std::vector<cv::Point2f> src_points, const pcl::PointCloud<PointT>::Ptr src_dense_cloud);

};

#endif /* INCLUDE_MOTION_ESTIMATOR_RANSAC_H_ */