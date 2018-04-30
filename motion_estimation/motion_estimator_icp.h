/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2018, Natalnet Laboratory for Perceptual Robotics
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

#ifndef INCLUDE_MOTION_ESTIMATOR_ICP_H_
#define INCLUDE_MOTION_ESTIMATOR_ICP_H_

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/uniform_sampling.h>

#include <common_types.h>

class MotionEstimatorICP
{

private:

	//(Uniform) point sampler
	pcl::UniformSampling<PointT> sampler_;

	//PCL ICP motion estimator
	pcl::IterativeClosestPoint<PointT, PointT> icp_;

	//Utility function to downsample cloud data
	void downSampleCloud(const pcl::PointCloud<PointT>::Ptr dense_cloud,
		                 pcl::PointCloud<PointT>& res_cloud);

public:

	//Default constructor
	MotionEstimatorICP();

	/* Main member function: estimates the motion between two point clouds as the registration transformation
	 * between two sparse clouds of visual features. The sparse clouds are given as two vectors of 2D points,
	 * from which the corresponding 3D points are extracted.
	 */
	Eigen::Affine3f estimate(const pcl::PointCloud<PointT>::Ptr tgt_dense_cloud,
		                     const pcl::PointCloud<PointT>::Ptr src_dense_cloud);

};

#endif /* INCLUDE_MOTION_ESTIMATOR_ICP_H_ */