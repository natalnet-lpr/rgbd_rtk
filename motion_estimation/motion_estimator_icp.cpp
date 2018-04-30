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

#include <cstdio>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

#include <motion_estimator_icp.h>

using namespace std;

void MotionEstimatorICP::downSampleCloud(const pcl::PointCloud<PointT>::Ptr dense_cloud,
		                                 pcl::PointCloud<PointT>& res_cloud)
{
	float radius = 0.05;

	sampler_.setInputCloud(dense_cloud);
	sampler_.setRadiusSearch(radius);
	sampler_.filter(res_cloud);
}

MotionEstimatorICP::MotionEstimatorICP()
{
	//Set ICP parameters
	icp_.setMaxCorrespondenceDistance(0.1);
	icp_.setMaximumIterations (50);
	icp_.setTransformationEpsilon(1e-9);
	icp_.setEuclideanFitnessEpsilon(0.001);
}

Eigen::Affine3f MotionEstimatorICP::estimate(const pcl::PointCloud<PointT>::Ptr tgt_dense_cloud,
		                                     const pcl::PointCloud<PointT>::Ptr src_dense_cloud)
{
	Eigen::Affine3f result = Eigen::Affine3f::Identity();
	pcl::PointCloud<PointT> tgt_cloud, src_cloud, aligned_cloud;
	pcl::PointCloud<PointT>::Ptr tgt_cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr src_cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

	//Uniformly sample input point clouds
	downSampleCloud(tgt_dense_cloud, tgt_cloud);
	downSampleCloud(src_dense_cloud, src_cloud);

	//Set ICP input data
	*tgt_cloud_ptr = tgt_cloud;
	*src_cloud_ptr = src_cloud;
	icp_.setInputSource(src_cloud_ptr);
	icp_.setInputTarget(tgt_cloud_ptr);

	//Estimate registration transformation
	icp_.align(aligned_cloud);
	result = icp_.getFinalTransformation();

	return result;
}