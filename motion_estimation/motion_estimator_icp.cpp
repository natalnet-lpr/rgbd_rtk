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
#include <event_logger.h>
#include <motion_estimator_ransac.h>

using namespace std;

void MotionEstimatorICP::downSampleCloud(const pcl::PointCloud<PointT>::Ptr& dense_cloud,
		                                 pcl::PointCloud<PointT>& res_cloud)
{
	sampler_.setInputCloud(dense_cloud);
	sampler_.setRadiusSearch(getRadius());
	sampler_.filter(res_cloud);
}

MotionEstimatorICP::MotionEstimatorICP()
{
	//Set ICP parameters
	setMaxCorrespondenceDistance(0.05);
	setMaximumIterations(50);
	setTransformationEpsilon(1e-9);
	setEuclideanFitnessEpsilon(0.001);
	setRadius(0.05);
	icp_.setMaxCorrespondenceDistance(getMaxCorrespondenceDistance());
	icp_.setMaximumIterations(getMaximumIterations());
	icp_.setTransformationEpsilon(getEuclideanFitnessEpsilon());
	icp_.setEuclideanFitnessEpsilon(getEuclideanFitnessEpsilon());
}

MotionEstimatorICP::MotionEstimatorICP(const double& max_correspondence_distance, const int& maximum_iterations, const double& transformation_epsilon,
									const double& euclidean_fitness_epsilon, const float& radius)
{
	//Set ICP parameters
	setMaxCorrespondenceDistance(max_correspondence_distance);
	setMaximumIterations(maximum_iterations);
	setTransformationEpsilon(transformation_epsilon);
	setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);	
	setRadius(radius);
	icp_.setMaxCorrespondenceDistance(getMaxCorrespondenceDistance());
	icp_.setMaximumIterations(getMaximumIterations());
	icp_.setTransformationEpsilon(getEuclideanFitnessEpsilon());
	icp_.setEuclideanFitnessEpsilon(getEuclideanFitnessEpsilon());
}

Eigen::Affine3f MotionEstimatorICP::estimate(const pcl::PointCloud<PointT>::Ptr& tgt_dense_cloud,
		                                     const pcl::PointCloud<PointT>::Ptr& src_dense_cloud)
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

float MotionEstimatorICP::getRadius()
{
	return this->radius_;
}
void MotionEstimatorICP::setRadius(const float& radius)
{
	try
	{
		if(radius < 0) throw 1;
		else this->radius_ = radius;
	}
	catch(int e)
	{
		logger.print(pcl::console::L_WARN, "[motion_estimator_icp.cpp] WARN: Radius can't be negative\nTrying to use default vallue: 0.05\n");
		this->radius_ = 0.05;
	}
}
double MotionEstimatorICP::getMaxCorrespondenceDistance()
{	
	return this->max_correspondence_distance_;
}
void MotionEstimatorICP::setMaxCorrespondenceDistance(const double& max_correspondence_distance)
{
	try{
		if(max_correspondence_distance < 0) throw 1;
		else this->max_correspondence_distance_ = max_correspondence_distance;
	}
	catch(int e)
	{
		logger.print(pcl::console::L_WARN, "[motion_estimator_icp.cpp] WARN: Max_correspondence_distance can't be negative\nTrying to use default value: 0.1\n");
		this->max_correspondence_distance_ = 0.1;
	}
}
int MotionEstimatorICP::getMaximumIterations()
{
	return this->maximum_iterations_;
}
void MotionEstimatorICP::setMaximumIterations(const int& maximum_iterations)
{
	try{
		if(maximum_iterations < 0) throw 1;
		else this->maximum_iterations_ = maximum_iterations;
	}
	catch(int e)
	{
		logger.print(pcl::console::L_WARN, "[motion_estimator_icp.cpp] WARN: Maximum_iterations_ can't be negative\nTrying to use default value: 50\n");
		this->maximum_iterations_ = 50;
	}
}
double MotionEstimatorICP::getTransformationEpsilon()
{
	return this->transformation_epsilon_;
}
void MotionEstimatorICP::setTransformationEpsilon(const double& transformation_epsilon)
{
	try{
		if(transformation_epsilon < 0) throw 1;
		else this->transformation_epsilon_ = transformation_epsilon;
	}
	catch(int e)
	{
		logger.print(pcl::console::L_WARN, "[motion_estimator_icp.cpp] WARN: Transformation_epsilon can't be negative\nTrying to use default value: 1e-9\n");
		this->transformation_epsilon_ = 1e-9;
	}
}
double MotionEstimatorICP::getEuclideanFitnessEpsilon()
{
	return this->euclidean_fitness_epsilon_;
}
void MotionEstimatorICP::setEuclideanFitnessEpsilon(const double& euclidean_fitness_epsilon)
{
	try{
		if(euclidean_fitness_epsilon < 0) throw 1;
		else this->euclidean_fitness_epsilon_ = euclidean_fitness_epsilon;
	}
	catch(int e)
	{
		logger.print(pcl::console::L_WARN, "[motion_estimator_icp.cpp] WARN: Euclidean_fitness_epsilon can't be negative\nTrying to use default value:0.001\n");
		this->euclidean_fitness_epsilon_ = 0.001;
	}
}