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

#include <Eigen/Geometry>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

#include <geometry.h>
#include <motion_estimator_ransac.h>

#include <pcl/common/transforms.h>

using namespace std;

//#define DEBUG

void MotionEstimatorRANSAC::setDataFromCorrespondences(const std::vector<cv::Point2f> tgt_points, const pcl::PointCloud<PointT>::Ptr tgt_dense_cloud,
		                                         const std::vector<cv::Point2f> src_points, const pcl::PointCloud<PointT>::Ptr src_dense_cloud)
{
	//Reset sparse src cloud buffer
	src_cloud_->clear();
	src_cloud_->is_dense = true;
	//src_cloud_->width = src_points.size();
	//src_cloud_->height = 1;
	//src_cloud_->points.resize(src_cloud_->width*src_cloud_->height);

	//Reset sparse tgt cloud buffer
	tgt_cloud_->clear();
	tgt_cloud_->is_dense = true;
	//tgt_cloud_->width = tgt_points.size();
	//tgt_cloud_->height = 1;
	//tgt_cloud_->points.resize(tgt_cloud_->width*tgt_cloud_->height);

	#ifdef DEBUG
	printf("Setting 3D correspondences:\n");
	#endif
	//For each correspondence, get both points and their corresponding 3D points in the dense 3D clouds
	//A correspondence is removed if any of the 3D points is invalid
	size_t valid_points = 0;
	for(size_t k = 0; k < src_points.size(); k++)
	{
		PointT tpt = get3Dfrom2D(tgt_points[k], tgt_dense_cloud);
		PointT spt = get3Dfrom2D(src_points[k], src_dense_cloud);

		if(is_valid(tpt) && is_valid(spt))
		{
			//printf("\t\t[%lu]: (%f,%f) <-> (%f,%f) [(%f,%f,%f) <-> (%f,%f,%f)]\n", k, src_points[k].x, src_points[k].y,
			//	                                                                      tgt_points[k].x, tgt_points[k].y,
			//	                                                                      spt.x, spt.y, spt.z, 
			//	                                                                      tpt.x, tpt.y, tpt.z);

			tgt_cloud_->push_back(tpt);
			src_cloud_->push_back(spt);
			//tgt_cloud_->points[valid_points] = tpt;
			//src_cloud_->points[valid_points] = spt;
			valid_points++;
		}
	}
	//tgt_cloud_->points.resize(valid_points);
	//src_cloud_->points.resize(valid_points);
	#ifdef DEBUG
	printf("\tvalid points (with depth)/total points: %lu/%lu\n", valid_points, src_points.size());
	#endif	
}

MotionEstimatorRANSAC::MotionEstimatorRANSAC()
{
	num_inliers_ = 0;
	tgt_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	src_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

	//Initialize intrinsics with Kinect default values
	intr_ = Intrinsics(0);
}

MotionEstimatorRANSAC::MotionEstimatorRANSAC(const Intrinsics intr)
{
	num_inliers_ = 0;
	tgt_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	src_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

	//Initialize intrinsics with the given values
	intr_ = intr;
	printf("Loading intrinsics:\n");
	printf("%f %f %f %f %f\n", intr.fx_, intr.fy_, intr.cx_, intr.cy_, intr.scale_);
	printf("%f %f %f %f %f\n", intr.k1_, intr.k2_, intr.p1_, intr.p2_, intr.k3_);
}

Eigen::Matrix4f MotionEstimatorRANSAC::estimate(const vector<cv::Point2f> tgt_points, const pcl::PointCloud<PointT>::Ptr tgt_dense_cloud,
	                                      const vector<cv::Point2f> src_points, const pcl::PointCloud<PointT>::Ptr src_dense_cloud)
{
	//Fill data buffers with the supplied data
	setDataFromCorrespondences(tgt_points, tgt_dense_cloud, src_points, src_dense_cloud);

	long unsigned N = src_cloud_->points.size();

	#ifdef DEBUG
	printf("\tRANSAC motion estimation: %lu <-> %lu\n", tgt_cloud_->size(), src_cloud_->size());
	#endif

	//Build a RANSAC registration model to estimate the rigid transformation
	pcl::SampleConsensusModelRegistration<PointT>::Ptr sac_model(new pcl::SampleConsensusModelRegistration<PointT>(src_cloud_));
	sac_model->setInputTarget(tgt_cloud_);
	pcl::RandomSampleConsensus<PointT> ransac(sac_model);
	ransac.setDistanceThreshold(0.008); //8mm
	ransac.computeModel();

	//Get the model estimated by RANSAC
	Eigen::VectorXf coeffs, opt_coeffs;
	ransac.getModelCoefficients(coeffs);

	//Get info from RANSAC data and set 0 for outlier and 1 for inlier in each position of the vector
	vector<int> inl;
	ransac.getInliers(inl);
	is_inlier_.resize(N, 0);
	for(size_t i = 0; i < inl.size(); i++)
	{
		int idx = inl[i];
		is_inlier_[idx] = 1;
	}
	num_inliers_ = inl.size();
	float inl_ratio = float(inl.size())/N;
		
	#ifdef DEBUG
	printf("\tinlier ratio: %f\n", inl_ratio);
	#endif

	//Optimize registration transformation using all inlier correspondences
	sac_model->optimizeModelCoefficients(inl, coeffs, opt_coeffs);

	//Set the transf. matrix data from the coeff. param. and return it
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
	trans(0,0) = opt_coeffs[0]; trans(0,1) = opt_coeffs[1]; trans(0,2) = opt_coeffs[2]; trans(0,3) = opt_coeffs[3];
	trans(1,0) = opt_coeffs[4]; trans(1,1) = opt_coeffs[5]; trans(1,2) = opt_coeffs[6]; trans(1,3) = opt_coeffs[7];
	trans(2,0) = opt_coeffs[8]; trans(2,1) = opt_coeffs[9]; trans(2,2) = opt_coeffs[10]; trans(2,3) = opt_coeffs[11];
	trans(3,0) = opt_coeffs[12]; trans(3,1) = opt_coeffs[13]; trans(3,2) = opt_coeffs[14]; trans(3,3) = opt_coeffs[15];
/*
	//DEBUG: estimate repr. error
	pcl::PointCloud<PointT>::Ptr tmp_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	pcl::transformPointCloud(*src_cloud_, *tmp_cloud, trans);
	for(size_t i = 0; i < N; i++)
	{
		float X = tmp_cloud->points[i].x;
		float Y = tmp_cloud->points[i].y;
		float Z = tmp_cloud->points[i].z;
		float x = intr_.fx_*X/Z + intr_.cx_;
		float y = intr_.fy_*Y/Z + intr_.cy_;
		float xc = tgt_points[i].x;
		float yc = tgt_points[i].y;
		float err = sqrt((x - xc)*(x - xc) + (y - yc)*(y - yc));
		printf("Point %lu: inlier(%i), pos(%f, %f), repr(%f, %f), error: %f\n", i, is_inlier_[i], xc, yc, x, y, err);
	}
*/
	return trans;
}