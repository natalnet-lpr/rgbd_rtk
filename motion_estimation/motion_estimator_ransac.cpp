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

#include <Eigen/Geometry>
#include <cstdio>
#include <cstdlib>
#include <pcl/common/transforms.h>
#include <pcl/common/transforms.h>
#include <algorithm>

#include <event_logger.h>
#include <geometry.h>
#include <motion_estimator_ransac.h>

using namespace std;

void MotionEstimatorRANSAC::setDataFromCorrespondences(
    const std::vector<cv::Point2f> &tgt_points, const pcl::PointCloud<PointT>::Ptr &tgt_dense_cloud,
    const std::vector<cv::Point2f> &src_points, const pcl::PointCloud<PointT>::Ptr &src_dense_cloud)
{
    // Reset sparse src cloud buffer
    src_cloud_->clear();
    src_cloud_->is_dense = true;
    // src_cloud_->width = src_points.size();
    // src_cloud_->height = 1;
    // src_cloud_->points.resize(src_cloud_->width*src_cloud_->height);

    // Reset sparse tgt cloud buffer
    tgt_cloud_->clear();
    tgt_cloud_->is_dense = true;
    // tgt_cloud_->width = tgt_points.size();
    // tgt_cloud_->height = 1;
    // tgt_cloud_->points.resize(tgt_cloud_->width*tgt_cloud_->height);

    // For each correspondence, get both points and their corresponding 3D points in the dense 3D
    // clouds
    // A correspondence is removed if any of the 3D points is invalid
    size_t valid_points = 0;

    mappers_2d_3d_.push_back(vector<int>());

    for (size_t k = 0; k < src_points.size(); k++)
    {
        PointT tpt = get3Dfrom2D(tgt_points[k], tgt_dense_cloud);
        PointT spt = get3Dfrom2D(src_points[k], src_dense_cloud);

        if (is_valid(tpt) && is_valid(spt))
        {
            tgt_cloud_->push_back(tpt);
            src_cloud_->push_back(spt);

            mappers_2d_3d_.back().push_back(k);

            valid_points++;
        }
    }

    // tgt_cloud_->points.resize(valid_points);
    // src_cloud_->points.resize(valid_points);
    MLOG_DEBUG(EventLogger::M_MOTION_ESTIMATION, "@MotionEstimatorRANSAC::setDataFromCorrespondences: \
                                                  valid points (with depth)/total points: %lu/%lu\n",
               valid_points, src_points.size());
}

void MotionEstimatorRANSAC::createAndComputeRansacModel(pcl::SampleConsensusModelRegistration<PointT>::Ptr &sac_model,
                                                        pcl::RandomSampleConsensus<PointT>::Ptr &ransac)
{
    sac_model.reset(new pcl::SampleConsensusModelRegistration<PointT>(src_cloud_));

    sac_model->setInputTarget(tgt_cloud_);

    ransac.reset(new pcl::RandomSampleConsensus<PointT>(sac_model));

    ransac->setDistanceThreshold(distance_threshold_);

    ransac->computeModel();
}

Eigen::Matrix4f MotionEstimatorRANSAC::createTransMatrixFromOptimizedCoefficients(const Eigen::VectorXf &opt_coeffs)
{
    // Set the transf. matrix data from the coeff. param. and return it
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    uint8_t numb_of_elements = 4 * 4; // -> a Matrix of 4 dimmentions have 4x4 elements

    uint8_t cols = 0;
    uint8_t rows = 0;

    for (uint8_t i = 0; i < numb_of_elements; i++)
    {
        rows = i / 4;
        cols = i % 4;

        trans(rows, cols) = opt_coeffs[i];
    }

    return trans;
}

MotionEstimatorRANSAC::MotionEstimatorRANSAC()
{
    distance_threshold_ = 0.008;
    inliers_ratio_ = 0.8;
    num_inliers_ = 0;

    tgt_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    src_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    // Initialize intrinsics with Kinect default values
    intr_ = Intrinsics(0);
}

MotionEstimatorRANSAC::MotionEstimatorRANSAC(const Intrinsics &intr,
                                             const float &distance_threshold,
                                             const float &inliers_ratio)
{
    distance_threshold_ = distance_threshold;
    inliers_ratio_ = inliers_ratio;
    num_inliers_ = 0;

    tgt_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    src_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    // Initialize intrinsics with the given values
    intr_ = intr;

    MLOG_INFO(EventLogger::M_MOTION_ESTIMATION, "@MotionEstimatorRANSAC: loading intrinsics...\n");
    MLOG_INFO(EventLogger::M_MOTION_ESTIMATION, "@MotionEstimatorRANSAC: %f %f %f %f %f\n",
              intr.fx_, intr.fy_, intr.cx_, intr.cy_, intr.scale_);
}

Eigen::Matrix4f MotionEstimatorRANSAC::estimate(const vector<cv::Point2f> &tgt_points,
                                                const pcl::PointCloud<PointT>::Ptr &tgt_dense_cloud,
                                                const vector<cv::Point2f> &src_points,
                                                const pcl::PointCloud<PointT>::Ptr &src_dense_cloud)
{
    // Fill data buffers with the supplied data
    setDataFromCorrespondences(tgt_points, tgt_dense_cloud, src_points, src_dense_cloud);

    long unsigned N = src_cloud_->points.size();

    MLOG_DEBUG(EventLogger::M_MOTION_ESTIMATION, "@MotionEstimatorRANSAC::estimate: RANSAC motion estimation: %lu <-> %lu\n",
               tgt_cloud_->size(), src_cloud_->size());

    pcl::RandomSampleConsensus<PointT>::Ptr ransac;
    pcl::SampleConsensusModelRegistration<PointT>::Ptr sac_model;

    createAndComputeRansacModel(sac_model, ransac);

    // Get the model estimated by RANSAC
    Eigen::VectorXf coeffs, opt_coeffs;

    ransac->getModelCoefficients(coeffs);

    // Get info from RANSAC data and set 0 for outlier and 1 for inlier in each position of the
    // vector
    vector<int> inl;
    ransac->getInliers(inl);
    is_inlier_.resize(N, 0);

    for (size_t i = 0; i < inl.size(); i++)
    {
        int idx = inl[i];
        is_inlier_[idx] = 1;
    }

    num_inliers_ = inl.size();

    if (inl.size() < min_inliers_number_)
    {
        // if the estimation is invalid based on the number of inliers
        // we should remove the last added mapper from the mappers vector
        mappers_2d_3d_.erase(mappers_2d_3d_.end() - 1);
        const std::string msg = "input size is less than the minimum inliers number";
        throw std::length_error(msg.c_str());
    }

    float inl_ratio = float(inl.size()) / N;

    MLOG_DEBUG(EventLogger::M_MOTION_ESTIMATION, "@MotionEstimatorRANSAC::estimate: \
                                                  inlier ratio is %f\n",
               inl_ratio);

    // Optimize registration transformation using all inlier correspondences
    sac_model->optimizeModelCoefficients(inl, coeffs, opt_coeffs);

    // probabily we should to change this to do not make copies
    sparsePointCloudPairs_.push_back(pair<pcl::PointCloud<PointT>, pcl::PointCloud<PointT>>(*tgt_cloud_, *src_cloud_));

    Eigen::Matrix4f trans = createTransMatrixFromOptimizedCoefficients(opt_coeffs);

    relative_poses_.push_back(trans);

    return trans;
}