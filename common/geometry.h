/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2021, Natalnet Laboratory for Perceptual Robotics
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
 *  Rodrigo Sarmento Xavier
 */

#ifndef INCLUDE_GEOMETRY_H_
#define INCLUDE_GEOMETRY_H_

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>

#include <common_types.h>

/**
 * Utility function: returns true if the 3D point does not contain NaN values
 * @param p PointT
 * @return boolean
 */
bool is_valid(const PointT &p);
/**
 * Utility function: returns the 3D point in the dense cloud corresponding to the 2D feature
 * We assume that the point cloud is registered with the rgb image
 * @param point point(x,y), point cloud @param dense_cloud
 * @return 3d point
 */
PointT get3Dfrom2D(const cv::Point2f &point, const pcl::PointCloud<PointT>::Ptr &dense_cloud);
/**
 * Computes a RGB-D point cloud from the camera intrinsic parameters and the RGB and depth data.
 * @param rgb @param depth image @param intr camera intrinsics
 * @return Point cloud as pcl::PointCloud<PointT>
 */
pcl::PointCloud<PointT> getPointCloud(const cv::Mat &rgb, const cv::Mat &depth,
                                      const Intrinsics &intr);
/**
 * Update the pose given by a distance offset
 * @param pose as Affine3f @param offset_distance as float
 * @return new pose
 */
Eigen::Affine3f newPoseOffset(const Eigen::Affine3f &pose, const float &offset_distance);

/**
 * Returns the distance between the centers of
 * two reference frames.
 * @param first_rf first ref. frame
 * @param second_rf second ref. frame
 * @return distance (same unit as the ref. frames)
 */
double distanceBetween(const Eigen::Affine3f &first_rf, const Eigen::Affine3f& second_rf);

/**
 * Returns the angle between the z-axis of
 * two reference frames.
 * @param first_rf first ref. frame
 * @param second_rf second ref. frame
 * @return angle (in degrees)
 */
bool zAxisAngleBetween(const Eigen::Affine3f& first_rf, const Eigen::Affine3f& second_rf);

/**
 * Converts the given Eigen::Affine3f to Eigen::Isometry3d.
 * @param m: the Eigen::Affine3f that should be transformed
 * @return the resulting Eigen::Isometry3d.
 */
Eigen::Isometry3d affineToIsometry(const Eigen::Affine3f &m);

/**
 * Converts the given Eigen::Isometry3d to Eigen::Affine3f.
 * @param m: the Eigen::Isometry3d that should be transformed
 * @return the resulting Eigen::Affine3f.
 */
Eigen::Affine3f isometryToAffine(const Eigen::Isometry3d &m);

/**
 * Computes the relative transformation between two poses A and B.
 * This is given as the transformation T that transforms ref. frame A into B.
 * T is given relative to ref. frame A.
 * Therefore, T = A^(-1).B, since the post multiplication A.T = A.A^(-1).B = B
 * @param from_pose the pose from which the transform must be computed
 * @param to_pose the pose which the original pose must be transformed into
 * @return The relative transformation from from_pose to to_pose.
 */
Eigen::Affine3f relativeTransform(const Eigen::Affine3f &from_pose, const Eigen::Affine3f &to_pose);

/**
 * Computes the relative transformation between two poses A and B.
 * This is given as the transformation T that transforms ref. frame A into B.
 * T is given relative to ref. frame A.
 * Therefore, T = A^(-1).B, since the post multiplication A.T = A.A^(-1).B = B
 * @param from_pose the pose from which the transform must be computed
 * @param to_pose the pose which the original pose must be transformed into
 * @return The relative transformation from from_pose to to_pose.
 */
Eigen::Isometry3d relativeTransform(const Eigen::Isometry3d &from_pose, const Eigen::Isometry3d &to_pose);

/**
 * Prints the given transformation.
 * @param t: transformation to be printed (Eigen::Affine3f)
 */
void printTransform(const Eigen::Affine3f& t);

/**
 * Prints the given transformation.
 * @param t: transformation to be printed (Eigen::Isometry3d)
 */
void printTransform(const Eigen::Isometry3d& t);

#endif /* INCLUDE_GEOMETRY_H_ */