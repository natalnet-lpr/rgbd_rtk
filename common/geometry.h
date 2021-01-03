/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or
 *     promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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

#endif /* INCLUDE_GEOMETRY_H_ */