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
 */

#ifndef INCLUDE_OPTICAL_FLOW_VISUAL_ODOMETRY_H_
#define INCLUDE_OPTICAL_FLOW_VISUAL_ODOMETRY_H_

#include <Eigen/Geometry>
#include <map>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>

#include <visual_odometry.h>

class OpticalFlowVisualOdometry: public VisualOdometry
{

public:

    /**
     * Builds a Visual Odometry algorithm with the
     * given parameters
     * @param vo_param: VisualOdometry parameters
     * @param initialPose: Visual Odometry initial pose
     */
    OpticalFlowVisualOdometry(const VisualOdometry::Parameters &vo_param,
                              const Eigen::Affine3f &initialPose =
                                    Eigen::Affine3f::Identity());

     /**
     * Main member function: computes camera pose
     * with KLT feature tracking and RANSAC motion estimation.
     * @param rgb: RGB image
     * @param depth: depth image
     * @param time_stamp: the time that the pose was computed
     */ 
    bool computeCameraPose(const cv::Mat& rgb, const cv::Mat& depth, const std::string& time_stamp = "");
};

#endif /* INCLUDE_OPTICAL_FLOW_VISUAL_ODOMETRY_H_ */