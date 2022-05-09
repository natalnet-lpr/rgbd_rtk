/*
*  Software License Agreement (BSD License)
*
*  Copyright (c) 2016-2022, Natalnet Laboratory for Perceptual Robotics
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

#pragma once

#include "./feature_based_mt.h"
#include <motion_estimation/motion_estimator_ransac.h>

class ScoredFBMT : public FeatureBasedMT
{
public:
  ScoredFBMT(MotionEstimatorRANSAC *motion_estimator, const Intrinsics &intrinsics, float dist_threshold, int score_threshold);

  /**
   * @brief Estimate the static points indexes, from a given point vector
   * 
   * @param[in] curr_pts, vector of points
   * @return vector<int> contain the index of all the static points
   */
  std::vector<int> estimateStaticPointsIndexes(const std::vector<cv::Point2f> &curr_pts);

protected:
  /**
   * @brief Convert two world coordinate points to camera coordinate using
   * a perspective projection and calculate it euclidian distance, if the distance
   * is higher than `dist_threshold_`, return true
   * 
   * @param[in] pt_from, Point3d in world coordinate
   * @param[in] pt_to, Point3d in world coordinate
   */
  bool isDynamicPoint(const PointT &pt_from, const PointT &pt_to);

  /**
   * @brief Fill the static points vector with the estimated static point indexes
   * 
   * @param[in] curr_pts, vector of points
   * @return vector<int> contain the index of all the static points
   */
  std::vector<int> getStaticPointIndexes(const std::vector<cv::Point2f> &curr_pts);

  // Intrinsics parameters from camera
  Intrinsics intrisics_;

  // Store the score of each point
  //    Key: Index of the point
  //    Value: Score of the point
  std::map<int, int> point_scores_;

  // Distance threshold used for `isDynamicPoint`
  float dist_threshold_;

  // Score threshold to estimate if a point is dynamic or not
  //    if score < score_threshold_ -> is dynamic
  int score_threshold_;

  float max_score_;
  float min_score_;
};