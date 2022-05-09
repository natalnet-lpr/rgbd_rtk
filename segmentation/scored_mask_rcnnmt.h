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

#include <vector>

#include "motion_treater.h"
#include "mask_rcnn_dnn_mt.h"
#include "scored_fbmt.h"

/**
 * @brief This class combines two Motion Treatment approaches
 *    ScoredFBMT + MaskRcnnDnnMT in one single pipeline
 */
class ScoredMaskRcnnMT
{
public:
  enum class MT_Types : uint8_t
  {
    MaskRcnnDnnMt,
    ScoredFbmt
  };

  ScoredMaskRcnnMT();

  ~ScoredMaskRcnnMT();

  /**
   * @brief Instanciate a MaskRcnnDnnMT object
   *
   * @param[in] backend_id, @see OpenCV documentation
   * @param[in] target_id, @see OpenCV documentation
   * @param[in] valid_classes, Vector with the valid classes to be segmented or detected in scene
   */
  void initializeMaskRcnnDnnMT(cv::dnn::Backend backend_id,
                               cv::dnn::Target target_id,
                               const std::vector<MaskRcnnClass> &valid_classes);
  /**
   * @brief Instanciate a ScoredFBMT object
   * 
   */
  void initializeScoredFBMT(MotionEstimatorRANSAC *motion_estimator, const Intrinsics &intrinsics, float dist_threshold, int score_threshold);

  /**
   * @brief  Segment the scene using one or both techniques (ScoredFBMT or MaskRcnnDnnMT)
   * 
   * @param[in] img_in, cv::Mat image
   * @param[out] img_out, cv::Mat binary mask image
   * @param[in] threshold, theshould to set the reliability of the segmented objects
   */
  void segment(const cv::Mat &img_in, cv::Mat &img_out,
               float threshold);

  /**
   * @brief Estimate the static points indexes, from a given point vector
   * 
   * @param[in] curr_pts, vector of points
   * @return vector<int> contain the index of all the static points
   */
  std::vector<int> estimateStaticPointsIndexes(const std::vector<cv::Point2f> &curr_pts);

private:
  // Map that store a implementation the segmentation classes
  std::map<MT_Types, MotionTreater *> motion_treater_map_;
};