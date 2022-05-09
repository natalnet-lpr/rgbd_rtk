/*
*  Software License Agreement (BSD License)
*
*  Copyright (c) 2016-2022, Natalnet Laboratory for Perceptual Robotics
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
*  Luiz Correia
*/

#include "scored_mask_rcnnmt.h"
#include <exception>

using namespace std;
using namespace cv;
using namespace cv::dnn;

ScoredMaskRcnnMT::ScoredMaskRcnnMT()
{
  motion_treater_map_[MT_Types::MaskRcnnDnnMt] = nullptr;
  motion_treater_map_[MT_Types::ScoredFbmt] = nullptr;
}

ScoredMaskRcnnMT::~ScoredMaskRcnnMT()
{
  map<MT_Types, MotionTreater *>::iterator it;

  for (it = motion_treater_map_.begin(); it != motion_treater_map_.end(); it++)
  {
    delete it->second;
  }
}

string convertValidClassesToString(const vector<MaskRcnnClass> &valid_classes_)
{
  string valid_classes_string;
  const MaskRcnnClass &last_valid_class = *valid_classes_.cbegin();
  for (const auto &valid_class : valid_classes_)
  {
    valid_classes_string = parseMaskRcnnClass(valid_class).class_name;
    if (valid_class != last_valid_class)
    {
      valid_classes_string = valid_classes_string + " ";
    }
  }
  return valid_classes_string;
}

void ScoredMaskRcnnMT::initializeMaskRcnnDnnMT(Backend backend_id,
                                               Target target_id,
                                               const vector<MaskRcnnClass> &valid_classes_)
{
  if (!motion_treater_map_[MT_Types::MaskRcnnDnnMt])
  {
    const string valid_classes_string = convertValidClassesToString(valid_classes_);
    MLOG_DEBUG(EventLogger::M_SEGMENTATION, format("@ScoredMaskRcnnMT::initializeMaskRcnnDnnMT: initialized the MaskRcnnDnnMt with the classes {}", valid_classes_string).c_str());
    motion_treater_map_[MT_Types::MaskRcnnDnnMt] = static_cast<MotionTreater *>(new MaskRcnnDnnMT(backend_id, target_id, valid_classes_));
  }
}

void ScoredMaskRcnnMT::initializeScoredFBMT(MotionEstimatorRANSAC *motion_estimator, const Intrinsics &intrinsics, float dist_threshold, int score_threshold)
{
  if (!motion_treater_map_[MT_Types::ScoredFbmt])
  {
    MLOG_DEBUG(EventLogger::M_SEGMENTATION, "@ScoredMaskRcnnMT::initializeScoredFBMT: initialized the ScoredFBMT with the classes");
    motion_treater_map_[MT_Types::ScoredFbmt] = static_cast<MotionTreater *>(new ScoredFBMT(motion_estimator, intrinsics, dist_threshold, score_threshold));
  }
}

void ScoredMaskRcnnMT::segment(const Mat &img_in, Mat &img_out,
                               float threshold)
{
  if (!motion_treater_map_[MT_Types::MaskRcnnDnnMt])
  {
    MLOG_ERROR(EventLogger::M_SEGMENTATION, "@ScoredMaskRcnnMT::segment You must to call initializeMaskRcnnDnnMT() first ");
    throw logic_error("You must to call initializeMaskRcnnDnnMT() first");
  }

  MaskRcnnDnnMT *ptr = static_cast<MaskRcnnDnnMT *>(motion_treater_map_[MT_Types::MaskRcnnDnnMt]);
  ptr->segment(img_in, img_out, threshold);
}

vector<int> ScoredMaskRcnnMT::estimateStaticPointsIndexes(const vector<Point2f> &curr_pts)
{
  if (!motion_treater_map_[MT_Types::ScoredFbmt])
  {
    MLOG_ERROR(EventLogger::M_SEGMENTATION, "@ScoredMaskRcnnMT::estimateStaticPointsIndexes You must to call initializeScoredFBMT() first ");
    throw logic_error("You must to call initializeScoredFBMT() first");
  }

  ScoredFBMT *ptr = static_cast<ScoredFBMT *>(motion_treater_map_[MT_Types::ScoredFbmt]);
  return ptr->estimateStaticPointsIndexes(curr_pts);
}
