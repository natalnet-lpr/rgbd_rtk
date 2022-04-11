#include "scored_mask_rcnnmt.h"
#include <exception>

ScoredMaskRcnnMT::ScoredMaskRcnnMT()
{
  motion_treater_map_[MT_Types::MaskRcnnDnnMt] = nullptr;
  motion_treater_map_[MT_Types::ScoredFbmt] = nullptr;
}

ScoredMaskRcnnMT::~ScoredMaskRcnnMT()
{
  std::map<MT_Types, MotionTreater *>::iterator it;

  for (it = motion_treater_map_.begin(); it != motion_treater_map_.end(); it++)
  {
    delete it->second;
  }
}

void ScoredMaskRcnnMT::initializeMaskRcnnDnnMT(cv::dnn::Backend backend_id,
                                               cv::dnn::Target target_id,
                                               std::vector<MaskRcnnClass> valid_classes_)
{
  motion_treater_map_[MT_Types::MaskRcnnDnnMt] = static_cast<MotionTreater *>(new MaskRcnnDnnMT(backend_id, target_id, valid_classes_));
}

void ScoredMaskRcnnMT::initializeScoredFBMT(MotionEstimatorRANSAC *motion_estimator, const Intrinsics &intrinsics, float dist_threshold, int score_threshold)
{
  motion_treater_map_[MT_Types::ScoredFbmt] = static_cast<MotionTreater *>(new ScoredFBMT(motion_estimator, intrinsics, dist_threshold, score_threshold));
}

void ScoredMaskRcnnMT::segment(const cv::Mat &img_in, cv::Mat &img_out,
                               float threshold)
{
  if (!motion_treater_map_[MT_Types::MaskRcnnDnnMt])
  {
    throw std::logic_error("You must to call initializeMaskRcnnDnnMT() first");
  }

  MaskRcnnDnnMT *ptr = static_cast<MaskRcnnDnnMT *>(motion_treater_map_[MT_Types::MaskRcnnDnnMt]);

  ptr->segment(img_in, img_out, threshold);
}

std::vector<int> ScoredMaskRcnnMT::estimateStaticPointsIndexes(const std::vector<cv::Point2f> &curr_pts)
{
  if (!motion_treater_map_[MT_Types::ScoredFbmt])
  {
    throw std::logic_error("You must to call initializeMaskRcnnDnnMT() first");
  }

  ScoredFBMT *ptr = static_cast<ScoredFBMT *>(motion_treater_map_[MT_Types::ScoredFbmt]);

  return ptr->estimateStaticPointsIndexes(curr_pts);
}
