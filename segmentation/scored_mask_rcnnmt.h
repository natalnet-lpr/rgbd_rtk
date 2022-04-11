#pragma once
#include <vector>

#include "motion_treater.h"
#include "mask_rcnn_dnn_mt.h"
#include "scored_fbmt.h"

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

  void initializeMaskRcnnDnnMT(cv::dnn::Backend backend_id,
                               cv::dnn::Target target_id,
                               std::vector<MaskRcnnClass> valid_classes);

  void initializeScoredFBMT(MotionEstimatorRANSAC *motion_estimator, const Intrinsics &intrinsics, float dist_threshold, int score_threshold);

  void segment(const cv::Mat &img_in, cv::Mat &img_out,
               float threshold);

  std::vector<int> estimateStaticPointsIndexes(const std::vector<cv::Point2f> &curr_pts);

private:
  std::map<MT_Types, MotionTreater *> motion_treater_map_;
};