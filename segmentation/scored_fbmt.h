#pragma once
#include "./feature_based_mt.h"
#include <unordered_map>
#include <motion_estimation/motion_estimator_ransac.h>

class ScoredFBMT : public FeatureBasedMT
{
public:
  ScoredFBMT(MotionEstimatorRANSAC *motion_estimator, const Intrinsics &intrinsics, float dist_threshold, int score_threshold);
  std::vector<int> estimateStaticPointsIndexes(const std::vector<cv::Point2f> &curr_pts);

protected:
  bool isDynamicPoint(const PointT &pt_from, const PointT &pt_to);

  Intrinsics intrisics_;
  std::map<int, int> point_scores_;
  float dist_threshold_;
  int score_threshold_;
  float max_score_;
  float min_score_;
};