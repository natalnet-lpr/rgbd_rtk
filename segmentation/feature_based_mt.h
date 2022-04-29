#pragma once

#include "motion_treater.h"

#include <vector>
#include <opencv2/opencv.hpp>
#include <motion_estimation/motion_estimator_ransac.h>

class FeatureBasedMT : public MotionTreater
{
public:
  FeatureBasedMT(MotionEstimatorRANSAC *motion_estimator);

  /**
   * @brief From a mixed features set, calculate the static ones and return's it indexes
   * 
   * @param[in] curr_pts, vector of 2d points
   * @return set of static points indexes
   */
  virtual std::vector<int> estimateStaticPointsIndexes(const std::vector<cv::Point2f> &curr_pts) = 0;

protected:
  MotionEstimatorRANSAC *motion_estimator_;
};