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