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
                               std::vector<MaskRcnnClass> valid_classes);
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