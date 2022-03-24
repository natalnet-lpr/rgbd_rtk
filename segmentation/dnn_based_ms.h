#pragma once
#include "motion_segmenter.h"
#include "object_detected.h"
#include <opencv2/dnn.hpp>
#include <vector>
#include <unordered_map>

class DnnBasedMS : public MotionSegmenter
{
public:
  /**
   * @brief Initialize the object with a DNN model
   * @see for more information, see opencv/dnn/dnn.hpp
   * @param[in] model path to the model file
   * @param[in] config path to the config file
   * @param[in] backend_id an integer that represents the id of the backend API @see cv::dnn::Backend
   * @param[in] target_id an integer that represents the id of the target device @see cv::dnn::Target
   */

  DnnBasedMS(const std::string &model, const std::string &config, const std::string &framework,
             const std::vector<std::string> output_layers_name,
             const std::unordered_map<uint8_t /*class id*/, std::string /*class name*/> valid_classes_map,
             cv::dnn::Backend backend_id = cv::dnn::Backend::DNN_BACKEND_DEFAULT,
             cv::dnn::Target target_id = cv::dnn::Target::DNN_TARGET_CPU);

  virtual ~DnnBasedMS();

  /**
   * @brief This function should perform the instance segmentation task
   * 
   * @param[in] img_in  cv::Mat RGB image
   * @param[out] img_out  cv::Mat image which is a binary mask
   */
  virtual void segment(const cv::Mat &img_in, cv::Mat &img_out, float threshold) = 0;

  /**
   * @brief Perform object detection passing a image
   * 
   * @param[in] img_in cv::Mat RGB image
   * @return ObjectDetected , @see "./object_detected.h"
   */
  virtual ObjectDetected detect(const cv::Mat &img_in, float threshold) = 0;

protected:
  std::string model_;
  std::string config_;
  std::string framework_;
  cv::dnn::Backend backend_id_;
  cv::dnn::Target target_id_;
  cv::dnn::Net net_;
  std::vector<std::string> output_layers_name_;
  std::unordered_map<uint8_t /*class id*/, std::string /*class name*/> valid_classes_map_;
  std::vector<cv::Mat> dnn_output_;
};