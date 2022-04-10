#pragma once
#include "motion_treater.h"
#include "object_detected.h"
#include <opencv2/dnn.hpp>
#include <vector>
#include <unordered_map>

class DnnObjectClass
{
public:
  DnnObjectClass(std::string name, uint8_t code) : name_(name), code_(code)
  {
  }

  DnnObjectClass(const DnnObjectClass &dnn_obj_class) : name_(dnn_obj_class.name_), code_(dnn_obj_class.code_)
  {
  }

  bool operator==(const DnnObjectClass &other)
  {
    return (other.code_ == code_ && other.name_ == name_);
  }

  bool operator==(uint8_t code)
  {
    return (code_ == code);
  }

  std::string name_;
  uint8_t code_;
};

class DnnBasedMT : public MotionTreater
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

  DnnBasedMT(const std::string &model, const std::string &config, const std::string &framework,
             const std::vector<std::string> output_layers_name,
             const std::vector<DnnObjectClass> valid_classes,
             cv::dnn::Backend backend_id = cv::dnn::Backend::DNN_BACKEND_DEFAULT,
             cv::dnn::Target target_id = cv::dnn::Target::DNN_TARGET_CPU);

  virtual ~DnnBasedMT();

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
  std::vector<DnnObjectClass> valid_classes_;
  std::vector<cv::Mat> dnn_output_;
};