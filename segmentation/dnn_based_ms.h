#pragma once
#include "motion_segmenter.h"
#include "object_detected.h"
#include <opencv2/dnn.hpp>
#include <vector>
#include <unordered_map>

using namespace std;
using namespace cv;
using namespace cv::dnn;

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

  DnnBasedMS(const string &model, const string &config, const string &framework,
             const vector<string> output_layers_name,
             const unordered_map<uint8_t /*class id*/, string /*class name*/> valid_classes_map,
             Backend backend_id = Backend::DNN_BACKEND_DEFAULT,
             Target target_id = Target::DNN_TARGET_CPU);

  virtual ~DnnBasedMS();

  /**
   * @brief This function should perform the instance segmentation task
   * 
   * @param[in] img_in  cv::Mat RGB image
   * @param[out] img_out  cv::Mat image which is a binary mask
   */
  virtual void segment(const Mat &img_in, Mat &img_out, float threshold) = 0;

  /**
   * @brief Perform object detection passing a image
   * 
   * @param[in] img_in cv::Mat RGB image
   * @return ObjectDetected , @see "./object_detected.h"
   */
  virtual ObjectDetected detect(const Mat &img_in, float threshold) = 0;

protected:
  string model_;
  string config_;
  string framework_;
  Backend backend_id_;
  Target target_id_;
  Net net_;
  vector<string> output_layers_name_;
  unordered_map<uint8_t /*class id*/, string /*class name*/> valid_classes_map_;
  vector<Mat> dnn_output_;
};