#pragma once

#include "dnn_based_mt.h"
#include "mask_rcnn_classes.h"

/**
 * @brief Do the instance segmentation task using the MaskRCNN model
 * @see https://arxiv.org/abs/1703.06870
 * 
 */
class MaskRcnnDnnMT : public DnnBasedMT
{
public:
  /**
   * @param[in] backend_id, @see OpenCV documentation
   * @param[in] target_id, @see OpenCV documentation
   * @param[in] valid_classes, Vector with the valid classes to be segmented or detected in scene
   */
  MaskRcnnDnnMT(cv::dnn::Backend backend_id,
                cv::dnn::Target target_id,
                std::vector<MaskRcnnClass> valid_classes);

  virtual ~MaskRcnnDnnMT();

  /**
   * @brief This function should perform the instance segmentation task
   * 
   * @param[in] img_in  cv::Mat RGB image
   * @param[out] img_out  cv::Mat image which is a binary mask
   */
  virtual void segment(const cv::Mat &img_in, cv::Mat &img_out,
                       float threshold) override;

  /**
   * @brief Perform object detection passing a image
   * 
   * @param[in] img_in cv::Mat RGB image
   * @return ObjectDetected , @see "./object_detected.h"
   */
  virtual ObjectDetected detect(const cv::Mat &img_in, float threshold) override;

protected:
  /**
   * @brief After forward the image to the network, the nertwork'll return a bunch of guesses
   * that should be validated to generate a proper output.
   * 
   * @param[in] in_img cv::Mat RGB image
   * @param[out] out_img cv::Mat image which is a binary mask
   * @param[in] dnn_guesses vector of Mat that represents a set of guesses returned by the neural network 
   * @param[in] threshold  this value'll be used to measure if the dnn_guess is good enough
   */
  void postProcessSegmentation(const cv::Mat &in_img, cv::Mat &out_img,
                               const std::vector<cv::Mat> dnn_guesses, const float threshold);

  /**
   * @brief After forward the image to the network, the nertwork'll return a bunch of guesses
   * that should be validated to generate a proper output.
   * 
   * @param[in] in_img cv::Mat RGB image
   * @param[in] dnn_guesses vector of cv::Mat that represents a set of guesses returned by the neural network 
   * @param[in] threshold  this float value that'll be used to measure if the dnn_guess is good enough
   * @return the detected object
   */
  ObjectDetected postProcessDetection(const cv::Mat &in_img,
                                      const std::vector<cv::Mat> dnn_guesses,
                                      const float threshold);

  std::vector<DnnObjectClass> parseMaskRcnnClasses(const std::vector<MaskRcnnClass> &mask_rcnn_classes) const;
};