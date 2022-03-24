#pragma once
#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Store a custom object related to any class
 * 
 */
struct ObjectDetected
{
  std::string class_name;
  uint8_t class_id;
  cv::Rect b_box;
};