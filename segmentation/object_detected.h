#pragma once
#include <opencv2/opencv.hpp>
#include <string>

using cv::Rect;
using std::string;

/**
 * @brief Store a custom object related to any class
 * 
 */
struct ObjectDetected
{
  string class_name;
  uint8_t class_id;
  Rect b_box;
};