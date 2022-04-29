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
  cv::Rect bouding_box;

  ObjectDetected(const std::string &name, const uint8_t id, const cv::Rect &b_box) : class_name(name), class_id(id), bouding_box(b_box)
  {
  }

  /**
  * @brief Check if the object belongs to the same class
  */
  virtual bool operator==(const ObjectDetected &other)
  {
    return (other.class_id == class_id);
  }
};