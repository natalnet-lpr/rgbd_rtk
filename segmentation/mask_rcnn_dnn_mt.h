#pragma once

#include "dnn_based_mt.h"

enum class MaskRcnnClass : uint8_t
{
  Person = 0,
  Bicycle,
  Car,
  Motorcycle,
  Airplane,
  Bus,
  Train,
  Truck,
  Boat,
  TrafficLight,
  FireHydrant,
  StopSign,
  ParkingMeter,
  Bench,
  Bird,
  Cat,
  Dog,
  Horse,
  Sheep,
  Cow,
  Elephant,
  Bear,
  Zebra,
  Giraffe,
  Backpack,
  Umbrella,
  Handbag,
  Tie,
  Suitcase,
  Frisbee,
  Skis,
  Snowboard,
  SportsBall,
  Kite,
  BaseballBat,
  BaseballGlove,
  Skateboard,
  Surfboard,
  TennisRacket,
  Bottle,
  WineGlass,
  Cup,
  Fork,
  Knife,
  Spoon,
  Bowl,
  Banana,
  Apple,
  Sandwich,
  Orange,
  Broccoli,
  Carrot,
  HotDog,
  Pizza,
  Donut,
  Cake,
  Chair,
  Couch,
  PottedPlant,
  Bed,
  DiningTable,
  Toilet,
  Tv,
  Laptop,
  Mouse,
  Remote,
  Keyboard,
  CellPhone,
  Microwave,
  Oven,
  Toaster,
  Sink,
  Refrigerator,
  Book,
  Clock,
  Vase,
  Scissors,
  TeddyBear,
  HairDrier,
  Toothbrush,

  InvalidClass = 254

};

/**
 * @brief Do the instance segmentation task using the MaskRCNN model
 * @see https://arxiv.org/abs/1703.06870
 * 
 */
class MaskRcnnDnnMT : public DnnBasedMT
{
public:
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

  DnnObjectClass parseMaskRcnnClass(const MaskRcnnClass &mask_rcnn_class) const;
};