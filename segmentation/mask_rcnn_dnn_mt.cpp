#include <cassert>

#include "mask_rcnn_dnn_mt.h"
#include "../common/constants.h"

using namespace std;
using namespace cv;
using namespace cv::dnn;
using namespace Constants;

MaskRcnnDnnMT::MaskRcnnDnnMT(Backend backend_id,
                             Target target_id,
                             vector<MaskRcnnClass> valid_classes)
    : DnnBasedMT(mask_rcnn_model_path, mask_rcnn_pbtxt_path, "tensorflow",
                 {"detection_out_final", "detection_masks"},
                 parseMaskRcnnClasses(valid_classes),
                 backend_id,
                 target_id)
{
}

MaskRcnnDnnMT::~MaskRcnnDnnMT()
{
}

void MaskRcnnDnnMT::segment(const Mat &img_in, Mat &img_out, float threshold)
{
  const float scale_factor = 1.0;
  const bool crop = false;
  const bool swapRB = true;

  Mat blob_img = blobFromImage(img_in, scale_factor, Size(), Scalar(), swapRB, crop);

  DnnBasedMT::net_.setInput(blob_img);

  net_.forward(dnn_output_, output_layers_name_);

  postProcessSegmentation(img_in, img_out, dnn_output_, threshold);
}

ObjectDetected MaskRcnnDnnMT::detect(const Mat &img_in, float threshold)
{
  assert((false, "MaskRcnnDnnMT::detect not implemented yet"));
  // just for compile
  (void)threshold;
  (void)img_in;
}

void MaskRcnnDnnMT::postProcessSegmentation(const Mat &in_img, Mat &out_img,
                                            const vector<Mat> dnn_guesses, const float threshold)
{
  // preparate the binary image (the mask)
  out_img = Mat(in_img.rows, in_img.cols, CV_8UC1, Scalar(255, 255, 255));

  // no copy performed
  Mat detections(dnn_guesses[0]);
  Mat masks(dnn_guesses[1]);

  // Output size of masks is NxCxHxW where
  // N - number of detected boxes
  // C - number of classes (excluding background)
  // HxW - segmentation shape
  const int num_detections = detections.size[2];
  const int num_classes = masks.size[1];

  // Each detection contains 7 elements,
  // @see https://github.com/matterport/Mask_RCNN/blob/master/mrcnn/model.py#L2487
  // - N - image index (useful just for sequence of images)
  // - class id
  // - score
  // - left coordinate
  // - top coordinate
  // - right coordinate
  // - bottom coordinate
  detections = detections.reshape(1, detections.total() / 7);

  for (int i = 0; i < num_detections; i++)
  {
    float score = detections.at<float>(i, 2);

    if (score > threshold)
    {
      uint8_t class_id = uint8_t(detections.at<float>(i, 1));
      vector<DnnObjectClass>::iterator it = find(valid_classes_.begin(), valid_classes_.end(), class_id);
      if (it != valid_classes_.end())
      {
        int left = int(in_img.cols * detections.at<float>(i, 3));
        int top = int(in_img.rows * detections.at<float>(i, 4));
        int right = int(in_img.cols * detections.at<float>(i, 5));
        int bottom = int(in_img.rows * detections.at<float>(i, 6));

        left = max(0, min(left, in_img.cols - 1));
        top = max(0, min(top, in_img.rows - 1));
        right = max(0, min(right, in_img.cols - 1));
        bottom = max(0, min(bottom, in_img.rows - 1));

        Rect box = Rect(left, top, right - left + 1, bottom - top + 1);

        // Extract the mask for the object
        Mat mask(masks.size[2], masks.size[3], CV_32F, masks.ptr<float>(i, class_id));

        Rect roi(Point(box.x, box.y), Point(box.x + box.width, box.y + box.height));

        resize(mask, mask, box.size());

        // convert the mask to a binary image
        mask = mask > threshold;
        bitwise_not(mask, mask);

        Mat mask_roi = out_img(roi);

        mask.copyTo(mask_roi);
      }
    }
  }
}

vector<DnnObjectClass> MaskRcnnDnnMT::parseMaskRcnnClasses(const vector<MaskRcnnClass> &mask_rcnn_classes) const
{
  vector<DnnObjectClass> dnn_classes;

  for (size_t i = 0; i < mask_rcnn_classes.size(); i++)
  {
    DnnObjectClass dnn_class = parseMaskRcnnClass(mask_rcnn_classes[i]);
    dnn_classes.push_back(dnn_class);
  }

  return dnn_classes;
}

DnnObjectClass MaskRcnnDnnMT::parseMaskRcnnClass(const MaskRcnnClass &mask_rcnn_class) const
{

  function<uint8_t(MaskRcnnClass)> toUint8 = [](MaskRcnnClass mask_rcnn_class)
  {
    return static_cast<uint8_t>(mask_rcnn_class);
  };

  DnnObjectClass dnn_class("Invalid", toUint8(MaskRcnnClass::InvalidClass));

  switch (mask_rcnn_class)
  {
  case MaskRcnnClass::Person:
    dnn_class.name_ = "Person";
    dnn_class.code_ = toUint8(MaskRcnnClass::Person);
    break;
  case MaskRcnnClass::Bicycle:
    dnn_class.name_ = "Bicycle";
    dnn_class.code_ = toUint8(MaskRcnnClass::Bicycle);
    break;
  case MaskRcnnClass::Car:
    dnn_class.name_ = "Car";
    dnn_class.code_ = toUint8(MaskRcnnClass::Car);
    break;
  case MaskRcnnClass::Motorcycle:
    dnn_class.name_ = "Motorcycle";
    dnn_class.code_ = toUint8(MaskRcnnClass::Motorcycle);
    break;
  case MaskRcnnClass::Airplane:
    dnn_class.name_ = "Airplane";
    dnn_class.code_ = toUint8(MaskRcnnClass::Airplane);
    break;
  case MaskRcnnClass::Bus:
    dnn_class.name_ = "Bus";
    dnn_class.code_ = toUint8(MaskRcnnClass::Bus);
    break;
  case MaskRcnnClass::Train:
    dnn_class.name_ = "Train";
    dnn_class.code_ = toUint8(MaskRcnnClass::Train);
    break;
  case MaskRcnnClass::Truck:
    dnn_class.name_ = "Truck";
    dnn_class.code_ = toUint8(MaskRcnnClass::Truck);
    break;
  case MaskRcnnClass::Boat:
    dnn_class.name_ = "Boat";
    dnn_class.code_ = toUint8(MaskRcnnClass::Boat);
    break;
  case MaskRcnnClass::TrafficLight:
    dnn_class.name_ = "TrafficLight";
    dnn_class.code_ = toUint8(MaskRcnnClass::TrafficLight);
    break;
  case MaskRcnnClass::FireHydrant:
    dnn_class.name_ = "FireHydrant";
    dnn_class.code_ = toUint8(MaskRcnnClass::FireHydrant);
    break;
  case MaskRcnnClass::StopSign:
    dnn_class.name_ = "StopSign";
    dnn_class.code_ = toUint8(MaskRcnnClass::StopSign);
    break;
  case MaskRcnnClass::ParkingMeter:
    dnn_class.name_ = "ParkingMeter";
    dnn_class.code_ = toUint8(MaskRcnnClass::ParkingMeter);
    break;
  case MaskRcnnClass::Bench:
    dnn_class.name_ = "Bench";
    dnn_class.code_ = toUint8(MaskRcnnClass::Bench);
    break;
  case MaskRcnnClass::Bird:
    dnn_class.name_ = "Bird";
    dnn_class.code_ = toUint8(MaskRcnnClass::Bird);
    break;
  case MaskRcnnClass::Cat:
    dnn_class.name_ = "Cat";
    dnn_class.code_ = toUint8(MaskRcnnClass::Cat);
    break;
  case MaskRcnnClass::Dog:
    dnn_class.name_ = "Dog";
    dnn_class.code_ = toUint8(MaskRcnnClass::Dog);
    break;
  case MaskRcnnClass::Horse:
    dnn_class.name_ = "Horse";
    dnn_class.code_ = toUint8(MaskRcnnClass::Horse);
    break;
  case MaskRcnnClass::Sheep:
    dnn_class.name_ = "Sheep";
    dnn_class.code_ = toUint8(MaskRcnnClass::Sheep);
    break;
  case MaskRcnnClass::Cow:
    dnn_class.name_ = "Cow";
    dnn_class.code_ = toUint8(MaskRcnnClass::Cow);
    break;
  case MaskRcnnClass::Elephant:
    dnn_class.name_ = "Elephant";
    dnn_class.code_ = toUint8(MaskRcnnClass::Elephant);
    break;
  case MaskRcnnClass::Bear:
    dnn_class.name_ = "Bear";
    dnn_class.code_ = toUint8(MaskRcnnClass::Bear);
    break;
  case MaskRcnnClass::Zebra:
    dnn_class.name_ = "Zebra";
    dnn_class.code_ = toUint8(MaskRcnnClass::Zebra);
    break;
  case MaskRcnnClass::Giraffe:
    dnn_class.name_ = "Giraffe";
    dnn_class.code_ = toUint8(MaskRcnnClass::Giraffe);
    break;
  case MaskRcnnClass::Backpack:
    dnn_class.name_ = "Backpack";
    dnn_class.code_ = toUint8(MaskRcnnClass::Backpack);
    break;
  case MaskRcnnClass::Umbrella:
    dnn_class.name_ = "Umbrella";
    dnn_class.code_ = toUint8(MaskRcnnClass::Umbrella);
    break;
  case MaskRcnnClass::Handbag:
    dnn_class.name_ = "Handbag";
    dnn_class.code_ = toUint8(MaskRcnnClass::Handbag);
    break;
  case MaskRcnnClass::Tie:
    dnn_class.name_ = "Tie";
    dnn_class.code_ = toUint8(MaskRcnnClass::Tie);
    break;
  case MaskRcnnClass::Suitcase:
    dnn_class.name_ = "Suitcase";
    dnn_class.code_ = toUint8(MaskRcnnClass::Suitcase);
    break;
  case MaskRcnnClass::Frisbee:
    dnn_class.name_ = "Frisbee";
    dnn_class.code_ = toUint8(MaskRcnnClass::Frisbee);
    break;
  case MaskRcnnClass::Skis:
    dnn_class.name_ = "Skis";
    dnn_class.code_ = toUint8(MaskRcnnClass::Skis);
    break;
  case MaskRcnnClass::Snowboard:
    dnn_class.name_ = "Snowboard";
    dnn_class.code_ = toUint8(MaskRcnnClass::Snowboard);
    break;
  case MaskRcnnClass::SportsBall:
    dnn_class.name_ = "SportsBall";
    dnn_class.code_ = toUint8(MaskRcnnClass::SportsBall);
    break;
  case MaskRcnnClass::Kite:
    dnn_class.name_ = "Kite";
    dnn_class.code_ = toUint8(MaskRcnnClass::Kite);
    break;
  case MaskRcnnClass::BaseballBat:
    dnn_class.name_ = "BaseballBat";
    dnn_class.code_ = toUint8(MaskRcnnClass::BaseballBat);
    break;
  case MaskRcnnClass::BaseballGlove:
    dnn_class.name_ = "BaseballGlove";
    dnn_class.code_ = toUint8(MaskRcnnClass::BaseballGlove);
    break;
  case MaskRcnnClass::Skateboard:
    dnn_class.name_ = "Skateboard";
    dnn_class.code_ = toUint8(MaskRcnnClass::Skateboard);
    break;
  case MaskRcnnClass::Surfboard:
    dnn_class.name_ = "Surfboard";
    dnn_class.code_ = toUint8(MaskRcnnClass::Surfboard);
    break;
  case MaskRcnnClass::TennisRacket:
    dnn_class.name_ = "TennisRacket";
    dnn_class.code_ = toUint8(MaskRcnnClass::TennisRacket);
    break;
  case MaskRcnnClass::Bottle:
    dnn_class.name_ = "Bottle";
    dnn_class.code_ = toUint8(MaskRcnnClass::Bottle);
    break;
  case MaskRcnnClass::WineGlass:
    dnn_class.name_ = "WineGlass";
    dnn_class.code_ = toUint8(MaskRcnnClass::WineGlass);
    break;
  case MaskRcnnClass::Cup:
    dnn_class.name_ = "Cup";
    dnn_class.code_ = toUint8(MaskRcnnClass::Cup);
    break;
  case MaskRcnnClass::Fork:
    dnn_class.name_ = "Fork";
    dnn_class.code_ = toUint8(MaskRcnnClass::Fork);
    break;
  case MaskRcnnClass::Knife:
    dnn_class.name_ = "Knife";
    dnn_class.code_ = toUint8(MaskRcnnClass::Knife);
    break;
  case MaskRcnnClass::Spoon:
    dnn_class.name_ = "Spoon";
    dnn_class.code_ = toUint8(MaskRcnnClass::Spoon);
    break;
  case MaskRcnnClass::Bowl:
    dnn_class.name_ = "Bowl";
    dnn_class.code_ = toUint8(MaskRcnnClass::Bowl);
    break;
  case MaskRcnnClass::Banana:
    dnn_class.name_ = "Banana";
    dnn_class.code_ = toUint8(MaskRcnnClass::Banana);
    break;
  case MaskRcnnClass::Apple:
    dnn_class.name_ = "Apple";
    dnn_class.code_ = toUint8(MaskRcnnClass::Apple);
    break;
  case MaskRcnnClass::Sandwich:
    dnn_class.name_ = "Sandwich";
    dnn_class.code_ = toUint8(MaskRcnnClass::Sandwich);
    break;
  case MaskRcnnClass::Orange:
    dnn_class.name_ = "Orange";
    dnn_class.code_ = toUint8(MaskRcnnClass::Orange);
    break;
  case MaskRcnnClass::Broccoli:
    dnn_class.name_ = "Broccoli";
    dnn_class.code_ = toUint8(MaskRcnnClass::Broccoli);
    break;
  case MaskRcnnClass::Carrot:
    dnn_class.name_ = "Carrot";
    dnn_class.code_ = toUint8(MaskRcnnClass::Carrot);
    break;
  case MaskRcnnClass::HotDog:
    dnn_class.name_ = "HotDog";
    dnn_class.code_ = toUint8(MaskRcnnClass::HotDog);
    break;
  case MaskRcnnClass::Pizza:
    dnn_class.name_ = "Pizza";
    dnn_class.code_ = toUint8(MaskRcnnClass::Pizza);
    break;
  case MaskRcnnClass::Donut:
    dnn_class.name_ = "Donut";
    dnn_class.code_ = toUint8(MaskRcnnClass::Donut);
    break;
  case MaskRcnnClass::Cake:
    dnn_class.name_ = "Cake";
    dnn_class.code_ = toUint8(MaskRcnnClass::Cake);
    break;
  case MaskRcnnClass::Chair:
    dnn_class.name_ = "Chair";
    dnn_class.code_ = toUint8(MaskRcnnClass::Chair);
    break;
  case MaskRcnnClass::Couch:
    dnn_class.name_ = "Couch";
    dnn_class.code_ = toUint8(MaskRcnnClass::Couch);
    break;
  case MaskRcnnClass::PottedPlant:
    dnn_class.name_ = "PottedPlant";
    dnn_class.code_ = toUint8(MaskRcnnClass::PottedPlant);
    break;
  case MaskRcnnClass::Bed:
    dnn_class.name_ = "Bed";
    dnn_class.code_ = toUint8(MaskRcnnClass::Bed);
    break;
  case MaskRcnnClass::DiningTable:
    dnn_class.name_ = "DiningTable";
    dnn_class.code_ = toUint8(MaskRcnnClass::DiningTable);
    break;
  case MaskRcnnClass::Toilet:
    dnn_class.name_ = "Toilet";
    dnn_class.code_ = toUint8(MaskRcnnClass::Toilet);
    break;
  case MaskRcnnClass::Tv:
    dnn_class.name_ = "Tv";
    dnn_class.code_ = toUint8(MaskRcnnClass::Tv);
    break;
  case MaskRcnnClass::Laptop:
    dnn_class.name_ = "Laptop";
    dnn_class.code_ = toUint8(MaskRcnnClass::Laptop);
    break;
  case MaskRcnnClass::Mouse:
    dnn_class.name_ = "Mouse";
    dnn_class.code_ = toUint8(MaskRcnnClass::Mouse);
    break;
  case MaskRcnnClass::Remote:
    dnn_class.name_ = "Remote";
    dnn_class.code_ = toUint8(MaskRcnnClass::Remote);
    break;
  case MaskRcnnClass::Keyboard:
    dnn_class.name_ = "Keyboard";
    dnn_class.code_ = toUint8(MaskRcnnClass::Keyboard);
    break;
  case MaskRcnnClass::CellPhone:
    dnn_class.name_ = "CellPhone";
    dnn_class.code_ = toUint8(MaskRcnnClass::CellPhone);
    break;
  case MaskRcnnClass::Microwave:
    dnn_class.name_ = "Microwave";
    dnn_class.code_ = toUint8(MaskRcnnClass::Microwave);
    break;
  case MaskRcnnClass::Oven:
    dnn_class.name_ = "Oven";
    dnn_class.code_ = toUint8(MaskRcnnClass::Oven);
    break;
  case MaskRcnnClass::Toaster:
    dnn_class.name_ = "Toaster";
    dnn_class.code_ = toUint8(MaskRcnnClass::Toaster);
    break;
  case MaskRcnnClass::Sink:
    dnn_class.name_ = "Sink";
    dnn_class.code_ = toUint8(MaskRcnnClass::Sink);
    break;
  case MaskRcnnClass::Refrigerator:
    dnn_class.name_ = "Refrigerator";
    dnn_class.code_ = toUint8(MaskRcnnClass::Refrigerator);
    break;
  case MaskRcnnClass::Book:
    dnn_class.name_ = "Book";
    dnn_class.code_ = toUint8(MaskRcnnClass::Book);
    break;
  case MaskRcnnClass::Clock:
    dnn_class.name_ = "Clock";
    dnn_class.code_ = toUint8(MaskRcnnClass::Clock);
    break;
  case MaskRcnnClass::Vase:
    dnn_class.name_ = "Vase";
    dnn_class.code_ = toUint8(MaskRcnnClass::Vase);
    break;
  case MaskRcnnClass::Scissors:
    dnn_class.name_ = "Scissors";
    dnn_class.code_ = toUint8(MaskRcnnClass::Scissors);
    break;
  case MaskRcnnClass::TeddyBear:
    dnn_class.name_ = "TeddyBear";
    dnn_class.code_ = toUint8(MaskRcnnClass::TeddyBear);
    break;
  case MaskRcnnClass::HairDrier:
    dnn_class.name_ = "HairDrier";
    dnn_class.code_ = toUint8(MaskRcnnClass::HairDrier);
    break;
  case MaskRcnnClass::Toothbrush:
    dnn_class.name_ = "Toothbrush";
    dnn_class.code_ = toUint8(MaskRcnnClass::Toothbrush);
    break;
  }

  return dnn_class;
}