#include <cassert>

#include "mask_rcnn_dnn_ms.h"
#include "../common/constants.h"

using namespace Constants;

MaskRcnnDnnMS::MaskRcnnDnnMS(uint8_t backend_id,
                             uint8_t target_id,
                             unordered_map<uint8_t, string> valid_classes_map)
    : DnnBasedMS(mask_rcnn_model_path, mask_rcnn_pbtxt_path, "tensorflow",
                 {"detection_out_final", "detection_masks"},
                 valid_classes_map,
                 backend_id,
                 target_id)
{
}

void MaskRcnnDnnMS::segment(const Mat &img_in, Mat &img_out, float threshold)
{
  const float scale_factor = 1.0;
  const bool crop = false;
  const bool swapRB = true;

  Mat blob_img = blobFromImage(img_in, scale_factor, cv::Size(), cv::Scalar(), swapRB, crop);

  DnnBasedMS::net_.setInput(blob_img);

  net_.forward(dnn_output_, output_layers_name_);

  (void)threshold;
  (void)img_out;

  postProcessSegmentation(img_in, img_out, dnn_output_, threshold);
}

ObjectDetected MaskRcnnDnnMS::detect(const Mat &img_in, float threshold)
{
  assert((false, "MaskRcnnDnnMS::detect not implemented yet"));
  // just for compile
  (void)threshold;
  (void)img_in;
}

void MaskRcnnDnnMS::postProcessSegmentation(const Mat &in_img, Mat &out_img,
                                            const vector<Mat> dnn_guesses, const float threshold)
{
  assert((false, "MaskRcnnDnnMS::postProcessSegmentation not implemented yet"));
  (void)in_img;
  (void)out_img;
  (void)dnn_guesses;
  (void)threshold;
}