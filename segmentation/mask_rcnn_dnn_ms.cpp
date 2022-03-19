#include <cassert>

#include "mask_rcnn_dnn_ms.h"
#include "../common/constants.h"

using namespace Constants;

MaskRcnnDnnMS::MaskRcnnDnnMS(uint8_t backend_id,
                             uint8_t target_id)
    : DnnBasedMS(mask_rcnn_model_path, mask_rcnn_pbtxt_path, "tensorflow",
                 {"detection_out_final", "detection_masks"},
                 backend_id,
                 target_id)
{
}

void MaskRcnnDnnMS::segment(const Mat &img_in, Mat &img_out, float threshold) const
{
  assert((false, "MaskRcnnDnnMS::segment() not implemented yet"));
  // just for compile
  (void)threshold;
  (void)img_in;
  (void)img_out;
}

ObjectDetected MaskRcnnDnnMS::detect(const Mat &img_in, float threshold) const
{
  assert((false, "MaskRcnnDnnMS::detect not implemented yet"));
  // just for compile
  (void)threshold;
  (void)img_in;
}
