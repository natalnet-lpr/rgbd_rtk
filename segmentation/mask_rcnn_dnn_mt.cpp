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
