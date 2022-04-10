#include "dnn_based_mt.h"
#include <algorithm>
using namespace std;
using namespace cv;
using namespace cv::dnn;

DnnBasedMT::DnnBasedMT(const string &model, const string &config, const string &framework,
                       const vector<string> output_layers_name,
                       const std::vector<DnnObjectClass> valid_classes,
                       Backend backend_id,
                       Target target_id)
    : output_layers_name_(output_layers_name),
      backend_id_(backend_id),
      target_id_(target_id),
      valid_classes_(valid_classes)
{
  net_ = readNet(model, config, framework);
  net_.setPreferableBackend(backend_id);
  net_.setPreferableTarget(target_id);
}

DnnBasedMT::~DnnBasedMT()
{
}