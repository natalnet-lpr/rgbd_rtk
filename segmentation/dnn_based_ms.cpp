#include "dnn_based_ms.h"

using namespace std;

DnnBasedMS::DnnBasedMS(const string &model, const string &config, const string &framework,
                       const vector<string> output_layers_name,
                       const unordered_map<uint8_t /*class id*/, string /*class name*/> valid_classes_map,
                       Backend backend_id,
                       Target target_id)
    : output_layers_name_(output_layers_name),
      valid_classes_map_(valid_classes_map),
      backend_id_(backend_id),
      target_id_(target_id)
{
  net_ = readNet(model, config, framework);
  net_.setPreferableBackend(backend_id);
  net_.setPreferableTarget(target_id);
}

DnnBasedMS::~DnnBasedMS()
{
}