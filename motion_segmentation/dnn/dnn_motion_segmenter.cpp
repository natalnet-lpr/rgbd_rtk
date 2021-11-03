#include "dnn_motion_segmenter.h"

DNNMotionSegmenter::DNNMotionSegmenter(const std::string & model_path,const std::string & config_file): 
    model_path_(model_path), config_file_(config_file)
{
    // currently just support CAFFE MODELS
}
void DNNMotionSegmenter::segment(const cv::Mat & in_img, cv::Mat & out_img)
{

}
