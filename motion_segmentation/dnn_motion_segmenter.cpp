#include "dnn_motion_segmenter.h"

DNNMotionSegmenter::DNNMotionSegmenter(std::string model_path_)
{
    
}
void DNNMotionSegmenter::segment(const cv::Mat & in_img, cv::Mat & out_img)
{

}

std::shared_ptr<DNNMotionSegmenter> DNNMotionSegmenter::createDNNMotionSegmenter(MotionSegmenterType type, std::string model_path)
{
    return std::make_shared<DNNMotionSegmenter>(model_path);
}