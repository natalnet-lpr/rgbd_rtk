#pragma once

#include "../motion_segmenter.h"
#include <string>
#include <opencv2/dnn.hpp>
// This class should be the base class for all 
// DNN methods
class DNNMotionSegmenter : public MotionSegmenter
{
    public:
        DNNMotionSegmenter() = delete;

        // DNN Motion Segmenter constructor
        //  all dnn classes need to have a model file
        DNNMotionSegmenter(const std::string & model_path, const std::string & config_file); 

        // This method should be moved to the child class,
        // but for now, I will place it here because we just
        // have 1 DNN segmentation method
        virtual void segment(const cv::Mat & in_img, cv::Mat & out_img);
            
    protected:

        std::string model_path_;
        std::string config_file_;
        cv::dnn::Net net_;
};
