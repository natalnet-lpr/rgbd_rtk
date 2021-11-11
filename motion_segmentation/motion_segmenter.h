#pragma once
#include <opencv2/core/mat.hpp>
#include <memory>



// this class should be the base class for all MotionSegmenter classes
class MotionSegmenter
{   
    protected:
    public:
        // this method should receive a image and
        // segment it, returning a mask matrix
        virtual void segment(const cv::Mat & in_img, cv::Mat & out_img) = 0 ;
        
};