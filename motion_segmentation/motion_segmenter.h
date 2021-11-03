#pragma once
#include <opencv2/core/mat.hpp>
#include <memory>
enum class MotionSegmenterType{
    DNN,
    Geometric
};


// this class should be the base class for all MotionSegmenter classes
class MotionSegmenter
{   
    protected:
    public:

        // this method should receive a image and
        // segment it, returning a mask matrix
        virtual void segment(const cv::Mat & in_img, cv::Mat & out_img) = 0;
        
        //this is a factory method, you shall pass the type of motion segmenter
        // followed by the right arguments that segmenter needs to be created
        static std::shared_ptr<MotionSegmenter> create(MotionSegmenterType type, ...);

};