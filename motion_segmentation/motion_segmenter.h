#pragma once
#include <opencv2/core/mat.hpp>
#include <memory>



// this class should be the base class for all MotionSegmenter classes
class MotionSegmenter
{   
    protected:

            float threshold_;
    public:
        /**
         * 
         * @param  {cv::Mat} in_img  : frame/image
         * @param  {cv::Mat} out_img : image binary mask
         */
        virtual void segment(const cv::Mat & in_img, cv::Mat & out_img) = 0 ;
        inline virtual void setThreshold(float threshold)
        {
            threshold_ = threshold;
        }
       
};