#pragma once

#include "../motion_segmenter.h"
#include <string>
#include <opencv2/dnn.hpp>
#include <vector>
#include <opencv2/core.hpp>
#include <unordered_map>
// This class should be the base class for all 
// DNN methods
typedef std::unordered_map<int,std::string> classMap;
class DNNMotionSegmenter : public MotionSegmenter
{
     //private:
        //void postProcess(cv::Mat & frame, const std::vector<cv::Mat> outputs,float threshold); 
    public:
        DNNMotionSegmenter(){};

        // DNN Motion Segmenter constructor
        //  all dnn classes need to have a model file
        DNNMotionSegmenter(const std::string & model_path, const std::string & config_file,classMap && valid_classes_map); 

        inline void setClassesMap(classMap && valid_classes_map)
        {
            classes_map_ = std::move(valid_classes_map);
            valid_classes_map.clear();
        }
        inline void setClassesMap(const classMap & valid_classes_map)
        {
            classes_map_ = valid_classes_map;
        }

        // This method should be moved to the child class,
        // but for now, I will place it here because we just
        // have 1 DNN segmentation method
        virtual void segment( const cv::Mat & in_img, cv::Mat & out_img) override;
    private:
        
        void postProcess(const cv::Mat & in_img,cv::Mat & out_img, const std::vector<cv::Mat> outputs,float threshold=0.3) const;

    protected:

        std::string model_path_;
        std::string config_file_;
        cv::dnn::Net net_;
        // we store here the class id 
        // and the label of the class
        // e.g: 
        //  key(0) map to -> class Person in coco dataset
        //  see https://github.com/tensorflow/models/blob/master/research/object_detection/data/mscoco_label_map.pbtxt
        classMap classes_map_;

};
