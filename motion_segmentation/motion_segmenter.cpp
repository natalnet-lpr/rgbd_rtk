#include "motion_segmenter.h"
#include "dnn_motion_segmenter.h"
#include <stdarg.h>
std::shared_ptr<MotionSegmenter> MotionSegmenter::create(MotionSegmenterType type , ...){
    
    va_list va;
    va_start(va, type);

    if(type == MotionSegmenterType::DNN)
    {
        std::string model_path;
        {
            model_path = va_arg(va, char*);
        }

        return DNNMotionSegmenter::createDNNMotionSegmenter(type,model_path);
    }
    va_end(va);
}