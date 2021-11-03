#pragma once
#include <iostream>
#include "motion_segmenter.h"
#include "dnn/dnn_motion_segmenter.h"
#include <stdarg.h>
#include <memory>

enum class MotionSegmenterType{
    DNN,
    Geometric
};

class FactoryMotionSegmenter{
    public:
        template <class... Args>
        static std::shared_ptr<MotionSegmenter> create( MotionSegmenterType type, Args&&... args )
        {
            if(type == MotionSegmenterType::DNN)
            {
                return std::make_shared<DNNMotionSegmenter>(std::forward<Args>(args)...);
            }
        }
        // possible to construct
        // see https://en.cppreference.com/w/cpp/language/sfinae
        template <typename T, typename... Args>
        std::enable_if_t< 
            std::is_constructible<T, Args...>::value,
            std::shared_ptr<T> 
        >
        create(Args&&... args)
        {
            return std::make_shared<T>(std::forward<Args>(args)...);
        }
        // possible to construct
        template <typename T, typename... Args>
        std::enable_if_t< 
            !std::is_constructible<T, Args...>::value,
            std::shared_ptr<T> 
        >
        create(Args&&... args)
        {
            return nullptr;
        }
    private:
        
};