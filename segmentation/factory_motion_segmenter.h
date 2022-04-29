#pragma once

#include "motion_segmenter.h"
#include "dnn/dnn_motion_segmenter.h"
#include "geometric/geometric_motion_segmenter.h"

#include <stdarg.h>
#include <memory>

enum class MotionSegmenterType : uint8_t
{
    DNN,
    Geometric
};

class FactoryMotionSegmenter
{
public:
    // possible to construct
    // see https://en.cppreference.com/w/cpp/language/sfinae
    template <typename T, typename... Args>
    static std::enable_if_t<std::is_constructible<T, Args...>::value, std::shared_ptr<T>>
    create(Args &&...args)
    {
        return std::make_shared<T>(std::forward<Args>(args)...);
    }
    // possible to construct
    template <typename T, typename... Args>
    static std::enable_if_t<!std::is_constructible<T, Args...>::value, std::shared_ptr<T>>
    create(Args &&...args)
    {
        // Bruno must revise this
        static_assert(std::is_constructible<T, Args...>::value, "error to create Motion Segmenter"); // you could use tis approach to stop compile if could not create
        return nullptr;
    }
};