#include <iostream>
#include <motion_segmenter.h>
#include <dnn_motion_segmenter.h>
using namespace std;

int main()
{       
    auto segmenter = MotionSegmenter::create(MotionSegmenterType::DNN);

    return 0;
}