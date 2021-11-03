#include <iostream>
#include <factory_motion_segmenter.h>
using namespace std;

int main()
{       
    auto segmenter = FactoryMotionSegmenter::create(MotionSegmenterType::DNN,"model_path","config_file");

    return 0;
}