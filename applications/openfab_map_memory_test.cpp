#include <iostream>
#include "fabMap/fabmap_memory.h"



using namespace std;


int main(){



    FABMapMemory memory;
    cv::Mat img;
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;

    memory.addTrainDataToVocab(img,detector,extractor);



    return 0;
}