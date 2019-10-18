#ifndef INCLUDE_FABMAP_MEMORY_H_
#define INCLUDE_FABMAP_MEMORY_H_
#include <openfabmap.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>
#include <vector>
#include <string>
#include <surf_tracker.h>


#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"




#include <iostream>
using namespace cv;
using namespace cv::xfeatures2d;


class FABMapMemory{
    //to do :
    // integrar os módulos tracker (que ainda não existe), a esse módulo, por enquanto trabalharemos
    // com apenas o surf_tracker

    protected:
        cv::Mat vocab_train_mat_; 

    public:
        FABMapMemory(){
                vocab_train_mat_ = Mat();

        }


        void addTrainDataToVocab(cv::Mat train_frame,cv::Ptr<cv::FeatureDetector> &detector,cv::Ptr<cv::DescriptorExtractor> &extractor);
        void generateVocabTrainDataFile(std::string filename);
        //input: a untrained vocabulary that will be trained and returned by this function
        Mat trainVocabulary(Mat untrained_vocab,double clusterRadius);



};

#endif