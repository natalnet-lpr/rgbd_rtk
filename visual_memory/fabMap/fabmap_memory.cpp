#include "fabmap_memory.h"
#include "bowmsctrainer.hpp"
/*
FABMapMemory::FABMapMemory(){
    vocab_train_mat_ = Mat();


}


*/
void FABMapMemory::addTrainDataToVocab(cv::Mat train_frame,cv::Ptr<cv::FeatureDetector> &detector,cv::Ptr<cv::DescriptorExtractor> &extractor)
{
    std::vector<cv::KeyPoint> kpts;
    cv::Mat descriptors;

    detector->detectAndCompute(train_frame,cv::Mat(),kpts,descriptors);

    vocab_train_mat_.push_back(descriptors);


}

void FABMapMemory::generateVocabTrainDataFile(std::string filename)
{
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::WRITE);
    fs << "VocabTrainData" << vocab_train_mat_;
    fs.release();


}

Mat FABMapMemory::trainVocabulary(Mat untrained_vocab,double clusterRadius){
    
   of2::BOWMSCTrainer  trainer(clusterRadius);
    trainer.add(untrained_vocab);

    return trainer.cluster();



}