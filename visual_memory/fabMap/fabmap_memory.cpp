#include "fabmap_memory.h"
#include "bowmsctrainer.hpp"
FABMapMemory::FABMapMemory(){
    vocab_train_mat_ = Mat();
    detector_ =  SURF::create(400);

}

FABMapMemory::~FABMapMemory(){
    delete detector_;


}


void FABMapMemory::addTrainDataToVocab(cv::Mat train_frame, const bool to_show)
{
    std::vector<cv::KeyPoint> kpts;
    cv::Mat descriptors;
    detector_->detectAndCompute(train_frame,Mat(),kpts,descriptors);

    vocab_train_mat_.push_back(descriptors);

    if(to_show)
    {
        cv::drawKeypoints(train_frame, kpts, train_frame);

        cv::imshow("Training data", train_frame);
    }
    

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