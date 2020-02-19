#include "fabmap_memory.h"
#include "bowmsctrainer.hpp"
FABMapMemory::FABMapMemory(){
    vocab_train_mat_ = Mat();
    detector_ =  SURF::create(400);

}

FABMapMemory::~FABMapMemory(){

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
  ;
    fs.release();
   

}



int FABMapMemory::trainVocabulary(std::string vocabPath,
                    std::string vocabTrainDataPath,
                    double clusterRadius)
{

    //ensure not overwriting a vocabulary
    std::ifstream checker;
    checker.open(vocabPath.c_str());
    if(checker.is_open()) {
        std::cerr << vocabPath << ": Vocabulary already present" <<
                     std::endl;
        checker.close();
        return -1;
    }

    std::cout << "Loading vocabulary training data" << std::endl;

    cv::FileStorage fs;

    //load in vocab training data
    fs.open(vocabTrainDataPath, cv::FileStorage::READ);
    cv::Mat vocabTrainData;
    fs["VocabTrainData"] >> vocabTrainData;
    if (vocabTrainData.empty()) {
        std::cerr << vocabTrainDataPath << ": Training Data not found" <<
                     std::endl;
        return -1;
    }
    fs.release();

    std::cout << "Performing clustering" << std::endl;

    //uses Modified Sequential Clustering to train a vocabulary
    of2::BOWMSCTrainer trainer(clusterRadius);
    trainer.add(vocabTrainData);
    cv::Mat vocab = trainer.cluster();

    //save the vocabulary
    std::cout << "Saving vocabulary" << std::endl;
    fs.open(vocabPath, cv::FileStorage::WRITE);
    fs << "Vocabulary" << vocab;
    fs.release();

    return 0;
}

