#include <visual_memory.h>
#include "opencv2/imgproc.hpp"

#include <iostream>

VisualMemory::VisualMemory(){
    matcher_ = new cv::FlannBasedMatcher();

}
VisualMemory::VisualMemory(std::vector<cv::Mat> trainDescriptorsSet){
    trainDescriptorsSet_ = trainDescriptorsSet;
    matcher_ = new cv::FlannBasedMatcher();

}
VisualMemory::VisualMemory(cv::Ptr<cv::DescriptorMatcher> matcher){
    this->matcher_ = matcher;
}
void VisualMemory::create(std::vector<cv::Mat> trainDescriptorsSet){
    trainDescriptorsSet_ = trainDescriptorsSet;


}


 void VisualMemory::add(cv::Mat trainDescriptors){
    //trainDescriptorsSet_.push_back(trainDescriptors);
    matcher_->add(trainDescriptors);
 }


bool VisualMemory::sort_assistant(std::pair<int,int> a,std::pair<int,int>  b){
    return (a.first>b.first);

}

std::vector<int> VisualMemory::searchDescriptor(cv::Mat queryDescriptor,int n){
    std::vector< std::vector<cv::DMatch>> matches; //isso é para o knnmatches       
    //std::vector<DMatch> matches;  

    if(!matcher_->getTrainDescriptors().empty())
    {
        matcher_->train();
        matcher_->knnMatch(queryDescriptor,matches,2); //usando o knnmatches
        //matcher_->match(queryDescriptor,matches);


    }
    else {
        matcher_->knnMatch(queryDescriptor,trainDescriptorsSet_,matches,2); //usando o knnmatches
        //matcher_->match(queryDescriptor,trainDescriptors_,matches);


    }

    //the first one is the amount of matches
    //the second is the imgidx
    
    std::vector< std::pair<int,int> > cont(matches.size());
   
    for(int i=0;i<matches.size();i++){// isso é para o knn matches

        for(int j=0;j<matches[i].size();j++){
            int idx = matches[i][j].imgIdx;
           
            cont[idx].first ++;
            

             cont[idx].second = idx;

            
        }

    }


   /* for(int j=0;j<matches.size();j++){
            int idx = matches[j].imgIdx;
           
            cont[idx].first ++;
            

             cont[idx].second = idx;

            
    }
    */
    
    std::sort(cont.begin(),cont.end(),sort_assistant);

    std::vector<int> output;

    for(int i=0;i<n;i++){
        std::cout<<"índice da imagem "<< cont[i].second<<" quantidade de matchings "<<cont[i].first<<std::endl;
        output.push_back(cont[i].second);

    }  

    return output;


}

std::vector<int> VisualMemory::searchDescriptor(cv::Mat queryDescriptor, int n, cv::Ptr<cv::DescriptorMatcher> matcher){
    
    std::vector< std::vector<cv::DMatch>> matches; //isso é para o knnmatches       
    matcher->train();
    matcher->knnMatch(queryDescriptor,matches,3); //usando o knnmatches
   
    std::vector< std::pair<int,int> > cont(matches.size());
    
    for(int i=0;i<matches.size();i++){// isso é para o knn matches

        for(int j=0;j<matches[i].size();j++){
            int idx = matches[i][j].imgIdx;
           
            cont[idx].first ++;
            

             cont[idx].second = idx;

            
        }

    }
    
    std::sort(cont.begin(),cont.end(),sort_assistant);

    std::vector<int> output;

    for(int i=0;i<n;i++){
        std::cout<<"índice da imagem "<< cont[i].second<<" quantidade de matchings "<<cont[i].first<<std::endl;
        output.push_back(cont[i].second);

    }
    return output;


}
std::vector<int> VisualMemory::searchImage(cv::Mat img){
    cv::Ptr<cv::FeatureDetector>  detector = SURF::create(400);;
    cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);

    std::vector<cv::KeyPoint> KPs;
    cv::Mat descriptors;
    detector->detect(img, KPs);
    detector->compute(img, KPs,descriptors);
   
    return searchDescriptor(descriptors,5);


    
}