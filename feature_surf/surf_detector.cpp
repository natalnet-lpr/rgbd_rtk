#include "surf_detector.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <utility>
#include <algorithm>

SurfDetector::SurfDetector()
{

    detector_ = SURF::create(400);

    initialized_ = false;
    matcher_ = new  cv::FlannBasedMatcher();
}
SurfDetector::SurfDetector(double minHessian)
{

    this->minHessian_ = minHessian;

    detector_ = SURF::create(this->minHessian_);

    initialized_ = false;
    matcher_ = new  cv::FlannBasedMatcher();
}

void SurfDetector::detect(Mat curr_frame, Mat prev_frame)
{
    //add a empty image to train images set
    //it is necessary to acess  trainImages_.back()
    // present in detectAndCompute method
    this->trainImages_.push_back(Mat());

    //convert the curr frame into a gray query image
    cvtColor(curr_frame, this->queryImage_, cv::COLOR_RGB2GRAY);

    //convert the prev frame into a gray train image

    cvtColor(prev_frame, this->trainImages_.back(), cv::COLOR_RGB2GRAY);


    //detect and compute the query keypoints and the query descriptors
    detector_->detectAndCompute(queryImage_,Mat(),queryKPs_,queryDescriptors_);

    //add a empty vector of keypoints to the vector of vectors of keypoints,
    //this is necessary to avoid segmentation fault
    this->trainKPs_.push_back(vector<KeyPoint> ());
    //the same for this lime
    this->trainDescriptors_.push_back(Mat());



    detector_->detectAndCompute(trainImages_.back(),Mat(),trainKPs_.back(),trainDescriptors_.back());

    matcher_->add(trainDescriptors_.back());

}
void SurfDetector::MatchDescriptors()
{
    refinedMatches_.clear();

    std::vector< DMatch > matches;

    matcher_->match(queryDescriptors_, trainDescriptors_.back(), matches );
    double max_dist = 0;
    double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < queryDescriptors_.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    for (int i = 0; i < queryDescriptors_.rows; i++)
    {
       if (matches[i].distance <= max(2 * min_dist, 0.1))

            refinedMatches_.push_back(matches[i]);

    }

        prev_good_Pts_.clear();
        curr_good_Pts_.clear();

    for(int i =0; i<refinedMatches_.size();i++){
            prev_good_Pts_.push_back(trainKPs_.back()[refinedMatches_[i].trainIdx].pt);
            curr_good_Pts_.push_back(queryKPs_[refinedMatches_[i].queryIdx].pt);

    }


}
void SurfDetector::detectAndMatch(Mat curr_frame, Mat prev_frame){

    detect(curr_frame, prev_frame);
    MatchDescriptors();


}

void SurfDetector::drawSurfMatches(Mat prev_img,Mat curr_img){


    Mat output;


    drawMatches(curr_img,  queryKPs_, prev_img,  trainKPs_.back(),
    refinedMatches_, output, Scalar::all(-1), Scalar::all(-1),
    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    imshow("output", output);




}

bool myfunction(pair<int,int> a,pair<int,int>  b){
    return (a.first>b.first);

}

vector<int>  SurfDetector::searchDescriptor(Mat queryDescriptor, int n){

    vector< vector<DMatch>> matches; //isso é para o knnmatches
   // vector<DMatch> matches;
    matcher_->knnMatch(queryDescriptor,trainDescriptors_,matches,2); //usando o knnmatches
 //   matcher_->match(queryDescriptor,trainDescriptors_,matches);
    //cout<<knnMatches.size()<<endl;
    //the first one is the amount of matches
    //the second is the imgidx
    vector< pair<int,int> > cont(matches.size());
    for(int i=0;i<matches.size();i++){// isso é para o knn matches

        for(int j=0;j<matches[i].size();j++){
            int idx = matches[i][j].imgIdx;

            cont[idx].first ++;


             cont[idx].second = idx;


        }

    }



    std::sort(cont.begin(),cont.end(),myfunction);

    vector<int> output;

    for(int i=0;i<n;i++){
        cout<<"índice da imagem "<< cont[i].second<<" quantidade de matchings "<<cont[i].first<<endl;
        output.push_back(cont[i].second);

    }
    return output;
}

Mat SurfDetector::getLastTrainDescriptor(){

    if(!trainDescriptors_.empty())
        return trainDescriptors_.back();
    else return Mat();
}
cv::Ptr<cv::DescriptorMatcher> SurfDetector::getMatcher(){

    return matcher_;

}