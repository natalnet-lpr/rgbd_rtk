#include "flexible_feature_detector.h"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
using std::cout;

FlexibleFeatureDetector::FlexibleFeatureDetector()
{
    detector_ = ORB::create();
    initialized_ = false;
    matcher_ = DescriptorMatcher::create("BruteForce");
}


FlexibleFeatureDetector::FlexibleFeatureDetector(Ptr<Feature2D> detector, Ptr<DescriptorMatcher> matcher)
{
    detector_ = detector;
    matcher_ = matcher;
    initialized_ = false;
}

void FlexibleFeatureDetector::detect(Mat curr_frame)
{
    curr_KPs_.clear();
    cvtColor(curr_frame, this->curr_frame_gray_, cv::COLOR_RGB2GRAY);
    detector_->detect(curr_frame_gray_, curr_KPs_);
    detector_->compute(curr_frame_gray_, curr_KPs_, curr_descriptors_);

    curr_KPs_.swap(prev_KPs_);
    cv::swap(curr_frame_gray_, prev_frame_gray_);

    cv::swap(curr_descriptors_, prev_descriptors_);
}

void FlexibleFeatureDetector::getGoodMatches(double threshold)
{
    good_matches_.clear();

    std::vector<DMatch> matches;
    
    matcher_->match( prev_descriptors_,curr_descriptors_, matches);
    

    double max_dist = 0;
    double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < prev_descriptors_.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    for (int i = 0; i < prev_descriptors_.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist, threshold))
        {
            good_matches_.push_back(matches[i]);
        }
    }

        prev_good_Pts_.clear();
        curr_good_Pts_.clear();
        
    for(int i =0; i<good_matches_.size();i++){
            prev_good_Pts_.push_back(prev_KPs_[good_matches_[i].queryIdx].pt);
            curr_good_Pts_.push_back(curr_KPs_[good_matches_[i].trainIdx].pt);


    }
        

}
void FlexibleFeatureDetector::detectAndGetGoodMatches(Mat curr_frame)
{
    detect(curr_frame);

    if (initialized_)
    {
        getGoodMatches(0.1);
    }
    else 
        initialized_ = true;
}



void FlexibleFeatureDetector::draw(Mat prev_img, Mat curr_img){

    Mat img_matches;

    drawMatches( prev_img, prev_KPs_, curr_img, curr_KPs_,
    good_matches_, img_matches, Scalar::all(-1), Scalar::all(-1),
    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    good_matches_.clear();
    imshow("Match Image",img_matches);

}

void FlexibleFeatureDetector::searchInTree(Mat descriptors){
    vector<DMatch> matches;
    matcher_->match(descriptors,matches);
    cout<<matches.size()<<endl;
}