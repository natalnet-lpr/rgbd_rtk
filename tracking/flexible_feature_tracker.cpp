#include "flexible_feature_tracker.h"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

FlexibleFeatureTracker::FlexibleFeatureTracker()
{
    feature_detector_ = ORB::create();
    descriptor_extractor_ = ORB::create();
    matcher_ = DescriptorMatcher::create("BruteForce");
    initialized_ = false;
    frame_idx_ = 0; 
    
}

FlexibleFeatureTracker::FlexibleFeatureTracker(Ptr<cv::FeatureDetector> feature_detector, Ptr<cv::DescriptorExtractor> descriptor_extractor, Ptr<DescriptorMatcher> matcher)
{
    feature_detector_ = feature_detector;
    descriptor_extractor_ = descriptor_extractor;
    matcher_ = matcher;
    initialized_ = false;
    frame_idx_ = 0;
}

void FlexibleFeatureTracker:: addKeypoints()
{
    for(size_t i = 0; i < curr_KPs_.size(); i++){
        
        //Create and add tracklet
        Tracklet tr(frame_idx_);
        tr.pts2D_.push_back(curr_KPs_[i].pt);
        tr.keypoint_indices_.push_back(i);
        tracklets_.push_back(tr);
    }
}

int FlexibleFeatureTracker::searchMatches(int keypoint_index){
    for (size_t i = 0; i < matches_.size(); i++)
    {
        if(keypoint_index == matches_[i].trainIdx)
        {
            return matches_[i].queryIdx;
        }
    }
    return -1;
}

void FlexibleFeatureTracker::detect(Mat curr_frame)
{
    curr_KPs_.clear();
    feature_detector_->detect(curr_frame_gray_, curr_KPs_);
    descriptor_extractor_->compute(curr_frame_gray_, curr_KPs_, curr_descriptors_);
}

void FlexibleFeatureTracker::getGoodMatches(double threshold)
{
    good_matches_.clear();

    double max_dist = 0;
    double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < prev_descriptors_.rows; i++)
    {
        double dist = matches_[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    for (int i = 0; i < matches_.size(); i++)
    {
        if (matches_[i].distance <= max(2 * min_dist, threshold))
        {
            good_matches_.push_back(matches_[i]);
        }
    }

    prev_good_Pts_.clear();
    curr_good_Pts_.clear();

    for (int i = 0; i < good_matches_.size(); i++)
    {
        prev_good_Pts_.push_back(prev_KPs_[good_matches_[i].trainIdx].pt);
        curr_good_Pts_.push_back(curr_KPs_[good_matches_[i].queryIdx].pt);
    }
}

void FlexibleFeatureTracker::updateBuffers()
{
    std::swap(curr_KPs_, prev_KPs_);
    std::swap(curr_descriptors_, prev_descriptors_);
}

void FlexibleFeatureTracker::track(Mat curr_frame)
{
    // Make a grayscale copy of the current frame
    cvtColor(curr_frame, curr_frame_gray_, CV_BGR2GRAY);

    //Update the internal buffers
    //std::cout << "entrou 5 ---" << std::endl;
    updateBuffers();

    //Detect KeyPoints and extract descriptors on the current frame
    detect(curr_frame_gray_);

    if (!initialized_)
    {
        initialized_ = true;
        //std::cout << "entrou" << std::endl;
        addKeypoints();
    }
    else
    {
        //std::cout << "entrou 4 ---" << std::endl;
        matcher_->match(curr_descriptors_, prev_descriptors_, matches_);
        
        keypoints_with_matches.resize(curr_KPs_.size());
        int tracked_pts = 0;
        // Updating the tracklets
        for (size_t i = 0; i < tracklets_.size(); i++){
            
            int keypoint_index = tracklets_[i].keypoint_indices_.back();
            int search_result = searchMatches(keypoint_index);
            
            if (search_result != -1)
            {
                tracklets_[i].pts2D_.push_back(curr_KPs_[search_result].pt);
                tracklets_[i].keypoint_indices_.push_back(search_result);
                tracklets_[tracked_pts] = tracklets_[i];
                keypoints_with_matches[search_result] = true;
                tracked_pts++;
            }
            else
            {
                // Remove the tracklet without match
                continue;    
            }
            
        }
        tracklets_.resize(tracked_pts);

        // Adding the new keypoints
        for (size_t i = 0; i < curr_KPs_.size(); i++){
            if (keypoints_with_matches[i] == false)
            {
                //Create and add tracklet
                Tracklet tr(frame_idx_);
                tr.pts2D_.push_back(curr_KPs_[i].pt);
                tr.keypoint_indices_.push_back(i);
                tracklets_.push_back(tr);
            }
        }
             

        std::cout << "\n--------------------------------------------- " << std::endl;
        std::cout << "Matches: " << std::endl;

        for(size_t i = 0; i < matches_.size(); i++)
        {
            std::cout << i << ": " << matches_[i].trainIdx << " ---> " << matches_[i].queryIdx << std::endl; 
        }
        
        std::cout << "\n--------------------------------------------- " << std::endl;
        std::cout << "Tracklets: " << std::endl;
        
        for(size_t i = 0; i < tracklets_.size(); i++)
        {
            for (size_t j = 0; j < tracklets_[i].keypoint_indices_.size(); j++) 
            {
                std::cout << tracklets_[i].keypoint_indices_[j]<< "  "; 
            }
            std::cout << "\n--------------------------------------------------------------------------------\n";
        }
        

        getGoodMatches(0.1);
        std:: cout << "CKPs size: " << curr_KPs_.size() << std:: endl;
        std:: cout << "Tracklets size: " << curr_KPs_.size() << std:: endl;
        //std::cout << "entrou 2 ---" << std::endl;
        
    }

    cv::swap(curr_frame_gray_, prev_frame_gray_);
    frame_idx_++;
    //std::cout << "entrou 3 ---" << std::endl;
}


