#include "flexible_feature_detector.h"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

FlexibleFeatureDetector::FlexibleFeatureDetector()
{
    feature_detector_ = ORB::create();
    descriptor_extractor_ = ORB::create();
    initialized_ = false;
    matcher_ = DescriptorMatcher::create("BruteForce");
}

FlexibleFeatureDetector::FlexibleFeatureDetector(Ptr<cv::FeatureDetector> feature_detector, Ptr<cv::DescriptorExtractor> descriptor_extractor, Ptr<DescriptorMatcher> matcher)
{
    feature_detector_ = feature_detector;
    descriptor_extractor_ = descriptor_extractor;
    matcher_ = matcher;
    initialized_ = false;
}

void FlexibleFeatureDetector::detect(Mat curr_frame)
{
    curr_KPs_.clear();
    feature_detector_->detect(curr_frame_gray_, curr_KPs_);
    descriptor_extractor_->compute(curr_frame_gray_, curr_KPs_, curr_descriptors_);
}

void FlexibleFeatureDetector::getGoodMatches(double threshold)
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

void FlexibleFeatureDetector::updateBuffers()
{
    std::swap(curr_KPs_, prev_KPs_);
    std::swap(curr_descriptors_, prev_descriptors_);
}

void FlexibleFeatureDetector::track(Mat curr_frame)
{
    // Make a grayscale copy of the current frame
    cvtColor(curr_frame, curr_frame_gray_, CV_BGR2GRAY);

    //Update the internal buffers
    updateBuffers();

    //Detect KeyPoints and extract descriptors on the current frame
    detect(curr_frame_gray_);

    if (!initialized_)
    {
        initialized_ = true;
    }
    else
    {
        matcher_->match(curr_descriptors_, prev_descriptors_, matches_);
        getGoodMatches(0.1);
    }

    cv::swap(curr_frame_gray_, prev_frame_gray_);
}


