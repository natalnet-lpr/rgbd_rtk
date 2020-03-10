#include <iostream>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "wide_baseline_tracker.h"
#include <event_logger.h>


/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

void WideBaselineTracker::detect_keypoints()
{
    curr_KPs_.clear();
    feature_detector_->detect(curr_frame_gray_, curr_KPs_);
    descriptor_extractor_->compute(curr_frame_gray_, curr_KPs_, curr_descriptors_);
    logger.print(pcl::console::L_DEBUG, "[WideBaselineTracker::detect_keypoints] DEBUG: detecting keyponts...\n");
    logger.print(pcl::console::L_DEBUG, "[WideBaselineTracker::detect_keypoints] DEBUG: detected pts.: %lu\n",curr_KPs_.size());
}

void WideBaselineTracker:: add_keypoints()
{
    for(size_t i = 0; i < curr_KPs_.size(); i++){
        
        //Create and add tracklet
        Tracklet tr(frame_idx_);
        tr.pts2D_.push_back(curr_KPs_[i].pt);
        tr.keypoint_indices_.push_back(i);
        tracklets_.push_back(tr);
    }
    logger.print(pcl::console::L_DEBUG, "[WideBaselineTracker::add_keypoints DEBUG: adding keypoints...\n");
    logger.print(pcl::console::L_DEBUG, "[WideBaselineTracker::add_keypoints DEBUG: added pts.: %lu\n",tracklets_.size());
}

int WideBaselineTracker::searchMatches(int keypoint_index){
    for (size_t i = 0; i < matches_.size(); i++)
    {
        if(keypoint_index == matches_[i].trainIdx)
        {
            return matches_[i].queryIdx;
        }
    }
    return -1;
}

void WideBaselineTracker::update_buffers()
{
    std::swap(curr_KPs_, prev_KPs_);
    std::swap(curr_descriptors_, prev_descriptors_);
}

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

WideBaselineTracker::WideBaselineTracker()
{
    feature_detector_ = ORB::create();
    descriptor_extractor_ = ORB::create();
    matcher_ = DescriptorMatcher::create("BruteForce");
    initialized_ = false;
    frame_idx_ = 0; 
    
}

WideBaselineTracker::WideBaselineTracker(Ptr<cv::FeatureDetector> feature_detector, Ptr<cv::DescriptorExtractor> descriptor_extractor, Ptr<DescriptorMatcher> matcher, const bool& log_stats)
{
    feature_detector_ = feature_detector;
    descriptor_extractor_ = descriptor_extractor;
    matcher_ = matcher;
    initialized_ = false;
    frame_idx_ = 0;

    if (log_stats)
    {
        initialize_logger("timing_stats.txt","tracking_stats.txt","heatmap_stats.txt");
    }
}

void WideBaselineTracker::getGoodMatches(double threshold)
{
    good_matches_.clear();

    double max_dist = 0;
    double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for (size_t i = 0; i < prev_descriptors_.rows; i++)
    {
        double dist = matches_[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    for (size_t i = 0; i < matches_.size(); i++)
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

bool WideBaselineTracker::track(const cv::Mat& img)
{
    //Make a grayscale copy of the current frame if it is in color
    if(img.channels() > 1)
    {
        cvtColor(img, curr_frame_gray_, CV_BGR2GRAY);
    }
    else
    {
        img.copyTo(curr_frame_gray_);
    }

    logger.print(pcl::console::L_DEBUG, "[WideBaselineTracker::track] DEBUG: tracking frame%i\n",frame_idx_);
   
    //Update the internal buffers
    update_buffers();

    //Detect KeyPoints and extract descriptors on the current frame
    detect_keypoints();

    if (!initialized_)
    {
        initialized_ = true;;
        add_keypoints();
    }
    else
    {
        
        matcher_->match(curr_descriptors_, prev_descriptors_, matches_);
        logger.print(pcl::console::L_DEBUG, "[WideBaselineTracker::track] DEBUG: matching descriptos...\n");
        
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
                tracked_pts++;
            }
        }
        
        logger.print(pcl::console::L_DEBUG, "[WideBaselineTracker::track] DEBUG: tracked points: %i\n",tracked_pts);
        
        getGoodMatches(0.1);   
        
    }

    keypoints_with_matches.clear();
    cv::swap(curr_frame_gray_, prev_frame_gray_);
    frame_idx_++;

    return true;
}

void WideBaselineTracker::clear(){
    tracklets_.clear();
    curr_KPs_.clear();
    prev_KPs_.clear();
    matches_.clear();
    //initialized_ = false;
}

