/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author:
 *
 *  Bruno Silva
 *  Marcos Henrique F. Marcone 
 */

#include <iostream>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/flann/miniflann.hpp>

#include <boost/algorithm/string.hpp>

#include "feature_map_tracker.h"
#include <event_logger.h>

using namespace cv;

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

void FeatureMapTracker::add_keypoints()
{
    for (size_t i = 0; i < curr_kpts_.size(); i++)
    {
        //Create and add tracklet
        Tracklet tr(frame_idx_);
        tr.pts2D_.push_back(curr_kpts_[i].pt);
        tr.keypoint_indices_.push_back(i);
        map_tracklets_.push_back(tr);
    }

    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::add_keypoints DEBUG]: adding keypoints...\n");
    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::add_keypoints DEBUG]: added pts.: %lu\n", map_tracklets_.size());
}

void FeatureMapTracker::detect_keypoints()
{
    curr_kpts_.clear();
    feature_detector_->detect(curr_frame_gray_, curr_kpts_);
    descriptor_extractor_->compute(curr_frame_gray_, curr_kpts_, curr_descriptors_);
    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::detect_keypoints] DEBUG: detecting keyponts...\n");
    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::detect_keypoints] DEBUG: detected pts.: %lu\n", curr_kpts_.size());
}

void FeatureMapTracker::update_buffers()
{
    //std::swap(curr_kpts_, map_kpts_);
    //std::swap(curr_descriptors_, map_descriptors_);
    std::copy(curr_kpts_.begin(), curr_kpts_.end(), std::back_inserter(map_kpts_));
    curr_descriptors_.copyTo(map_descriptors_);
    life_of_keypoints_on_the_map_.resize(map_kpts_.size());
    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::detect_keypoints] DEBUG: update buffers...\n");

}

void FeatureMapTracker::getGoodMatches(const float& dist_ratio)
{

    good_matches_.clear();
    good_matches_.resize(matches_.rows);

    int number_of_good_matches = 0;

    for (int i = 0; i < matches_.rows; i++){
        //int idx1 = i; // matches has the same number of query points (current points)
        //int idx2 = matches_.at<int>(i,0); // Index of the first match

        // The distances to the first and the second matches
        float dist_first = dists_.at<float>(i,0), dist_second = dists_.at<float>(i,1);

        // Selecting the good matches
        if (dist_first < dist_ratio*dist_second)
        {
            good_matches_[i] = true;
            number_of_good_matches++;
        }
        else
        {
            good_matches_[i] = false;
        }
    }
    logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: number of good matches: %i\n", number_of_good_matches);
}

void FeatureMapTracker::updateTheMap()
{
    curr_kpts_with_matches_.clear();
    curr_kpts_with_matches_.resize(curr_kpts_.size());
    tracked_pts_ = 0;

    // Searching for the matches
    for (size_t i = 0; i < map_kpts_.size();i++){

        int train_index = i;
        int query_index = searchMatches(i);

        // Successful match
        if (query_index != -1)
        {
            // Update the map
            map_kpts_[train_index] = curr_kpts_[query_index];
            curr_descriptors_.row(query_index).copyTo(map_descriptors_.row(train_index));
            curr_kpts_with_matches_[query_index] = true;
            life_of_keypoints_on_the_map_[train_index] = 0;

            // Update the tracklets
            map_tracklets_[train_index].pts2D_.push_back(curr_kpts_[query_index].pt);
            map_tracklets_[train_index].keypoint_indices_.push_back(query_index);
            tracked_pts_++;

        }
        else
        {
            life_of_keypoints_on_the_map_[train_index]++;
        }
        
    }

    logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::updateTheMap] DEBUG: number of keypoints replaced on the map: %i\n", tracked_pts_);
}

void FeatureMapTracker::removeInvalidKeypointsFromTheMap()
{
    int ok_keypoints = 0;

    //Verify if a keypoint exceed the max number of frames without matches
    for (size_t i = 0; i < life_of_keypoints_on_the_map_.size(); i++)
    {
        if(life_of_keypoints_on_the_map_[i] > lifespan_of_a_feature_)
        {
            continue;
        }
        else
        {
            map_kpts_[ok_keypoints] = map_kpts_[i];
            map_descriptors_.row(i).copyTo(map_descriptors_.row(ok_keypoints));
            life_of_keypoints_on_the_map_[ok_keypoints] = life_of_keypoints_on_the_map_[i];
            map_tracklets_[ok_keypoints] = map_tracklets_[i];
            ok_keypoints++;

        }
    }

    logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::removeInvalidKeypointsFromTheMap] DEBUG: number of keypoints removed from the map: %i\n", (map_kpts_.size()-ok_keypoints));
    map_kpts_.resize(ok_keypoints);
    map_descriptors_.resize(ok_keypoints);
    life_of_keypoints_on_the_map_.resize(ok_keypoints);
    map_tracklets_.resize(ok_keypoints);
}

void FeatureMapTracker::addNewKeypointsToTheMap()
{
    int added_kpts = 0;
    // Include at the end of the map the features of the current frame without matches 
    for (size_t i = 0; i < curr_kpts_.size(); i++)
    {
        if (curr_kpts_with_matches_[i] == false)
        {
            map_kpts_.push_back(curr_kpts_[i]);
            map_descriptors_.push_back(curr_descriptors_.row(i));
            life_of_keypoints_on_the_map_.push_back(0);

            //Create and add tracklet
            Tracklet tr(frame_idx_);
            tr.pts2D_.push_back(curr_kpts_[i].pt);
            tr.keypoint_indices_.push_back(i);
            map_tracklets_.push_back(tr);
            tracked_pts_++;
            added_kpts++;
        }
    }

    logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::addNewKeypointsToTheMap] DEBUG: number of new keypoints added to the map: %i\n", added_kpts);
}

int FeatureMapTracker::searchMatches(const int &keypoint_index)
{
    int train_index;

    for (int query_index = 0; query_index < matches_.rows; query_index++)
    {
        train_index = matches_.at<int>(query_index, 0);
        if (keypoint_index == train_index && good_matches_[query_index])
        {
            return query_index;
        }
    }
    return -1;
}

void FeatureMapTracker::setCurrentPoints()
{
    for (int i = 0; i < matches_.rows; i++)
    {
        if (good_matches_[i])
        {
            curr_pts_.push_back(curr_kpts_[i].pt);
        }
    }
}

// Trocar o nome do metodo
// Nova versão
void FeatureMapTracker::setMapPoints()
{
    for (int i = 0; i < matches_.rows; i++)
    {
        if (good_matches_[i])
        {
            int map_index = matches_.at<int>(i,0); 
            map_pts_.push_back(map_kpts_[map_index].pt);
        }
    }
}

void FeatureMapTracker::setFeatureDetector(const std::string &feature_detector)
{
    std::string upper_feature_detector = feature_detector;
    boost::to_upper(upper_feature_detector);

    if (upper_feature_detector == "ORB")
    {
        feature_detector_ = cv::ORB::create();
    }
    else if (upper_feature_detector == "AKAZE")
    {
        feature_detector_ = cv::AKAZE::create();
    }
    else if (upper_feature_detector == "GFTT")
    {
        feature_detector_ = cv::GFTTDetector::create();
    }
    else if (upper_feature_detector == "FAST")
    {
        feature_detector_ = cv::FastFeatureDetector::create();
    }
    else if (upper_feature_detector == "AGAST")
    {
        feature_detector_ = cv::AgastFeatureDetector::create();
    }
    else if (upper_feature_detector == "BRISK")
    {
        feature_detector_ = cv::BRISK::create();
    }
    else if (upper_feature_detector == "SURF")
    {
        feature_detector_ = cv::xfeatures2d::SURF::create();
    }
    else if (upper_feature_detector == "SIFT")
    {
        feature_detector_ = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        logger.print(EventLogger::L_ERROR, "[FeatureMapTracker::setFeatureDetector] ERROR: Attribute %s couldn't be loaded, insert a valid feature detector\n", upper_feature_detector.c_str());
        throw std::invalid_argument("Insert a valid feature detector");
    }

    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::setFeatureDetector] DEBUG: Attribute feature_detector_ = %s\n", upper_feature_detector.c_str());
}

void FeatureMapTracker::setDescriptorExtractor(const std::string &descriptor_extractor)
{
    std::string upper_descriptor_extractor = descriptor_extractor;
    boost::to_upper(upper_descriptor_extractor);

    if (upper_descriptor_extractor == "ORB")
    {
        descriptor_extractor_ = cv::ORB::create();
    }
    else if (upper_descriptor_extractor == "AKAZE")
    {
        descriptor_extractor_ = cv::AKAZE::create();
    }
    else if (upper_descriptor_extractor == "BRISK")
    {
        descriptor_extractor_ = cv::BRISK::create();
    }
    else if (upper_descriptor_extractor == "SURF")
    {
        descriptor_extractor_ = cv::xfeatures2d::SURF::create();
    }
    else if (upper_descriptor_extractor == "SIFT")
    {
        descriptor_extractor_ = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        logger.print(EventLogger::L_ERROR, "[FeatureMapTracker::setDescriptorExtractor] ERROR: Attribute %s couldn't be loaded, insert a valid descriptor extractor\n", upper_descriptor_extractor.c_str());
        throw std::invalid_argument("Insert a valid descriptor extractor");
    }

    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::setDescriptorExtractor] DEBUG: Attribute descriptor_extractor_ = %s\n", upper_descriptor_extractor.c_str());
}

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

FeatureMapTracker::FeatureMapTracker() : FeatureTracker()
{
    setFeatureDetector("ORB");
    setDescriptorExtractor("ORB");

    lifespan_of_a_feature_ = 3;
}

FeatureMapTracker::FeatureMapTracker(const std::string &feature_detector, const std::string &descriptor_extractor, const int& lifespan_of_a_feature, const bool &log_stats) : FeatureTracker()
{
    setFeatureDetector(feature_detector);
    setDescriptorExtractor(descriptor_extractor);

    lifespan_of_a_feature_ = lifespan_of_a_feature;

    if (log_stats)
    {
        initialize_logger("timing_stats.txt", "tracking_stats.txt", "heatmap_stats.txt");
    }
}

FeatureMapTracker::FeatureMapTracker(const std::string &feature_detector, const std::string &descriptor_extractor, const int& lifespan_of_a_feature, const int &min_pts, const int &max_pts, const bool &log_stats) : FeatureTracker(min_pts, max_pts, log_stats)
{
    setFeatureDetector(feature_detector);
    setDescriptorExtractor(descriptor_extractor);

    lifespan_of_a_feature_ = lifespan_of_a_feature;
}

bool FeatureMapTracker::track(const cv::Mat &img)
{
    bool is_keyframe = false;

    //Make a grayscale copy of the current frame if it is in color
    if (img.channels() > 1)
    {
        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: converting to a grayscale image...\n");
        cvtColor(img, curr_frame_gray_, CV_BGR2GRAY);
    }
    else
    {
        img.copyTo(curr_frame_gray_);
    }

    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::track] DEBUG: tracking frame%i\n", frame_idx_);

    //Detect KeyPoints and extract descriptors on the current frame
    detect_keypoints();

    if (!initialized_)
    {
        initialized_ = true;
        add_keypoints();
        update_buffers();
         //Descriptor matcher: FLANN Index Object  
        index_ = new cv::flann::Index();
    }
    else
    {
        // Convert the map_descriptors_ and the curr_descriptors_ to CV_32F       
        map_descriptors_.convertTo(map_descriptors_, CV_32F);   
        curr_descriptors_.convertTo(curr_descriptors_,CV_32F);
        
        
        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: inicial map size: %i\n", map_descriptors_.rows);

        //cv::flann::Index index(map_descriptors_, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: building the Index object...\n");

        double el_time = (double) cvGetTickCount();
        index_->build(map_descriptors_, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
        el_time = ((double) cvGetTickCount()- el_time)/(cvGetTickFrequency()*1000.0);
        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: build index time: %f ms\n", el_time);

        
        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: executing the knnSearch...\n");
        //Searches similar descriptors in the current frame for each descriptor in the map
        // The matches_ object is a Nx2 Matrix, where N is the size of the curr_descriptors_ object  
        index_->knnSearch(curr_descriptors_, matches_, dists_, 2, cv::flann::SearchParams());
        

        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: calculating the good matches...\n");
        getGoodMatches(DIST_RATIO_);

        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: defining the current and the previous points...\n");
        setCurrentPoints();
        setMapPoints();

        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: updating the map...\n");
        updateTheMap();

        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: removing invalid keypoints from the map...\n");
        removeInvalidKeypointsFromTheMap();
       
        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: adding new keypoints to the map...\n");
        addNewKeypointsToTheMap();

        logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::track] DEBUG: tracked points: %i\n", tracked_pts_);
        logger.print(EventLogger::L_DEBUG, "[FeatuerMapTracker::track] DEBUG: final map size: %i\n", map_kpts_.size());

        ;
    }

    frame_idx_++;

    return is_keyframe;
}

void FeatureMapTracker::clear()
{
    map_tracklets_.clear();
    curr_kpts_.clear();
    map_kpts_.clear();
    curr_pts_.clear();
    map_pts_.clear();
    initialized_ = false;
}

FeatureMapTracker::~FeatureMapTracker(){
    if (index_ != nullptr){
        delete index_;
    }
}
