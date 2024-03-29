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
 *  Authors:
 * 
 *  Marcos Henrique F. Marcone
 *  Luiz Felipe Maciel Correia
 *  Bruno Silva
 */

#include <iostream>

#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <boost/algorithm/string.hpp>
#include <cmath>

#include "wide_baseline_tracker.h"
#include <event_logger.h>

using namespace cv;

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

void WideBaselineTracker::detect_keypoints(const cv::Mat& mask)
{
    curr_kpts_.clear();
    feature_detector_->detect(curr_frame_gray_, curr_kpts_, mask);
    descriptor_extractor_->compute(curr_frame_gray_, curr_kpts_, curr_descriptors_);
    
    MLOG_DEBUG(EventLogger::M_TRACKING, "@WideBaselineTracker::detect_keypoints: detecting keypoints...\n");
    MLOG_DEBUG(EventLogger::M_TRACKING, "@WideBaselineTracker::detect_keypoints: detected pts.: %lu\n",
               added_pts_.size());
}

void WideBaselineTracker::add_keypoints()
{
    for (size_t i = 0; i < curr_kpts_.size(); i++)
    {

        // Create and add tracklet
        Tracklet tr(frame_idx_);
        tr.pts2D_.push_back(curr_kpts_[i].pt);
        tr.keypoint_indices_.push_back(i);
        tracklets_.push_back(tr);
    }

    MLOG_DEBUG(EventLogger::M_TRACKING, "@WideBaselineTracker::add_keypoints: added pts.: %lu\n",
               added_pts_.size());
    MLOG_DEBUG(EventLogger::M_TRACKING, "@WideBaselineTracker::add_keypoints: prev pts.: %lu\n",
               prev_pts_.size());
}

void WideBaselineTracker::update_buffers()
{
    std::swap(curr_kpts_, prev_kpts_);
    std::swap(curr_descriptors_, prev_descriptors_);
}

int WideBaselineTracker::searchMatches(const int &keypoint_index)
{
    for (size_t i = 0; i < matches_.size(); i++)
    {
        if (keypoint_index == matches_[i].trainIdx) return matches_[i].queryIdx;
    }
    return -1;
}

void WideBaselineTracker::setCurrentPoints()
{
    curr_pts_.clear();

    for (size_t i = 0; i < matches_.size(); i++)
    {
        curr_pts_.push_back(curr_kpts_[matches_[i].queryIdx].pt);
    }
}

void WideBaselineTracker::setPreviousPoints()
{
    prev_pts_.clear();

    for (size_t i = 0; i < matches_.size(); i++)
    {
        prev_pts_.push_back(prev_kpts_[matches_[i].trainIdx].pt);
    }
}

void WideBaselineTracker::setFeatureDetector(const std::string &feature_detector)
{
    std::string upper_feature_detector = feature_detector;
    boost::to_upper(upper_feature_detector);

    if (upper_feature_detector == "ORB")
        feature_detector_ = cv::ORB::create();
    else if (upper_feature_detector == "AKAZE")
        feature_detector_ = cv::AKAZE::create();
    else if (upper_feature_detector == "GFTT")
        feature_detector_ = cv::GFTTDetector::create();
    else if (upper_feature_detector == "FAST")
        feature_detector_ = cv::FastFeatureDetector::create();
    else if (upper_feature_detector == "AGAST")
        feature_detector_ = cv::AgastFeatureDetector::create();
    else if (upper_feature_detector == "BRISK")
        feature_detector_ = cv::BRISK::create();
    else if (upper_feature_detector == "SURF")
        feature_detector_ = cv::xfeatures2d::SURF::create();
    else if (upper_feature_detector == "SIFT")
    {
        #if CV_MAJOR_VERSION < 4
            feature_detector_ = cv::xfeatures2d::SIFT::create();
        #else
            feature_detector_ = cv::SIFT::create();  
        #endif
    }
    else
    {
        MLOG_ERROR(EventLogger::M_TRACKING, "@WideBaselineTracker::setFeatureDetector:  \
                                             attribute %s couldn't be loaded, insert a valid \
                                             feature detector.\n",
                                             upper_feature_detector.c_str());

        throw std::invalid_argument("Insert a valid feature detector.");
    }

    MLOG_DEBUG(EventLogger::M_TRACKING, "@WideBaselineTracker::setFeatureDetector: \
                                         feature_detector_ = %s\n",
                                         upper_feature_detector.c_str());
}

void WideBaselineTracker::setDescriptorExtractor(const std::string &descriptor_extractor)
{
    std::string upper_descriptor_extractor = descriptor_extractor;
    boost::to_upper(upper_descriptor_extractor);

    if (upper_descriptor_extractor == "ORB")
        descriptor_extractor_ = cv::ORB::create();
    else if (upper_descriptor_extractor == "AKAZE")
        descriptor_extractor_ = cv::AKAZE::create();
    else if (upper_descriptor_extractor == "BRISK")
        descriptor_extractor_ = cv::BRISK::create();
    else if (upper_descriptor_extractor == "SURF")
        descriptor_extractor_ = cv::xfeatures2d::SURF::create();
    else if (upper_descriptor_extractor == "SIFT")
    {
        #if CV_MAJOR_VERSION < 4
            descriptor_extractor_ = cv::xfeatures2d::SIFT::create();
        #else
            descriptor_extractor_ = cv::SIFT::create();  
        #endif
    }
    else
    {
        MLOG_ERROR(EventLogger::M_TRACKING, "@WideBaselineTracker::setDescriptorExtractor:  \
                                             attribute %s couldn't be loaded, insert a valid \
                                             descriptor extractor.\n",
                                             upper_descriptor_extractor.c_str());

        throw std::invalid_argument("Insert a valid descriptor extractor.");
    }

    MLOG_DEBUG(EventLogger::M_TRACKING, "@WideBaselineTracker::setDescriptorExtractor: \
                                         descriptor_extractor_ = %s\n",
                                         upper_descriptor_extractor.c_str());
}

void WideBaselineTracker::setMatcher(const std::string &matcher)
{
    std::string upper_matcher = matcher;
    boost::to_upper(upper_matcher);

    if (upper_matcher == "BRUTEFORCE")
        matcher_ = cv::DescriptorMatcher::create("BruteForce");
    else if (upper_matcher == "FLANNBASED")
        matcher_ = cv::DescriptorMatcher::create("FlannBased");
    else
    {
        MLOG_ERROR(EventLogger::M_TRACKING, "@WideBaselineTracker::setMatcher:  \
                                             attribute %s couldn't be loaded, insert a valid \
                                             matcher.\n",
                                             upper_matcher.c_str());

        throw std::invalid_argument("Insert a valid matcher.");
    }

    MLOG_DEBUG(EventLogger::M_TRACKING, "@WideBaselineTracker::setMatcher: \
                                         descriptor_extractor__ = %s\n",
                                         upper_matcher.c_str());
}

bool WideBaselineTracker::checkCombination(const std::string& feature_detector, 
                                         const std::string& descriptor_extractor)
{
    
    std::string upper_feature_detector = feature_detector;
    boost::to_upper(upper_feature_detector);
    
    std::string upper_descriptor_extractor = descriptor_extractor;
    boost::to_upper(upper_descriptor_extractor);

    bool valid_combination = true;

    if (upper_descriptor_extractor == "AKAZE" && upper_feature_detector != "AKAZE")
    {
        valid_combination = false;
    }
    else if (upper_feature_detector == "SIFT" && upper_descriptor_extractor == "ORB")
    {
        valid_combination = false;
    }

    if (!valid_combination)
    {
        MLOG_ERROR(EventLogger::M_TRACKING, "@FeatureMapTracker::checkCombination: \
                                             the combination FD: %s and DE: %s is not valid.\n",
                   upper_feature_detector.c_str(), 
                   upper_descriptor_extractor.c_str());

        throw std::invalid_argument(
              "Insert a valid combination of feature detector and descriptor extractor.");
    }

    return valid_combination; 
}

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

WideBaselineTracker::WideBaselineTracker() : FeatureTracker()
{
    setFeatureDetector("ORB");
    setDescriptorExtractor("ORB");
    setMatcher("BRUTEFORCE");
    keyframe_threshold_ = 0.3;
}

WideBaselineTracker::WideBaselineTracker(
    const std::string &feature_detector,
    const std::string &descriptor_extractor,
    const std::string &matcher,
    const float &keyframe_threshold, 
    const bool &log_stats)
    : FeatureTracker()
{
    if (checkCombination(feature_detector, descriptor_extractor))
    {
        setFeatureDetector(feature_detector);
        setDescriptorExtractor(descriptor_extractor);
        setMatcher(matcher);
        keyframe_threshold_ = keyframe_threshold;
    }
    

    if (log_stats) initialize_logger("timing_stats.txt", "tracking_stats.txt", "heatmap_stats.txt");
}

WideBaselineTracker::WideBaselineTracker(
    const std::string &feature_detector,
    const std::string &descriptor_extractor,
    const std::string &matcher, 
    const int &min_pts,
    const int &max_pts,
    const float &keyframe_threshold, 
    const bool &log_stats)
    : FeatureTracker(min_pts, max_pts, log_stats)
{
    if (checkCombination(feature_detector, descriptor_extractor))
    {
        setFeatureDetector(feature_detector);
        setDescriptorExtractor(descriptor_extractor);
        setMatcher(matcher);
        keyframe_threshold_ = keyframe_threshold;
    }
    
}

WideBaselineTracker::WideBaselineTracker(const FeatureTracker::Parameters& param):
    FeatureTracker(param.min_pts_, param.max_pts_, param.log_stats_)
{
    if (checkCombination(param.feature_detector_, param.descriptor_extractor_))
    {
        setFeatureDetector(param.feature_detector_);
        setDescriptorExtractor(param.descriptor_extractor_);
        setMatcher(param.matcher_);
    }
}

void WideBaselineTracker::getGoodMatches(const double &threshold)
{
    good_matches_.clear();

    double max_dist = 0;
    double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < prev_descriptors_.rows; i++)
    {
        double dist = matches_[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    for (size_t i = 0; i < matches_.size(); i++)
    {
        if (matches_[i].distance <= max(2 * min_dist, threshold))
            good_matches_.push_back(matches_[i]);
    }

    prev_good_pts_.clear();
    curr_good_pts_.clear();

    for (size_t i = 0; i < good_matches_.size(); i++)
    {
        prev_good_pts_.push_back(prev_kpts_[good_matches_[i].trainIdx].pt);
        curr_good_pts_.push_back(curr_kpts_[good_matches_[i].queryIdx].pt);
    }
}

bool WideBaselineTracker::track(const cv::Mat &img, const cv::Mat& mask)
{
    bool is_keyframe = false;

    // Make a grayscale copy of the current frame if it is in color
    if (img.channels() > 1)
    {
        #if CV_MAJOR_VERSION < 4
            cvtColor(img, curr_frame_gray_, CV_BGR2GRAY);
        #else
            cvtColor(img, curr_frame_gray_, COLOR_BGR2GRAY);
        #endif
    }
    else
        img.copyTo(curr_frame_gray_);

    MLOG_DEBUG(EventLogger::M_TRACKING, "@WideBaselineTracker::track: tracking frame %i\n",
               frame_idx_);

    // Update the internal buffers
    update_buffers();

    // Detect KeyPoints and extract descriptors on the current frame
    detect_keypoints(mask);

    if (!initialized_)
    {
        initialized_ = true;
        add_keypoints();
    }
    else
    {
        matcher_->match(curr_descriptors_, prev_descriptors_, matches_);

        setPreviousPoints();
        setCurrentPoints();

        keypoints_with_matches.resize(curr_kpts_.size());
        int tracked_pts = 0;

        // Updating the tracklets
        for (size_t i = 0; i < tracklets_.size(); i++)
        {

            int keypoint_index = tracklets_[i].keypoint_indices_.back();
            int search_result = searchMatches(keypoint_index);

            if (search_result != -1)
            {
                tracklets_[i].pts2D_.push_back(curr_kpts_[search_result].pt);
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
        for (size_t i = 0; i < curr_kpts_.size(); i++)
        {
            if (keypoints_with_matches[i] == false)
            {
                // Create and add tracklet
                Tracklet tr(frame_idx_);
                tr.pts2D_.push_back(curr_kpts_[i].pt);
                tr.keypoint_indices_.push_back(i);
                tracklets_.push_back(tr);
                tracked_pts++;
            }
        }

        MLOG_DEBUG(EventLogger::M_TRACKING, "@WideBaselineTracker::track: \
                                             tracked points:  %i\n",
                                             tracked_pts);

        //getGoodMatches(0.1);

        // Percentage of the variation of features of the current instant 
        // in relation to the previous instant
        if (abs(curr_kpts_.size()-prev_kpts_.size())/(float)prev_kpts_.size() > keyframe_threshold_)
        {
            // Make the current frame a new keyframe
            is_keyframe = true;
        }
    }

    keypoints_with_matches.clear();
    cv::swap(curr_frame_gray_, prev_frame_gray_);
    frame_idx_++;

    return is_keyframe;
}

void WideBaselineTracker::clear()
{
    tracklets_.clear();
    curr_kpts_.clear();
    prev_kpts_.clear();
    matches_.clear();
    curr_pts_.clear();
    prev_pts_.clear();
    initialized_ = false;
}
