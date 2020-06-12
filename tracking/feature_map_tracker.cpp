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

    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::add_keypoints DEBUG: adding keypoints...\n");
    logger.print(EventLogger::L_DEBUG, "[FeatureMapTracker::add_keypoints DEBUG: added pts.: %lu\n", map_tracklets_.size());
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
    std::swap(curr_kpts_, map_kpts_);
    std::swap(curr_descriptors_, map_descriptors_);
}

int FeatureMapTracker::searchMatches(const int &keypoint_index)
{
    int train_index;

    for (int query_index = 0; query_index < matches_.rows; query_index++)
    {
        train_index = matches_.at<int>(query_index, 0);
        if (keypoint_index == train_index)
        {
            return query_index;
        }
    }
    return -1;
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
}

FeatureMapTracker::FeatureMapTracker(const std::string &feature_detector, const std::string &descriptor_extractor, const bool &log_stats) : FeatureTracker()
{
    setFeatureDetector(feature_detector);
    setDescriptorExtractor(descriptor_extractor);

    if (log_stats)
    {
        initialize_logger("timing_stats.txt", "tracking_stats.txt", "heatmap_stats.txt");
    }
}

FeatureMapTracker::FeatureMapTracker(const std::string &feature_detector, const std::string &descriptor_extractor, const int &min_pts, const int &max_pts, const bool &log_stats) : FeatureTracker(min_pts, max_pts, log_stats)
{
    setFeatureDetector(feature_detector);
    setDescriptorExtractor(descriptor_extractor);
}

bool FeatureMapTracker::track(const cv::Mat &img)
{
    bool is_keyframe = false;

    //Make a grayscale copy of the current frame if it is in color
    if (img.channels() > 1)
    {
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
        update_buffers();
        add_keypoints();
    }
    else
    {
        //Descriptor matcher: FLANN Index Object
        index_ = new cv::flann::Index(map_descriptors_, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);

        //Searches similar descriptors in the current frame for each descriptor in the map
        index_->knnSearch(curr_descriptors_, matches_, dists_, 1, cv::flann::SearchParams());

        delete index_;
    }

    return is_keyframe;
}

void FeatureMapTracker::clear()
{
    tracklets_.clear();
    curr_kpts_.clear();
    prev_kpts_.clear();
    map_kpts_.clear();
    curr_pts_.clear();
    prev_pts_.clear();
    initialized_ = false;
}
