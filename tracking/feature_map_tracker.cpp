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

void FeatureMapTracker::add_keypoints()
{
    std::cout << "To do";
}

void FeatureMapTracker::detect_keypoints()
{
    std::cout << "To do";
}

void FeatureMapTracker::update_buffers()
{
    std::cout << "To do";
}

void FeatureMapTracker::setFeatureDetector(const std::string& feature_detector)
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
    else if (upper_feature_detector== "SIFT")
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

void FeatureMapTracker::setDescriptorExtractor(const std::string& descriptor_extractor)
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




FeatureMapTracker::FeatureMapTracker():
    FeatureTracker()
{
    setFeatureDetector("ORB");
    setDescriptorExtractor("ORB");
}

FeatureMapTracker::FeatureMapTracker(const std::string& feature_detector, const std::string& descriptor_extractor, const bool& log_stats):
    FeatureTracker()
{
    setFeatureDetector(feature_detector);
    setDescriptorExtractor(descriptor_extractor);

    if (log_stats)
    {
        initialize_logger("timing_stats.txt", "tracking_stats.txt", "heatmap_stats.txt");
    }
}

FeatureMapTracker::FeatureMapTracker(const std::string& feature_detector, const std::string& descriptor_extractor, const int& min_pts, const int& max_pts, const bool& log_stats):
    FeatureTracker(min_pts, max_pts, log_stats)
{
    setFeatureDetector(feature_detector);
    setDescriptorExtractor(descriptor_extractor);
}

bool FeatureMapTracker::track(const cv::Mat& img)
{
    std::cout << img.rows << std::endl;
    std::cout << "To do";
    return true;
}

void FeatureMapTracker::clear()
{
    tracklets_.clear();
    curr_kpts_.clear();
    prev_kpts_.clear();
    kpts_map_.clear();
    curr_pts_.clear();
    prev_pts_.clear();
    initialized_ = false;
}
