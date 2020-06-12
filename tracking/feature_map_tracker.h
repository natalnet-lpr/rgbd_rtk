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

#ifndef INCLUDE_FEATURE_MAP_TRACKER_H_
#define INCLUDE_FEATURE_MAP_TRACKER_H_

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <vector>

#include <feature_tracker.h>
#include <common_types.h>

class FeatureMapTracker : public FeatureTracker
{

protected:
    //Feature Detector object
    cv::Ptr<cv::FeatureDetector> feature_detector_;

    //Descriptor Extractor object
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;

    //FLANN Index object pointer
    cv::flann::Index* index_;

    //Boolean vector that indicates if the keypoint i have a match
    std::vector<bool> keypoints_with_matches;

    //Detect KeyPoints and extract descriptors using this function
    void detect_keypoints();
    
    //Adds keypoints detected in the first frame to the tracker
    void add_keypoints();

    //Update internal buffers (updates the previous points (in the current frame) with the current points (from the previous frame))
    void update_buffers();

    //Searches if a keypoint have a match
    int searchMatches(const int& keypoint_index);

    //Set the coordinates of the current keypoints to curr_pts_.
    //void setCurrentPoints();

    //Set the coordinates of the previous keypoints to prev_pts_.
    //void setPreviousPoints();

    //Set the coordinates of the map keypoints to map_pts_.
    //void setMapPoints();

    //Set the feature_detector_ attribute from a string
    void setFeatureDetector(const std::string& feature_detector);

    //Set the descriptor_extractor_ attribute from a string
    void setDescriptorExtractor(const std::string& descriptor_extractor);

public:

     //Store all the current keypoints founded in the current frame
    std::vector<cv::KeyPoint> curr_kpts_;

    //Store all the current descriptors founded in the current frame
    cv::Mat curr_descriptors_;

    //Store all the previous keypoints founded in the previous frame
    std::vector<cv::KeyPoint> prev_kpts_;

    //Store all the previous descriptors founded in the previous frame
    cv::Mat prev_descriptors_;

    //Feature Map Positions
    std::vector<cv::Point2f> map_pts_;

    //Store the keypoints that set the feature map.
    std::vector<cv::KeyPoint> map_kpts_; 

    //Store the descriptors that set the feature map.
    cv::Mat map_descriptors_;

    //Map Tracklets: history of each point of the feature map as a vector of point2f
    std::vector<Tracklet> map_tracklets_;

    //Store all the matches beetween features
    cv::Mat matches_;

    //Store the distances beetween the matches 
    cv::Mat dists_;

    //Default constructor: create a ORB feature detector and descriptor extractor. 
    FeatureMapTracker();

    //Constructor with flexible feature detector and descriptor extractor and flag to log statistics
    FeatureMapTracker(const std::string& feature_detector, const std::string& descriptor_extractor, const bool& log_stats);
    
    //Constructor with the minimum number of tracked points, maximum number of tracked points and flag to log statistics
    FeatureMapTracker(const std::string& feature_detector, const std::string& descriptor_extractor, const int& min_pts, const int& max_pts, const bool& log_stats);

    //Tracks keypoints between the current frame and the previous.
    bool track(const cv::Mat& img);

    //Clear all data about tracked points
    void clear();
};

#endif /* INCLUDE_FEATURE_MAP_TRACKER_H_ */
