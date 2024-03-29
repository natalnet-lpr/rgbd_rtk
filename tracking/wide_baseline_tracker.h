/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or
 *     promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author:
 *
 *  Bruno Silva
 *  Luiz Felipe Maciel Correia
 *  Marcos Henrique F. Marcone
 */

#ifndef INCLUDE_WIDE_BASELINE_TRACKER_H_
#define INCLUDE_WIDE_BASELINE_TRACKER_H_

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/core/core.hpp>
#include <vector>

#include <common_types.h>
#include <feature_tracker.h>

class WideBaselineTracker : public FeatureTracker
{

protected:
    //Feature Detector object
    cv::Ptr<cv::FeatureDetector> feature_detector_;

    //Descriptor Extractor object
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;

    // Matcher who store all the descriptors of the scene
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    //Boolean vector that indicates if the keypoint i have a match
    std::vector<bool> keypoints_with_matches;

    /**
     * Detect KeyPoints and extract descriptors using this function
     */
    void detect_keypoints(const cv::Mat& mask);

    /**
     * Adds keypoints detected in the first frame to the tracker
     */
    void add_keypoints();

    /**
     * Update internal buffers (updates the previous points (in the current frame)
     * with the current points (from the previous frame))
     */
    void update_buffers();

    /**
     * Searches if a keypoint have a match
     * @param keypoint_index id of keypoint
     * @return match id
     */
    int searchMatches(const int &keypoint_index);

    /**
     * Set the coordinates of the current keypoints to curr_pts_.
     */
    void setCurrentPoints();

    /**
     * Set the coordinates of the previous keypoints to prev_pts_.
     */
    void setPreviousPoints();

    /**
     * Set the feature_detector_ attribute from a string
     * @param feature_detector name of feature_detector, the list of options are:
     * ORB, AKAZE, GFTT, FAST, AGAST, BRISK, SURF, SIFT
     */
    void setFeatureDetector(const std::string &feature_detector);

    /**
     * Set the descriptor_extractor_ attribute from a string
     * @param descriptor_extractor name of descriptor_extractor, the list of options are:
     * ORB, AKAZE, BRISK, SURF, SIFT 
     */
    void setDescriptorExtractor(const std::string &descriptor_extractor);

    /**
     * Set the matcher_ attribute from a string
     * @param matcher name of matcher, the list of options are:
     * BRUTEFORCE AND FLANNBASED
     */
    void setMatcher(const std::string &matcher);

    /**
     * Checks if a valid combination of feature detector and descriptor extractor has been made
     * @param feature_detector name of feature_detector 
     * @param descriptor_extractor name of descriptor_extractor
     * @return boolean
    */
    bool checkCombination(const std::string& feature_detector, 
                          const std::string& descriptor_extractor);
                          
public:
    // Store all the current keypoints founded in the current frame
    std::vector<cv::KeyPoint> curr_kpts_;

    // Store all the current descriptors founded in the current frame
    cv::Mat curr_descriptors_;

    // Store all the previous keypoints founded in the previous frame
    std::vector<cv::KeyPoint> prev_kpts_;

    // Store all the previous descriptors founded in the previous frame
    cv::Mat prev_descriptors_;

    // Store all the "good" current keypoints that's points who has more
    // accurracy than others calculated through euclidian distance
    std::vector<cv::Point2f> curr_good_pts_;

    // Store all the "good" previous keypoints that's points who has more
    // accurracy than others calculated through euclidian distance
    std::vector<cv::Point2f> prev_good_pts_;

    // Store all the matches beetween features
    std::vector<cv::DMatch> matches_;

    // Store only the "good" matches, in other words, store only the matchings
    // of "good" features
    std::vector<cv::DMatch> good_matches_;

    // Threshold that defines whether a frame is a keyframe or not
    float keyframe_threshold_;

    /**
     * Default constructor: create a ORB detector and extractor and a BruteForce matcher.
     */
    WideBaselineTracker();

    /**
     * Constructor with flexible feature detector, descriptor extractor and matcher, 
     * keyframe threshold and flag to log statistics
     * The following combinations of (Feature Detector, Descriptor Extractor) are invalid:
     * (ORB, AKAZE); (GFTT, AKAZE); (FAST, AKAZE); (AGAST, AKAZE); (BRISK, AKAZE); (SURF, AKAZE); 
     * (SIFT, AKAZE); (SIFT, ORB);
     * @param feature_detector ORB, AKAZE, GFTT, FAST, AGAST, BRISK, SURF, SIFT
     * @param descriptor_extractor ORB, AKAZE, BRISK, SURF, SIFT
     * @param matcher BRUTEFORCE AND FLANNBASED
     * @param keyframe_threshold threshold that defines whether a frame is a keyframe or not
     * @param log_stats if log statistics should be displayed
     */
    WideBaselineTracker(const std::string &feature_detector,
                        const std::string &descriptor_extractor, 
                        const std::string &matcher,
                        const float &keyframe_threshold, 
                        const bool &log_stats);

    /**
      * Constructor with flexible feature detector, descriptor extractor and matcher, 
      * keyframe threshold, flag to log statistics 
      * and the minimum and maximum number of points to detect
      * The following combinations of (Feature Detector, Descriptor Extractor) are invalid:
      * (ORB, AKAZE); (GFTT, AKAZE); (FAST, AKAZE); (AGAST, AKAZE); (BRISK, AKAZE); (SURF, AKAZE); 
      * (SIFT, AKAZE); (SIFT, ORB);
      * @param feature_detector ORB, AKAZE, GFTT, FAST, AGAST, BRISK, SURF, SIFT
      * @param descriptor_extractor ORB, AKAZE, BRISK, SURF, SIFT
      * @param matcher BRUTEFORCE AND FLANNBASED
      * @param min_pts to detect @param max_pts to detect
      * @param keyframe_threshold threshold that defines whether a frame is a keyframe or not
      * @param log_stats if log statistics should be displayed
      */
    WideBaselineTracker(const std::string &feature_detector,
                        const std::string &descriptor_extractor, 
                        const std::string &matcher,
                        const int &min_pts, 
                        const int &max_pts, 
                        const float &keyframe_threshold, 
                        const bool &log_stats);

    /**
     * Constructor with a set of tracker parameters
     * @param param struct holding different tracker parameters
     */
    WideBaselineTracker(const FeatureTracker::Parameters& param);

    /**
     * Get the "good" Matches of your features
     * @param threshold as double
     */
    void getGoodMatches(const double &threshold);

    /**
     * Tracks keypoints between the current frame and the previous.
     * @param img rgb image
     * @return boolean
     */
    bool track(const cv::Mat &img, const cv::Mat& mask = cv::Mat());

    /**
     * Clear all data about tracked points
     */
    void clear();
};

#endif /* INCLUDE_WIDE_BASELINE_TRACKER_H_ */