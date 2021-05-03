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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <vector>
#include <iterator>

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
    cv::flann::Index* index_ = nullptr;

    //Boolean vector that indicates if the current keypoint i have a match
    std::vector<bool> curr_kpts_with_matches_;

    //The number of followed frames without matches to keypoint i on the map
    std::vector<int> life_of_keypoints_on_the_map_;

    /**
     * Detect KeyPoints and extract descriptors using this function
     */
    void detect_keypoints();
    
    /**
     * Adds keypoints detected in the first frame to the tracker
     */
    void add_keypoints();

    /**
     * Update internal buffers (updates the map keypoints (in the first frame) 
     * with the current points)
     */
    void update_buffers();

    /**
     * Define whether a match is a good match 
     * based on the distance between the two closest matches
     * @param dist_ratio distance ratio between the two closest matches
     */
    void getGoodMatches(const float& dist_ratio);

    /**
     * Update the map features that have been successfully matched
     */ 
    void updateTheMap();

    /**
     * Remove invalid keypoints from the map, i.e, 
     * the keypoints with an expired life
     */
    void removeInvalidKeypointsFromTheMap();

    /**
     * Add new keypoints to the map, i.e,
     * the current keypoints without a good match
     */ 
    void addNewKeypointsToTheMap();

     /**
     * Searches if a keypoint have a match
     * @param keypoint_index id of keypoint
     * @return match id
     */
    int searchMatches(const int& keypoint_index);

    /**
     * Set the current points that have some correspondence 
     * with some point in the previous frame that belong to the map to curr_pts_.
     */
    void setCurrentPoints();

    /**
     * Set the points on the map belonging to the previous frame 
     * that has a correspondence with some point on the current frame to prev_pts_.
     */ 
    void setPreviousPoints();
    
    /**
     * Set the coordinates of the map keypoints to map_pts_.
     */ 
    //void setMapPoints();

    /**
     * Set the feature_detector_ attribute from a string
     * @param feature_detector name of feature_detector, the list of options are:
     * ORB, AKAZE, GFTT, FAST, AGAST, BRISK, SURF, SIFT
     */
    void setFeatureDetector(const std::string& feature_detector);

    /**
     * Set the descriptor_extractor_ attribute from a string
     * @param descriptor_extractor name of descriptor_extractor, the list of options are:
     * ORB, AKAZE, BRISK, SURF, SIFT
     */
    void setDescriptorExtractor(const std::string& descriptor_extractor);

    /**
     * Checks if a valid combination of feature detector and descriptor extractor has been made
     * @param feature_detector name of feature_detector 
     * @param descriptor_extractor name of descriptor_extractor
     * @return boolean
    */
    bool checkCombination(const std::string& feature_detector, 
                          const std::string& descriptor_extractor);

public:

    //Store all the current keypoints founded in the current frame
    std::vector<cv::KeyPoint> curr_kpts_;

    //Store all the current descriptors founded in the current frame
    cv::Mat curr_descriptors_;

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

    //Define if the matche i is a good a match
    std::vector<bool> good_matches_;

    //Store the distances beetween the matches 
    cv::Mat dists_;

    //History of the indices of the images that a keypoint of the map appeared.
    //std::vector<std::vector<int>> inverted_indices_;

    //Number of keypoints tracked in each frame
    int tracked_pts_;

    // Distance Ratio to compare the two closest matches
    const float DIST_RATIO_ = 0.8;

    // Max number of allowed frames without matches for a keypoint.
    int lifespan_of_a_feature_;

    /**
     * Default constructor: create a ORB feature detector and descriptor extractor 
     * and  lifespan_of_a_feature_= 3. 
     */
    FeatureMapTracker();

    /**
     * Constructor with flexible feature detector and descriptor extractor, lifespan of a feature and flag to 
     * log statistics
     * The following combinations of (Feature Detector, Descriptor Extractor) are invalid:
     * (ORB, AKAZE); (GFTT, AKAZE); (FAST, AKAZE); (AGAST, AKAZE); (BRISK, AKAZE); (SURF, AKAZE); 
     * (SIFT, AKAZE); (SIFT, ORB);
     * @param feature_detector ORB, AKAZE, GFTT, FAST, AGAST, BRISK, SURF, SIFT
     * @param descriptor_extractor ORB, AKAZE, BRISK, SURF, SIFT
     * @param lifespan_of_a_feature  Max number of allowed frames without matches for a keypoint.
     * @param log_stats if log statistics should be displayed 
     * 
     */ 
    FeatureMapTracker(const std::string& feature_detector, 
                      const std::string& descriptor_extractor, 
                      const int& lifespan_of_a_feature, const bool& log_stats);
    
    /**
     * Constructor with flexible feature detector and descriptor extractor, 
     * the feature life (K), minimum number of tracked points, 
     * maximum number of tracked points and flag to log statistics
     * The following combinations of (Feature Detector, Descriptor Extractor) are invalid:
     * (ORB, AKAZE); (GFTT, AKAZE); (FAST, AKAZE); (AGAST, AKAZE); (BRISK, AKAZE); (SURF, AKAZE); 
     * (SIFT, AKAZE); (SIFT, ORB);
     * @param feature_detector ORB, AKAZE, GFTT, FAST, AGAST, BRISK, SURF, SIFT
     * @param descriptor_extractor ORB, AKAZE, BRISK, SURF, SIFT
     * @param lifespan_of_a_feature  Max number of allowed frames without matches for a keypoint.
     * @param min_pts to detect @param max_pts to detect
     * @param log_stats if log statistics should be displayed 
     */ 
    FeatureMapTracker(const std::string& feature_detector, 
                      const std::string& descriptor_extractor, 
                      const int& lifespan_of_a_feature, const int& min_pts,
                      const int& max_pts, const bool& log_stats);

    /**
     * Default Destructor
     */ 
    ~FeatureMapTracker();

     /**
     * Tracks keypoints between the current frame and the map.
     * @param img rgb image
     * @return boolean
     */
    bool track(const cv::Mat& img);

    /**
     * Clear all data about tracked points
     */
    void clear();
};

#endif /* INCLUDE_FEATURE_MAP_TRACKER_H_ */
