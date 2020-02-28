/*
 * Authors: 
 * Luiz Felipe Maciel Correia (y9luizufrn@gmail.com)
 * Marcos Henrique Fernandes Marcone (marcosmarcone48@gmail.com)
 */

#ifndef INCLUDE_FLEXIBLE_FEATURE_TRACKER_H_
#define INCLUDE_FLEXIBLE_FEATURE_TRACKER_H_

#include "opencv2/features2d.hpp"
#include <opencv2/core/core.hpp>
#include <vector>

#include <feature_tracker.h>
#include <common_types.h>

using namespace cv;


class FlexibleFeatureTracker : public FeatureTracker
{

protected:
    // Feature Tracker object
    Ptr<cv::FeatureDetector> feature_detector_;

    // Descriptor Extractor object
    Ptr<cv::DescriptorExtractor> descriptor_extractor_;

    //Matcher who store all the descriptors of the scene
    Ptr<cv::DescriptorMatcher> matcher_;

    // Maximum number of features
    int numMax_;

    // Boolean vector that indicates if the keypoint i have a match
    std::vector<bool> keypoints_with_matches;

    //Detect KeyPoints and extract descriptors using this function
    void detect_keypoints();
    
    // Adds keypoints detected in the first frame to the tracker
    void add_keypoints();

    // Update internal buffers (updates the previous points (in the current frame)
    // with the current points (from the previous frame))
    void update_buffers();

    // Searches if a keypoint have a match
    int searchMatches(int keypoint_index);

public:
    
    //Store all the current keypoints founded in the current frame
    std::vector<KeyPoint> curr_KPs_;

    //Store all the current descriptors founded in the current frame
    Mat curr_descriptors_;

    //Store all the previous keypoints founded in the previous frame
    std::vector<KeyPoint> prev_KPs_;

    //Store all the previous descriptors founded in the previous frame
    Mat prev_descriptors_;

    //Store all the "good" current keypoints that's points who has more
    //accurracy than others calculed through euclidian distance
    std::vector<Point2f> curr_good_Pts_;

    //Store all the "good" previous keypoints that's points who has more
    //accurracy than others calculed through euclidian distance
    std::vector<Point2f> prev_good_Pts_;

    //Store all the matches beetween features
    std::vector<DMatch> matches_;

    //Store only the "good" matches, in other words, store only the matchings
    //of "good" features
    std::vector<DMatch> good_matches_;

    //Default constructor: create a ORB detector and extractor and a BruteForce matcher.
    FlexibleFeatureTracker();

    //Constructor with flexible feature detector, descriptor extractor and matcher
    FlexibleFeatureTracker(Ptr<cv::FeatureDetector> feature_detector, Ptr<cv::DescriptorExtractor> descriptor_extractor, Ptr<cv::DescriptorMatcher> matcher);
    
    //Get the "good" Matches of your features
    void getGoodMatches(double threshold);
    
    //Tracks keypoints between the current frame and the previous.
    bool track(const cv::Mat& img);
};

#endif /* INCLUDE_FLEXIBLE_FEATURE_TRACKER_H_ */
