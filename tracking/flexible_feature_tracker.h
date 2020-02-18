/*
 * Authors: 
 * Luiz Felipe Maciel Correia (y9luizufrn@gmail.com)
 * Marcos Henrique Fernandes Marcone (marcosmarcone48@gmail.com)
 */

#include "opencv2/features2d.hpp"
#include <opencv2/core/core.hpp>
#include <vector>

#include <common_types.h>

using namespace cv;



class FlexibleFeatureTracker
{
protected:
    // Feature Tracker object
    Ptr<cv::FeatureDetector> feature_detector_;

    // Descriptor Extractor object
    Ptr<cv::DescriptorExtractor> descriptor_extractor_;

    // Current gray frame
    Mat curr_frame_gray_;

    // Previous gray frame
    Mat prev_frame_gray_;

    // Maximum number of features
    int numMax_;

    // Flag variable to indicated if the process is able to matching features
    // that is, if we have 2 or more frames to matching process
    bool initialized_;

    // Current frame number
    int frame_idx_;

    // Boolean vector that indicates if the keypoint i have a match
    std::vector<bool> keypoints_with_matches;

    // Adds keypoints detected in the first frame to the tracker
    void addKeypoints();

    // Searches if a keypoint have a match
    int searchMatches(int keypoint_index);

public:
    
    // Tracklets: history of each point as a vector of point2f
    std::vector<Tracklet> tracklets_;

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

    //Matcher who store all the descriptors of the scene
    Ptr<cv::DescriptorMatcher> matcher_;

    //Default constructor: create a ORB detector and extractor and a BruteForce matcher.
    FlexibleFeatureTracker();

    //Constructor with flexible feature detector, descriptor extractor and matcher
    FlexibleFeatureTracker(Ptr<cv::FeatureDetector> feature_detector, Ptr<cv::DescriptorExtractor> descriptor_extractor, Ptr<cv::DescriptorMatcher> matcher);

    //Detect KeyPoints and extract descriptors using this function
    void detect(Mat curr_frame);

    //Get the "good" Matches of your features
    void getGoodMatches(double threshold);

    // Update internal buffers (updates the previous points (in the current frame)
    // with the current points (from the previous frame))
    void updateBuffers();

    //Tracks keypoints between the current frame and the previous.
    void track(Mat curr_frame);
};
