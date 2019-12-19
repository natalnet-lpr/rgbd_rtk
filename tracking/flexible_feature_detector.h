#include "opencv2/features2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>

using namespace cv;
using namespace cv::xfeatures2d;
/*
 * Authors: 
 * Luiz Felipe Maciel Correia (y9luizufrn@gmail.com)
 * Marcos Henrique Fernandes Marcone (marcosmarcone48@gmail.com)
 */

class FlexibleFeatureDetector
{
    protected:
    //Detector object 
    Ptr<Feature2D> detector_;
    //current gray frame
    Mat curr_frame_gray_;
    //previous gray frame
    Mat prev_frame_gray_;
    //maximum number of features
    int numMax_;
    //flag variable to indicated if the process is able to matching features
    //that is, if we have 2 or more frames to matching process
    bool initialized_;

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
    //Matcher who store all the descriptors of the scene
    Ptr<cv::DescriptorMatcher> matcher_;
    //default constructor: create a ORB detector and a BruteForce matcher.
    FlexibleFeatureDetector();
    //Constructor with a generic detector and a generic matcher
    FlexibleFeatureDetector(Ptr<Feature2D> detector, Ptr<cv::DescriptorMatcher> matcher);
    //Detect KeyPoints and descriptors using this function
    void detect(Mat curr_frame);
    //Get the "good" Matches of your features
    void getGoodMatches(double threshold);
    //this function calls Detect and getGoodMatches
    void detectAndGetGoodMatches(Mat curr_frame);
    //Only draw the keypoints
    void draw(Mat prev_img,Mat curr_img);
    //searching in your matcher for a passed descriptor set
    void searchInTree(Mat descriptors);
};