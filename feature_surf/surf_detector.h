#include "opencv2/features2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <feature_surf.h>
#include <vector>

using namespace cv;
using namespace cv::xfeatures2d;
/*
 * Author: Luiz Felipe Maciel Correia
 * y9luizufrn@gmail.com
 */

class SurfDetector
{
    protected:
    //Detector object 
    Ptr<SURF> detector_;
    //current gray frame
    Mat curr_frame_gray_;
    //previous gray frame
    Mat prev_frame_gray_;
    //minimum hessian constant for the surf detector
    double minHessian_;
    //maximum number of surf features
    int numMax_;
    //flag variable to indicated if the process is able to matching features
    //that is, if we have 2 or more frames to matching process
    bool initialized;

  public:
    //Store all the current keypoints founded in the current frame
    vector<KeyPoint> curr_KPs_;
    //Store all the current descriptors founded in the current frame
    Mat curr_descriptors_;
    //Store all the previous keypoints founded in the current frame
    vector<KeyPoint> prev_KPs_;
    //Store all the previous descriptors founded in the current frame
    Mat prev_descriptors_;
    //Store all the "good" current keypoints that's points who has more
    //accurracy than others calculed through euclidian distance 
    vector<Point2f> curr_good_Pts;
    //Store all the "good" previous keypoints that's points who has more
    //accurracy than others calculed through euclidian distance 
    vector<Point2f> prev_good_Pts;
    //Store all the matches beetween features
    std::vector< DMatch > matches;
    //Store only the "good" matches, in other words, store only the matchings
    //of "good" features
    std::vector<DMatch> good_matches;
    //Matcher who store all the descriptors of the scene
    Ptr<cv::FlannBasedMatcher> matcher;
    //default constructor
    SurfDetector();
    //Constructor with the minimum hessian value
    SurfDetector(double minHessian);
    //Detect KeyPoints and descriptors using this function
    void Detect(Mat curr_frame);
    //Get the "good" Matches your surf features
    void getGoodMatches(double threshold);
    //this function calls Detect and getGoodMatches
    void DetectAndGetGoodMatches(Mat curr_frame);
    //Only draw the keypoints
    void Draw(Mat prev_img,Mat curr_img);
    //searching in your matcher for a passed descriptor set
    void SearchInTree(Mat descriptors);
};