#ifndef INCLUDE_SURF_DETECTOR_H_
#define INCLUDE_SURF_DETECTOR_H_
#include "opencv2/features2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>

using namespace cv;
using namespace cv::xfeatures2d;
using std::vector;
using namespace std;
/*
 * Author: Luiz Felipe Maciel Correia
 * y9luizufrn@gmail.com
 */



class SurfDetector
{
    protected:

    cv::Ptr<cv::FeatureDetector> detector_;

    cv::Ptr<cv::DescriptorMatcher> matcher_;

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
    bool initialized_;

    //train informations
    vector<vector<KeyPoint>> trainKPs_;
    vector<Mat> trainImages_;
    vector<Mat> trainDescriptors_;

    //query information
    vector<KeyPoint> queryKPs_;
    Mat queryImage_;
    Mat queryDescriptors_;

    vector<DMatch> refinedMatches_;

    void MatchDescriptors();

  public:
    vector<Point2f> prev_good_Pts_;
    vector<Point2f> curr_good_Pts_;

    Mat getLastTrainDescriptor();

    SurfDetector();
    SurfDetector(double minHessian);
    void detect(Mat curr_frame, Mat prev_frame);

    void detectAndMatch(Mat curr_frame, Mat prev_frame);
    void drawSurfMatches(cv::Mat prev_img,cv::Mat curr_img);
    //search in all train set, to find the 'n' images with the highest number of matches
    vector<int> searchDescriptor(Mat queryDescriptor,int n);
    cv::Ptr<cv::DescriptorMatcher> getMatcher();

};
#endif