/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2021, Natalnet Laboratory for Perceptual Robotics
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
 *  Authors:
 *
 *  Bruno Silva
 */

#include <cstdio>
#include <cstdlib>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/sfm/fundamental.hpp>

#include <geometry.h>
#include <config_loader.h>
#include <kitti_stereo_loader.h>
#include <stereo_cloud_generator.h>
#include <reconstruction_visualizer.h>

using namespace std;
using namespace cv;

vector<Point2f> keypointsToPoints(const vector<KeyPoint>& kpts)
{
    vector<Point2f> pts;
    for(size_t i = 0; i < kpts.size(); i++)
    {
        pts.push_back(kpts[i].pt);
    }

    return pts;
}

int main(int argc, char **argv)
{
    string root_path;
    KITTIStereoLoader loader;
    ConfigLoader cfg("../../config_files/calib_cam_to_cam.yaml");
    Mat left, right, left_orig, right_orig;

    if(argc != 4)
    {
        fprintf(stderr, "Usage: %s <kitti datasets root dir.> <sequence_number> <use_color_camera>\n", argv[0]);
        fprintf(stderr, "Example: /path/to/parent/directory/of/sequences 11 0 - loads the KITTI grayscale sequence number 11.\n\n");
        fprintf(stderr, "That is, if the 00, 01, 02, ... folders are within /Datasets/KITTI_Datasets_Gray/sequences,\n");
        fprintf(stderr, "the grayscale sequence 11 is loaded with:\n");
        fprintf(stderr, "%s /Datasets/KITTI_Datasets_Gray/ 11 0\n", argv[0]);
        fprintf(stderr, "(the 'sequences' part of the path is added automatically and must be omitted).\n");
        exit(0);
    }

    root_path = argv[1];
    loader.loadStereoSequence(root_path, atoi(argv[2]), atoi(argv[3]));

    Mat K_l, K_r, R, T, S, E, F, P_l, P_r;

    //cfg.checkAndGetMat("K_00", K_l);
    //cfg.checkAndGetMat("K_01", K_r);
    cfg.checkAndGetMat("P_rect_00", P_l);
    K_l = Mat::eye(3, 3, CV_64F);
    K_l.at<double>(0,0) = P_l.at<double>(0,0); K_l.at<double>(0,1) = P_l.at<double>(0,1); K_l.at<double>(0,2) = P_l.at<double>(0,2);
    K_l.at<double>(1,0) = P_l.at<double>(1,0); K_l.at<double>(1,1) = P_l.at<double>(1,1); K_l.at<double>(1,2) = P_l.at<double>(1,2);
    K_l.at<double>(2,0) = P_l.at<double>(2,0); K_l.at<double>(2,1) = P_l.at<double>(2,1); K_l.at<double>(2,2) = P_l.at<double>(2,2);
    cfg.checkAndGetMat("P_rect_01", P_r);
    K_r = Mat::eye(3, 3, CV_64F);
    K_r.at<double>(0,0) = P_r.at<double>(0,0); K_r.at<double>(0,1) = P_r.at<double>(0,1); K_r.at<double>(0,2) = P_r.at<double>(0,2);
    K_r.at<double>(1,0) = P_r.at<double>(1,0); K_r.at<double>(1,1) = P_r.at<double>(1,1); K_r.at<double>(1,2) = P_r.at<double>(1,2);
    K_r.at<double>(2,0) = P_r.at<double>(2,0); K_r.at<double>(2,1) = P_r.at<double>(2,1); K_r.at<double>(2,2) = P_r.at<double>(2,2);
    R = Mat::eye(3, 3, CV_64F);
    T = Mat::zeros(3, 1, CV_64F); //set values manually
    T.at<double>(0,0) = -0.537; T.at<double>(1,0) = 0.0; T.at<double>(2,0) = 0.0;

    S = Mat::zeros(3, 3, CV_64F);
    S.at<double>(0,0) =  0.0;               S.at<double>(0,1) = -T.at<double>(0,2); S.at<double>(0,2) = T.at<double>(0,1);
    S.at<double>(1,0) =  T.at<double>(0,2); S.at<double>(1,1) = 0.0;                S.at<double>(1,2) = -T.at<double>(0,0);
    S.at<double>(2,0) = -T.at<double>(0,1); S.at<double>(2,1) =  T.at<double>(0,0); S.at<double>(2,2) = 0.0;

    E = R*S;
    printf("Essential matrix:\n");
    cout << E << endl;
    
    double n = norm(T);

    vector<Mat> Rs, ts;

    printf("True translation:\n");
    cout << T << endl;
    printf("True rotation:\n");
    cout << R << endl;
    cv::sfm::motionFromEssential(E, Rs, ts);
    for(size_t i = 0; i < Rs.size(); i++)
    {
        printf("Solution %lu\n", i);
        cout << Rs[i] << endl;
        cout << n*ts[i] << endl;
    }

    cv::sfm::fundamentalFromEssential(E, K_l, K_r, F);
    printf("Fundamental matrix:\n");
    cout << F << endl;

    vector<KeyPoint> kpts_left, kpts_right;
    Mat desc_left, desc_right;
    vector<Point2f> pts_left, pts_right;
    vector<Point3f> lines_left, lines_right;

    left_orig = loader.getNextLeftImage(false);
    right_orig = loader.getNextRightImage(false);
    cvtColor(left_orig, left, CV_GRAY2RGB);
    cvtColor(right_orig, right, CV_GRAY2RGB);
    //Mat result(left.rows, left.cols + right.cols, CV_8UC1);
    //left.copyTo(result(Rect(0, 0, left.cols, left.rows)));
    //right.copyTo(result(Rect(left.cols, 0, right.cols, right.rows)));

    Ptr<ORB> orb = ORB::create(300);
    Ptr<DescriptorMatcher> matcher_ = DescriptorMatcher::create("BruteForce");
    orb->detectAndCompute(left, noArray(), kpts_left, desc_left);
    orb->detectAndCompute(right, noArray(), kpts_right, desc_right);

    pts_left = keypointsToPoints(kpts_left);
    computeCorrespondEpilines(pts_left, 1, F, lines_left);
    //pts_right = keypointsToPoints(kpts_right);
    //computeCorrespondEpilines(pts_right, 2, F, lines_right);

    //Draw info on left image
    for(size_t i = 0; i < pts_left.size(); i++)
    {
        if(i % 50 == 0)
        {
            int x0, y0, x1, y1;

            x0 = 0;
            y0 = -lines_left[i].z/lines_left[i].y;
            x1 = left.cols;
            y1 = -(lines_left[i].z + lines_left[i].x*left.cols)/lines_left[i].y;

            line(right, Point(x0, y0), Point(x1, y1), CV_RGB(255, 0, 0));
            circle(left, pts_left[i], 5, CV_RGB(255, 0, 0), -1);

            printf("\npoint-y: %f\n", pts_left[i].y);
            printf("y0: %i\n", y0);
            printf("y1: %i\n", y1);
        }
    }

    //Draw info on right image
    /*
    for(size_t i = 0; i < pts_right.size(); i++)
    {
        line(left, Point(0, -lines_right[i].z/lines_right[i].y), Point(right.cols, -(lines_right[i].z + lines_right[i].x*right.cols)/lines_right[i].y), CV_RGB(255, 0, 0));
        circle(right, kpts_right[i].pt, 5, CV_RGB(255, 0, 0), -1);
    }
    */

    imshow("Left", left);
    imshow("Right", right);
    char key = waitKey(0);

    return 0;
}
