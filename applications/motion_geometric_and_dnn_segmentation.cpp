/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2022, Natalnet Laboratory for Perceptual Robotics
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
 *  Luiz Correia
 */

#include <Eigen/Geometry>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <common/constants.h>
#include <config_loader.h>
#include <event_logger.h>
#include <geometry.h>
#include <klttw_tracker.h>
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>
#include <rgbd_loader.h>
#include <segmentation/scored_mask_rcnnmt.h>

using namespace std;
using namespace cv;
using namespace pcl;

/**
 * This program shows the use of camera motion estimation based on
 * KLT keypoint tracking and RANSAC.
 * @param .yml config. file (from which index_file is used)
 */
int main(int argc, char** argv)
{
    EventLogger& logger = EventLogger::getInstance();
    logger.setVerbosityLevel(EventLogger::L_DEBUG);

    RGBDLoader loader;
    Size sz(5, 5);
    KLTTWTracker tracker(600, 5000, sz, 0);
    Intrinsics intr(3);
    ReconstructionVisualizer visualizer;

    Mat frame, depth;
    string index_file;
    float ransac_distance_threshold, ransac_inliers_ratio;
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    Eigen::Affine3f refined_trans = Eigen::Affine3f::Identity();

    PointCloud<PointT>::Ptr prev_cloud(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr curr_cloud(new PointCloud<PointT>);

    if (argc != 2)
    {
        logger.print(EventLogger::L_INFO, "[motion_estimation_test.cpp] Usage: %s <path/to/config_file.yaml>\n",
                     argv[0]);
        exit(0);
    }

    ConfigLoader param_loader(argv[1]);
    param_loader.checkAndGetString("index_file", index_file);
    param_loader.checkAndGetFloat("ransac_distance_threshold", ransac_distance_threshold);
    param_loader.checkAndGetFloat("ransac_inliers_ratio", ransac_inliers_ratio);
    loader.processFile(index_file);

    MotionEstimatorRANSAC motion_estimator(intr, ransac_distance_threshold, ransac_inliers_ratio);
    ScoredMaskRcnnMT motion_treater;

    vector<MaskRcnnClass> mask_rcnn_classes;
    mask_rcnn_classes.push_back(MaskRcnnClass::Person);
    mask_rcnn_classes.push_back(MaskRcnnClass::Car);

    motion_treater.initializeMaskRcnnDnnMT(cv::dnn::Backend::DNN_BACKEND_DEFAULT, cv::dnn::Target::DNN_TARGET_CPU,
                                           mask_rcnn_classes);
    motion_treater.initializeScoredFBMT(&motion_estimator, intr, 0.6, 2);

    //Track points on each image
    for (int i = 0; i < loader.num_images_; i++)
    {
        std::string timestamp;

        //Load RGB-D image and point cloud
        loader.getNextImage(frame, depth);
        *curr_cloud = getPointCloud(frame, depth, intr);

        Mat mask;

        motion_treater.segment(frame, mask, 0.7);

        //Track feature points in the current frame
        tracker.track(frame, mask);

        //Estimate motion between the current and the previous frame/point clouds
        if (i > 0)
        {
            try
            {
                trans = motion_estimator.estimate(tracker.prev_pts_, prev_cloud, tracker.curr_pts_, curr_cloud);

                vector<int> static_pts_idxs = motion_treater.estimateStaticPointsIndexes(tracker.curr_pts_);

                vector<cv::Point2f> curr_static_pts;
                vector<cv::Point2f> prev_static_pts;

                for (size_t k = 0; k < static_pts_idxs.size(); k++)
                {
                    const int& idx = static_pts_idxs[k];
                    const cv::Point2f& pt1 = tracker.curr_pts_[idx];
                    const cv::Point2f& pt2 = tracker.prev_pts_[idx];

                    curr_static_pts.push_back(pt1);
                    prev_static_pts.push_back(pt2);

                    circle(frame, pt1, 1, CV_RGB(147, 0, 147), -1);
                    circle(frame, pt2, 3, CV_RGB(0, 255, 0), -1);
                    line(frame, pt1, pt2, CV_RGB(0, 0, 255));
                }

                refined_trans = motion_estimator.estimate(prev_static_pts, prev_cloud, curr_static_pts, curr_cloud);
                trans = refined_trans;
            }
            catch (std::exception& e)
            {
                trans = Eigen::Affine3f::Identity();

                logger.print(EventLogger::L_ERROR, "[motion_image_geometric_and_dnn_segmentation.cpp] Error: %s\n",
                             e.what());
            }

            pose = pose * (trans);
        }

        if (i == 0)
            visualizer.addReferenceFrame(pose, "origin");
        //visualizer.addQuantizedPointCloud(curr_cloud, 0.3, pose);
        visualizer.viewReferenceFrame(pose);
        //visualizer.viewPointCloud(curr_cloud, pose);
        //visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, pose);

        visualizer.spinOnce();

        //Show RGB-D image
        imshow("Image view", frame);
        imshow("Depth view", depth);
        imshow("Mask view", mask);

        char key = waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            logger.print(EventLogger::L_INFO, "[motion_estimator_test.cpp] Exiting\n", argv[0]);
            break;
        }

        //Let the prev. cloud in the next frame be the current cloud
        *prev_cloud = *curr_cloud;
    }

    visualizer.close();

    return 0;
}