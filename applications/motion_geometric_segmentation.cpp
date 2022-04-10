/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2019, Natalnet Laboratory for Perceptual Robotics
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
 */

#include <cstdio>
#include <cstdlib>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>

#include <geometry.h>
#include <config_loader.h>
#include <rgbd_loader.h>
#include <event_logger.h>
#include <klt_tracker.h>
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>
#include <segmentation/scored_fbmt.h>

using namespace cv;

/**
 * This program shows the use of camera motion estimation based on
 * KLT keypoint tracking and RANSAC.
 * @param .yml config. file (from which index_file is used)
 */

void writePoseToFile(ofstream &poses_file_, Eigen::Affine3f pose_, const std::string &time_stamp)
{
	Eigen::Matrix3f Rot;
	Rot(0, 0) = pose_(0, 0);
	Rot(0, 1) = pose_(0, 1);
	Rot(0, 2) = pose_(0, 2);
	Rot(1, 0) = pose_(1, 0);
	Rot(1, 1) = pose_(1, 1);
	Rot(1, 2) = pose_(1, 2);
	Rot(2, 0) = pose_(2, 0);
	Rot(2, 1) = pose_(2, 1);
	Rot(2, 2) = pose_(2, 2);

	Eigen::Quaternionf q(Rot);

	poses_file_ << time_stamp << " " << pose_(0, 3) << " "
							<< pose_(1, 3) << " "
							<< pose_(2, 3) << " "
							<< q.x() << " "
							<< q.y() << " "
							<< q.z() << " "
							<< q.w() << "\n";
}
int main(int argc, char **argv)
{
	EventLogger &logger = EventLogger::getInstance();
	logger.setVerbosityLevel(EventLogger::L_DEBUG);

	RGBDLoader loader;
	KLTTracker tracker;
	Intrinsics intr(0);
	ReconstructionVisualizer visualizer;
	//PCLVisualizerPtr test_viewer;

	Mat frame, depth;
	string index_file;
	float ransac_distance_threshold, ransac_inliers_ratio;
	Eigen::Affine3f pose = Eigen::Affine3f::Identity();
	boost::shared_ptr<Eigen::Affine3f> trans = boost::make_shared<Eigen::Affine3f>(Eigen::Affine3f::Identity());
	pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr curr_cloud(new pcl::PointCloud<PointT>);
	std::ofstream file("poses.txt");
	if (argc != 2)
	{
		logger.print(EventLogger::L_INFO, "[motion_image_geometric_segmentation.cpp] Usage: %s <path/to/config_file.yaml>\n", argv[0]);
		exit(0);
	}

	ConfigLoader param_loader(argv[1]);
	param_loader.checkAndGetString("index_file", index_file);

	param_loader.checkAndGetFloat("ransac_distance_threshold", ransac_distance_threshold);
	param_loader.checkAndGetFloat("ransac_inliers_ratio", ransac_inliers_ratio);

	loader.processFile(index_file);

	MotionEstimatorRANSAC motion_estimator(intr, ransac_distance_threshold,
																				 ransac_inliers_ratio);
	motion_estimator.setMinInliersNumber(10);

	ScoredFBMT segmenter(&motion_estimator, 0.1, 2);

	cv::Mat mask;
	std::string rgb_img_time_stamp;

	vector<Point2f> prev_static_pts;
	vector<Point2f> curr_static_pts;
	//Track points on each image
	for (int i = 0; i < loader.num_images_; i++)
	{
		//Load RGB-D image and point cloud
		loader.getNextImage(frame, depth);
		rgb_img_time_stamp = loader.getNextRgbImageTimeStamp();

		*curr_cloud = getPointCloud(frame, depth, intr);
		//Track feature points in the current frame
		tracker.track(frame);

		//Estimate motion between the current and the previous frame/point clouds
		if (i > 0)
		{
			try
			{
				auto estimated_trans = motion_estimator.estimate(tracker.prev_pts_, prev_cloud,
																												 tracker.curr_pts_, curr_cloud);

				*trans = estimated_trans;

				vector<int> static_pt_idxs = segmenter.getStaticPointsIndexes(tracker.curr_pts_);

				curr_static_pts.clear();
				prev_static_pts.clear();

				for (int i = 0; i < static_pt_idxs.size(); i++)
				{
					const int &idx = static_pt_idxs[i];
					curr_static_pts.push_back(tracker.curr_pts_[idx]);
					prev_static_pts.push_back(tracker.prev_pts_[idx]);
				}

				size_t size = std::min(curr_static_pts.size(), prev_static_pts.size());

				if (size > 0)
				{
					for (size_t k = 0; k < size; k++)
					{
						Point2i pt1 = (curr_static_pts)[k];
						Point2i pt2 = (prev_static_pts)[k];
						cv::circle(frame, pt1, 1, cv::Scalar(147, 0, 147), -1);
						cv::circle(frame, pt2, 3, cv::Scalar(147, 0, 147), -1);
						line(frame, pt1, pt2, CV_RGB(0, 0, 255));
					}

					auto refined_trans = motion_estimator.estimate(prev_static_pts, prev_cloud,
																												 curr_static_pts, curr_cloud);
					*trans = refined_trans;
				}
			}
			catch (const std::exception &e)
			{
				*trans = Eigen::Affine3f::Identity();
				logger.print(EventLogger::L_ERROR, "[motion_image_geometric_segmentation.cpp] Error: %s\n", e.what());
			}

			pose = pose * (*trans);
		}

		//View tracked points
		for (size_t k = 0; k < tracker.curr_pts_.size(); k++)
		{
			Point2i pt1 = tracker.prev_pts_[k];
			Point2i pt2 = tracker.curr_pts_[k];
			//circle(frame, pt1, 1, CV_RGB(255,0,0), -1);
			//circle(frame, pt2, 3, CV_RGB(0,0,255), -1);
			//line(frame, pt1, pt2, CV_RGB(0,0,255));
		}

		if (i == 0)
			visualizer.addReferenceFrame(pose, "origin");
		//visualizer.addQuantizedPointCloud(curr_cloud, 0.3, pose);
		visualizer.viewReferenceFrame(pose);
		//visualizer.viewPointCloud(curr_cloud, pose);
		//visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, pose);

		visualizer.spinOnce();

		writePoseToFile(file, pose, rgb_img_time_stamp);
		//Show RGB-D image
		imshow("Image view", frame);
		imshow("Depth view", depth);
		if (!mask.empty())
			imshow("Mask view, black points = dynamic points", mask);

		char key = waitKey(1);
		if (key == 27 || key == 'q' || key == 'Q')
		{
			logger.print(EventLogger::L_INFO, "[motion_image_geometric_segmentation.cpp] Exiting\n", argv[0]);
			break;
		}
		//Let the prev. cloud in the next frame be the current cloud
		*prev_cloud = *curr_cloud;
	}

	visualizer.close();
	file.close();
	return 0;
}