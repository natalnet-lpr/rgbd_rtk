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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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

#include <Eigen/Geometry>
#include <cstdio>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <config_loader.h>
#include <event_logger.h>
#include <geometry.h>
#include <optical_flow_visual_odometry.h>
#include <reconstruction_visualizer.h>
#include <rgbd_loader.h>

using namespace std;
using namespace cv;

/**
 * This program shows the use of camera pose estimation using optical flow visual odometry.
 * @param .yml config. file (from which index_file is used)
 */
int main(int argc, char** argv)
{
	EventLogger& logger = EventLogger::getInstance();
	logger.setVerbosityLevel(EventLogger::L_DEBUG);
	
	ReconstructionVisualizer visualizer;
	RGBDLoader loader;
	FeatureTracker::Parameters tracking_param;
	Intrinsics intr(0);

	string index_file;
	Mat frame, depth;
	float ransac_thr;

	std::string rgb_img_time_stamp;

	if(argc != 2)
	{
		logger.print(EventLogger::L_INFO, "[optical_flow_visual_odometry_test.cpp] Usage: %s <path/to/config_file.yaml>\n", argv[0]);
		exit(0);
	}

	ConfigLoader param_loader(argv[1]);
	//file parameters
    param_loader.checkAndGetString("index_file", index_file);
    //tracker parameters
    param_loader.checkAndGetString("tracker_type", tracking_param.type_);
    param_loader.checkAndGetInt("min_pts", tracking_param.min_pts_);
    param_loader.checkAndGetInt("max_pts", tracking_param.max_pts_);
    param_loader.checkAndGetBool("log_stats", tracking_param.log_stats_);
    //motion estimator parameters
    param_loader.checkAndGetFloat("ransac_distance_threshold", ransac_thr);

	OpticalFlowVisualOdometry vo(intr, tracking_param, ransac_thr);

	loader.processFile(index_file);

	visualizer.addReferenceFrame(vo.pose_, "origin");

	//Compute visual odometry on each image
	for(int i = 0; i < loader.num_images_; i++)
	{
		//Load RGB-D image 
		loader.getNextImage(frame, depth);

		// Get the rgb image time stamp
		rgb_img_time_stamp = loader.getNextRgbImageTimeStamp();

		//Estimate current camera pose
		bool is_kf = vo.computeCameraPose(frame, depth, rgb_img_time_stamp);

		//View tracked points
		for(size_t k = 0; k < vo.tracker_ptr_->curr_pts_.size(); k++)
		{
			Point2i pt1 = vo.tracker_ptr_->prev_pts_[k];
			Point2i pt2 = vo.tracker_ptr_->curr_pts_[k];
			Scalar color;

			is_kf ? color = CV_RGB(255,0,0) : color = CV_RGB(0,0,255);

			circle(frame, pt1, 1, color, -1);
			circle(frame, pt2, 3, color, -1);
			line(frame, pt1, pt2, color);
		}

		visualizer.viewReferenceFrame(vo.pose_);
		visualizer.viewPointCloud(vo.curr_dense_cloud_, vo.pose_);
		//visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.05, vo.pose_);
		//visualizer.addCameraPath(vo.pose_);

		if(is_kf)
		{
			Keyframe kf = vo.createKeyframe(i);
			visualizer.addQuantizedPointCloud(kf.local_cloud_, 0.02, kf.pose_);
			//visualizer.addPointCloud(kf.local_cloud_, kf.pose_);
		}

		visualizer.spinOnce();

		//Show RGB-D image
		imshow("Image view", frame);
		imshow("Depth view", depth);
		char key = waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			logger.print(EventLogger::L_INFO, "[optical_flow_visual_odometry_test.cpp] Exiting\n", argv[0]);
			break;
		}
		vo.writePoseToFile(rgb_img_time_stamp);
	}

	visualizer.close();

	return 0;
}