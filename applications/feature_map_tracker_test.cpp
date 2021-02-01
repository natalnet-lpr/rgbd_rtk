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
 *  Marcos Henrique F. Marcone
 *  Bruno Silva
 */

#include <cstdio>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rgbd_loader.h>
#include <feature_map_tracker.h>
#include <config_loader.h>
#include <event_logger.h>
#include <common_types.h>
#include <fstream>

using namespace std;
using namespace cv;

void draw_current_features(Mat &img, const vector<Point2f> curr_pts_);

/**
 * This program shows the use of tracking by detection algorithm.
 * @param .yml config. file (from which index_file is used)
 */
int main(int argc, char **argv)
{
	EventLogger& logger = EventLogger::getInstance();
	logger.setVerbosityLevel(EventLogger::L_DEBUG);
	
	RGBDLoader loader;
	string index_file;
	int log_stats, lifespan_of_a_feature;
	Mat frame, depth, current_frame, previous_frame;
	string feature_detector, descriptor_extractor;

	
	
	if (argc != 2)
	{
		logger.print(EventLogger::L_INFO, "[feature_map_tracker_test.cpp] Usage: %s <path/to/config_file.yaml>\n", argv[0]);
		exit(0);
	}

	ofstream file;
	file.open("log-feature_map_tracker_test.txt");

	ConfigLoader param_loader(argv[1]);
	param_loader.checkAndGetString("index_file", index_file);
	param_loader.checkAndGetInt("log_stats", log_stats);
	param_loader.checkAndGetString("feature_detector", feature_detector);
	param_loader.checkAndGetString("descriptor_extractor", descriptor_extractor);
	param_loader.checkAndGetInt("lifespan_of_a_feature",lifespan_of_a_feature);
	
	
	FeatureMapTracker feature_map_tracker(feature_detector, descriptor_extractor,lifespan_of_a_feature,log_stats);

	loader.processFile(index_file);
	int frame_number = 0;
	// Track points on each image
	for (int i = 0; i < loader.num_images_; i++)
	{
		loader.getNextImage(frame, depth);

		double el_time = (double) cvGetTickCount();
		bool is_kf = feature_map_tracker.track(frame);
		el_time = ((double) cvGetTickCount() - el_time)/(cvGetTickFrequency()*1000.0);
		logger.print(EventLogger::L_INFO,"[feature_map_tracker_test.cpp] INFO: Tracking time: %f ms\n", el_time);

		
		//draw_current_features(frame, feature_map_tracker.curr_pts_);
		//logger.print(EventLogger::L_INFO, "[feature_map_tracker_test.cpp] INFO: Current Keypoints: \n");
		file << "[feature_map_tracker_test.cpp] INFO: Current Keypoints: \n";

		for (size_t i = 0; i < feature_map_tracker.curr_kpts_.size(); i++)
		{
			file << frame_number << " - Current Keypoint [" << i << "]" << endl;
			file << "Coordinate: X = " << feature_map_tracker.curr_kpts_[i].pt.x << " -- Y = " << feature_map_tracker.curr_kpts_[i].pt.y << endl;
			file << "Descriptor: " << feature_map_tracker.curr_descriptors_.row(i) << endl;
			file << "------------------------------------------------------------------" << endl;
		}

		if (i>0)
		{
			draw_current_features(frame,feature_map_tracker.curr_pts_);
			//logger.print(EventLogger::L_INFO, "[feature_map_tracker_test.cpp] INFO: Good Matches: \n");
			file << "[feature_map_tracker_test.cpp] INFO: Good Matches: \n";
			for (int i = 0; i < feature_map_tracker.matches_.rows; i++)
			{
				if (feature_map_tracker.good_matches_[i])
				{
					file << frame_number <<": " <<feature_map_tracker.matches_.at<int>(i,0) << " --> " << i << endl;
				}
			}

		}


		file << "------------------------------------------------------------------" << endl;

		//logger.print(EventLogger::L_INFO,"[feature_map_tracker_test.cpp] INFO: Map Keypoints: \n");
		file << "[feature_map_tracker_test.cpp] INFO: Map Keypoints: \n";
		for (size_t i = 0; i < feature_map_tracker.map_kpts_.size(); i++)
		{

			// Feature Life
			file << frame_number << " - Map Keypoint [" << i << "]" << endl;
			file << "Coordinate: X = " << feature_map_tracker.map_kpts_[i].pt.x << " -- Y = " << feature_map_tracker.map_kpts_[i].pt.y << endl;
			file << "Descriptor: " << feature_map_tracker.map_descriptors_.row(i) << endl;
			file << "------------------------------------------------------------------" << endl;
		}

		// Tracklets
		//logger.print(EventLogger::L_INFO,"[feature_map_tracker_test.cpp] INFO: Tracklets: \n");
		file << "[feature_map_tracker_test.cpp] INFO: Tracklets: \n";
		for (size_t i = 0; i < feature_map_tracker.map_tracklets_.size(); i++)
		{
			file << frame_number << " - Tracklet [" << i <<"]" << endl;
			for (size_t j = 0; j < feature_map_tracker.map_tracklets_[i].keypoint_indices_.size(); j++)
			{
				file << feature_map_tracker.map_tracklets_[i].keypoint_indices_[j] << " --> ";
			}
			file << endl;
		}

		file << "----------------------------------------------------------------------------------" << endl;
		frame_number++;
	}

	file.close();

	return 0;
}


void draw_current_features(Mat &img, const vector<Point2f> curr_pts_)
{

	Scalar color;
	color = CV_RGB(0,255,0);
	
	for (size_t i = 0; i < curr_pts_.size(); i++)
	{
		Point2i pt;
		pt.x = curr_pts_[i].x;
		pt.y = curr_pts_[i].y;

		circle(img,pt,2,color,2);
	}
}
