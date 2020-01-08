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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rgbd_loader.h>
#include <config_loader.h>
#include <event_logger.h>
#include <klt_tracker.h>
#include <common_types.h>

using namespace std;
using namespace cv;

void draw_last_track(Mat& img, const vector<Point2f> prev_pts, const vector<Point2f> curr_pts, bool is_kf);
void draw_tracks(Mat& img, const vector<Tracklet> tracklets);

/**
 * This program shows the use of keypoint tracking KLT algorithm.
 * @param .yml config. file (from which index_file is used)
 */
int main(int argc, char **argv)
{
	EventLogger& logger = EventLogger::getInstance();
	logger.setVerbosityLevel(pcl::console::L_INFO);

	RGBDLoader loader;
	int min_pts, max_pts, log_stats;
	Mat frame, depth;
	string index_file;

	if(argc != 2)
	{
		logger.print(pcl::console::L_INFO, "[klt_tracker_test.cpp] Usage: %s <path/to/config_file.yaml>\n", argv[0]);
		exit(0);
	}
	ConfigLoader param_loader(argv[1]);
	param_loader.checkAndGetInt("min_pts",min_pts);
	param_loader.checkAndGetInt("max_pts",max_pts);	
	param_loader.checkAndGetInt("log_stats", log_stats);
	param_loader.checkAndGetString("index_file", index_file);
	KLTTracker tracker(min_pts,max_pts,log_stats);
	loader.processFile(index_file);

	//Track points on each image
	for(int i = 0; i < loader.num_images_; i++)
	{
		loader.getNextImage(frame, depth);

		double el_time = (double) cvGetTickCount();
		bool is_kf = tracker.track(frame);
		el_time = ((double) cvGetTickCount() - el_time)/(cvGetTickFrequency()*1000.0);
		logger.print(pcl::console::L_INFO,"[klt_tracker_test.cpp] INFO: Tracking time: %f ms\n", el_time);
		
		draw_last_track(frame, tracker.prev_pts_, tracker.curr_pts_, is_kf);
		//draw_tracks(frame, tracker.tracklets_);

		imshow("Image view", frame);
		imshow("Depth view", depth);
		char key = waitKey(15);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			logger.print(pcl::console::L_INFO, "[klt_tracker_test.cpp] Exiting\n", argv[0]);
			break;
		}
	}

	return 0;
}

void draw_last_track(Mat& img, const vector<Point2f> prev_pts, const vector<Point2f> curr_pts, bool is_kf)
{
	Scalar color;
	if(is_kf)
	{
		color = CV_RGB(255, 0, 0);
	}
	else
	{
		color = CV_RGB(0, 255, 0);
	}
	for(size_t k = 0; k < curr_pts.size(); k++)
	{
		Point2i pt1, pt2;
		pt1.x = prev_pts[k].x;
		pt1.y = prev_pts[k].y;
		pt2.x = curr_pts[k].x;
		pt2.y = curr_pts[k].y;

		circle(img, pt1, 1, color, 2);
		circle(img, pt2, 3, color, 2);
		line(img, pt1, pt2, color);
	}
}
void draw_tracks(Mat& img, const vector<Tracklet> tracklets)
{
	for(size_t i = 0; i < tracklets.size(); i++)
	{
		for(size_t j = 0; j < tracklets[i].pts2D_.size(); j++)
		{
			Point2i pt1;
			pt1.x = tracklets[i].pts2D_[j].x;
			pt1.y = tracklets[i].pts2D_[j].y;
			circle(img, pt1, 3, CV_RGB(0,255,0), 1);
			if(j > 0)
			{
				Point2i pt2;
				pt2.x = tracklets[i].pts2D_[j-1].x;
				pt2.y = tracklets[i].pts2D_[j-1].y;
				line(img, pt1, pt2, CV_RGB(0,255,0));
			}
		}
	}
}