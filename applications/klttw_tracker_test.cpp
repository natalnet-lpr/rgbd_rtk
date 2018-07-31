/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2018, Natalnet Laboratory for Perceptual Robotics
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
 */

#include <cstdio>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rgbd_loader.h>
#include <klttw_tracker.h>
#include <common_types.h>

using namespace std;
using namespace cv;

void draw_last_track(Mat& img, const vector<Point2f> prev_pts, const vector<Point2f> curr_pts, const Size sz, bool is_kf)
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
		Point2i pt1, pt2, pr1, pr2;
		pt1.x = prev_pts[k].x;
		pt1.y = prev_pts[k].y;
		pt2.x = curr_pts[k].x;
		pt2.y = curr_pts[k].y;

		circle(img, pt1, 1, color, 2);
		circle(img, pt2, 3, color, 2);
		line(img, pt1, pt2, color);

		pr1.x = curr_pts[k].x - float(sz.width)/2;
		pr1.y = curr_pts[k].y - float(sz.width)/2;
		pr2.x = curr_pts[k].x + float(sz.height)/2;
		pr2.y = curr_pts[k].y + float(sz.height)/2;
		rectangle(img, pr1, pr2, color);
	}
}

void draw_rejected_points(Mat& img, const vector<Point2f> pts)
{
	for(size_t i = 0; i < pts.size(); i++)
	{
		circle(img, pts[i], 1, CV_RGB(255, 255, 0), 2);
	}
}

int main(int argc, char **argv)
{
	string index_file_name;
	RGBDLoader loader;
	KLTTWTracker tracker;

	Mat frame, depth;

	if(argc != 2)
	{
		fprintf(stderr, "Usage: %s <index file>\n", argv[0]);
		exit(0);
	}

	index_file_name = argv[1];
	loader.processFile(index_file_name);

	//Track points on each image
	for(int i = 0; i < loader.num_images_; i++)
	{
		loader.getNextImage(frame, depth);

		double el_time = (double) cvGetTickCount();
		bool is_kf = tracker.track(frame);
		el_time = ((double) cvGetTickCount() - el_time)/(cvGetTickFrequency()*1000.0);
		printf("Tracking time: %f ms\n", el_time);
		
		draw_last_track(frame, tracker.prev_pts_, tracker.curr_pts_, tracker.window_size_, is_kf);
		draw_rejected_points(frame, tracker.rejected_points_);

		imshow("Image view", frame);
		imshow("Depth view", depth);
		char key = waitKey(15);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}
	}

	return 0;
}