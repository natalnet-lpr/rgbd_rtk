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

#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/video.hpp>

#include <kltatcw_tracker.h>

using namespace std;
using namespace cv;

//#define DEBUG

/*
 * Utility function: returns true if pt is inside any circular window of tracked_points.
 * Each circular window j has radius radiuses[j]. 
 *
 */
bool is_inside_any_window(const vector<Point2f> tracked_pts, const Point2f pt, const vector<float> radiuses)
{
	float r;
	for(size_t j = 0; j < tracked_pts.size(); j++)
	{
		r = sqrt((tracked_pts[j].x - pt.x)*(tracked_pts[j].x - pt.x) + 
			     (tracked_pts[j].y - pt.y)*(tracked_pts[j].y - pt.y));
		if(r <= radiuses[j])
		{
			return true;
		}
	}	
	return false;
}

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

void KLTATCWTracker::detect_keypoints()
{
	//Detect Shi-Tomasi keypoints and add them to a temporary buffer.
	//The buffer is erased at the end of add_keypoints()
	goodFeaturesToTrack(curr_frame_gray_, added_pts_, max_pts_, 0.01, 10.0, Mat(), 3, false, 0.04);
	#ifdef DEBUG
	printf("detecting keypoints...\n");
	printf("\tdetected pts.: %lu\n", added_pts_.size());
	#endif
}

void KLTATCWTracker::add_keypoints()
{
	rejected_points_.clear();

	size_t i = 0;
	//If there are no points in the tracker, detect and add points as normally
	if(prev_pts_.empty())
	{
		for(size_t i = 0; i < added_pts_.size(); i++)
		{
			prev_pts_.push_back(added_pts_[i]);

			//Create and add tracklet
			Tracklet tr(frame_idx_);
			tr.pts2D_.push_back(added_pts_[i]);
			tracklets_.push_back(tr);
			radiuses_.push_back(1);
		}
	}
	else
	{
		//Increase/decrease radiuses of circular windows
		update_tracking_circles();

		while(prev_pts_.size() < max_pts_ && i < added_pts_.size())
		{		
			if(!is_inside_any_window(prev_pts_, added_pts_[i], radiuses_))
			{	
				prev_pts_.push_back(added_pts_[i]);

				//Create and add tracklet
				Tracklet tr(frame_idx_);
				tr.pts2D_.push_back(added_pts_[i]);
				tracklets_.push_back(tr);								
			}
			else
			{
				rejected_points_.push_back(added_pts_[i]);
			}
			i++;
		}
	}
	
	#ifdef DEBUG
	printf("adding keypoints...\n");
	printf("\tadded pts.: %lu\n", added_pts_.size());
	printf("\tprev. pts: %lu\n", prev_pts_.size());
	#endif

	//Erase buffer
	added_pts_.clear();
}

void KLTATCWTracker::update_buffers()
{
	std::swap(curr_pts_, prev_pts_);
}

bool KLTATCWTracker::trigger_keyframe()
{
	float factor = 0.2;
	int Nk = num_points_last_kf_;
	int Nj = curr_pts_.size();
	int dN = abs(Nk - Nj);

	#ifdef DEBUG
	//printf(">>> is %i keyframe? Nk: %i, Nj: %i, dN: %i, k*Nk: %f\n", frame_idx_, Nk, Nj, dN, factor*Nk);
	#endif

	if(dN >= factor*Nk)
	{
		num_points_last_kf_ = Nj;

		return true;
	}

	return false;
}

void KLTATCWTracker::update_tracking_circles()
{	
	#ifdef DEBUG
	printf("\tupdating %lu circles\n", radiuses_.size());
	#endif

	for(size_t i = 0; i < tracklets_.size(); i++)
	{
		//Increase the radius of the tracking circle according to the tracklet size
		//(we could multiply by a scalar factor)
		radiuses_[i] = tracklets_[i].pts2D_.size();	
		if(radiuses_[i] > max_radius_)
		{	
			radiuses_[i] = max_radius_;
		}
		else if(radiuses_[i] < min_radius_)
		{
			radiuses_[i] = min_radius_;	
		}
	}
}

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

KLTATCWTracker::KLTATCWTracker() :
	num_points_last_kf_(0)
{
	//Calls FeatureTracker default constructor

	//Sets default min/max sizes
	min_radius_ = 5;
	max_radius_ = 30;
}

KLTATCWTracker::KLTATCWTracker(const int min_pts, const int max_pts, const float min_r, const float max_r, const bool log_stats) :
	FeatureTracker(min_pts, max_pts, log_stats),
	num_points_last_kf_(0),
	min_radius_(min_r),
	max_radius_(max_r)
{
	
}

bool KLTATCWTracker::track(Mat curr_frame)
{
	//Make a grayscale copy of the current frame
	cvtColor(curr_frame, curr_frame_gray_, CV_BGR2GRAY);

	#ifdef DEBUG
	printf("#### Tracking frame %i ####\n", frame_idx_);
	#endif

	//Swap buffers: prev_pts_ = curr_pts_
	update_buffers();

	//Adds keypoints detected in the previous frame to prev_pts_
	add_keypoints();

	//Tracker is not initialized
	if(!initialized_)
	{
		//Initialize tracker
		detect_keypoints();
		initialized_ = true;
	}
	//Tracker is initialized: track keypoints
	else
	{
		//Track points with Optical Flow
		vector<uchar> status;
		vector<float> err;
		Size win_size(53, 53); //def is 31x31
		TermCriteria crit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);

		calcOpticalFlowPyrLK(prev_frame_gray_, curr_frame_gray_, prev_pts_, curr_pts_, status, err, win_size,
							 3, crit, 0, 0.00001);

		#ifdef DEBUG
		printf("tracking...\n");
		#endif

		//Update internal data according to the tracking result
		//Additional tests have to be applied to discard points outside the image boundaries.
		int tracked_pts = 0;
		for(int i = 0; i < curr_pts_.size(); i++)
		{
			if(!status[i] || curr_pts_[i].x < 0 || curr_pts_[i].y < 0 ||
				curr_pts_[i].x >= prev_frame_gray_.cols || curr_pts_[i].y >= prev_frame_gray_.rows)
			{
				continue;
			}

			#ifdef DEBUG
			//printf("\t\ttracked[%i]: (%f,%f) -> (%f,%f)\n", i, prev_pts_[i].x, prev_pts_[i].y, curr_pts_[i].x, curr_pts_[i].y);
			#endif

			prev_pts_[tracked_pts] = prev_pts_[i];
			curr_pts_[tracked_pts] = curr_pts_[i];
			tracklets_[tracked_pts] = tracklets_[i];

			tracklets_[tracked_pts].pts2D_.push_back(curr_pts_[i]);

			tracked_pts++;
		}
		prev_pts_.resize(tracked_pts);
		curr_pts_.resize(tracked_pts);
		tracklets_.resize(tracked_pts);
		radiuses_.resize(tracked_pts);
		#ifdef DEBUG
		printf("\ttracked points/max_points: %i/%i\n", tracked_pts, max_pts_);
		#endif

		//Detect new features at every frame, hold them and add them to the tracker in the next frame
		detect_keypoints();
	}

	//print_track_info();
	//write_tracking_info();
	//write_timing_info();
	//write_heatmap_info();
	//float total_time = get_time_per_frame();

	#ifdef DEBUG
	//printf("time per frame: %f ms\n", total_time);
	#endif

	cv::swap(curr_frame_gray_, prev_frame_gray_);
	frame_idx_++;

	return trigger_keyframe();
}