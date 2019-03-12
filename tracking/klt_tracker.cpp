/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Natalnet Laboratory for Perceptual Robotics
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
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/video.hpp>

#include <klt_tracker.h>

using namespace std;
using namespace cv;

//#define DEBUG

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

void KLTTracker::detect_keypoints()
{
	//Detect Shi-Tomasi keypoints and add them to a temporary buffer.
	//The buffer is erased at the end of add_keypoints()
	goodFeaturesToTrack(curr_frame_gray_, added_pts_, max_pts_, 0.01, 10.0, Mat(), 3, false, 0.04);
	#ifdef DEBUG
	printf("detecting keypoints...\n");
	printf("\tdetected pts.: %lu\n", added_pts_.size());
	#endif
}

void KLTTracker::add_keypoints()
{
	for(size_t i = 0; i < added_pts_.size(); i++)
	{
		prev_pts_.push_back(added_pts_[i]);

		//Create and add tracklet
		Tracklet tr(frame_idx_);
		tr.pts2D_.push_back(added_pts_[i]);
		tracklets_.push_back(tr);
	}
	#ifdef DEBUG
	printf("adding keypoints...\n");
	printf("\tadded pts.: %lu\n", added_pts_.size());
	printf("\tprev. pts: %lu\n", prev_pts_.size());
	#endif

	//Erase buffer
	added_pts_.clear();
}

void KLTTracker::update_buffers()
{
	std::swap(curr_pts_, prev_pts_);
	add_keypoints();
}

float KLTTracker::get_avg_track_length()
{
	const int N = tracklets_.size();

	int sum = 0;
	for(int i = 0; i < N; i++)
	{
		sum += (int) tracklets_[i].pts2D_.size();
	}

	return ((float) sum)/N;
}

void KLTTracker::print_track_info()
{
	const size_t N = tracklets_.size();

	for(int i = 0; i < N; i++)
	{
		const size_t M = tracklets_[i].pts2D_.size();

		printf("track[%i]: (size: %lu)\n\t", i, M);
		for(size_t j = 0; j < M; j++)
		{
			printf("[%lu](%f,%f)->", tracklets_[i].start_ + j, tracklets_[i].pts2D_[j].x, tracklets_[i].pts2D_[j].y);
		}
		printf("\n");
	}
}

float KLTTracker::get_time_per_frame()
{
	float total_time = 0.0;
	for(int i = 0; i < op_time_.size(); i++)
	{
		total_time += op_time_[i];
	}

	return total_time;
}

bool KLTTracker::write_tracking_info()
{
	if(tracking_info_.is_open())
	{
		if(frame_idx_ == 0)
		{
			tracking_info_ << "#keypoints_at_motion added_pts inliers\n";
			tracking_info_ << 0 << " " << curr_pts_.size() << " " << 0 << "\n";
		}
		else
		{
			tracking_info_ << (curr_pts_.size() - added_pts_.size()) << " " << added_pts_.size() << " " << num_inliers_ << "\n";
		}
	}

	return tracking_info_.good();
}

bool KLTTracker::write_timing_info()
{
	if(timing_info_.is_open())
	{
		if(frame_idx_ == 0)
		{
			timing_info_ << "#tracking_time ransac_time detection_time\n";
		}

		timing_info_ << op_time_[0] << " " << op_time_[1] << " " << op_time_[2] << "\n";
	}

	return timing_info_.good();
}

bool KLTTracker::write_heatmap_info()
{
	if(heatmap_info_.is_open())
	{
		if(frame_idx_ == 0)
		{
			heatmap_info_ << "#(kp0_x, kp0_y), (kp1_x, kp1_y),..., (kpMi_x, kpMi_y)\n";
		}

		for(int i = 0; i < curr_pts_.size(); i++)
		{
			heatmap_info_ << "(" << (int) curr_pts_[i].x << "," << (int) curr_pts_[i].y << ") ";
		}
		heatmap_info_ << "\n";
	}

	return heatmap_info_.good();
}

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

KLTTracker::KLTTracker() : min_pts_(600), max_pts_(5000)
{
	initialized_ = false;
	frame_idx_ = 0;
	num_inliers_ = 0;
}

KLTTracker::KLTTracker(int min_pts, int max_pts)
{
	initialized_ = false;
	frame_idx_ = 0;
	num_inliers_ = 0;

	min_pts_ = min_pts;
	max_pts_ = max_pts;
}

bool KLTTracker::track(Mat curr_frame)
{
	//Make a grayscale copy of the current frame
	//printf("curr frame:%i , curr_frame_gray:%i ",curr_frame.channels(),curr_frame_gray_.channels());
	cvtColor(curr_frame, curr_frame_gray_, CV_BGR2GRAY);

	#ifdef DEBUG
	printf("#### Tracking frame %i ####\n", frame_idx_);
	#endif

	//Update internal buffers
	update_buffers();

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

			//printf("\t\ttracked[%i]: (%f,%f) -> (%f,%f)\n", i, prev_pts_[i].x, prev_pts_[i].y, curr_pts_[i].x, curr_pts_[i].y);

			prev_pts_[tracked_pts] = prev_pts_[i];
			curr_pts_[tracked_pts] = curr_pts_[i];
			tracklets_[tracked_pts] = tracklets_[i];

			tracklets_[tracked_pts].pts2D_.push_back(curr_pts_[i]);

			tracked_pts++;
		}
		prev_pts_.resize(tracked_pts);
		curr_pts_.resize(tracked_pts);
		tracklets_.resize(tracked_pts);
		#ifdef DEBUG
		printf("\ttracked points/max_points: %i/%i\n", tracked_pts, max_pts_);
		#endif

		//Insufficient number of points being tracked
		if(tracked_pts < min_pts_)
		{
			//Detect new features, hold them and add them to the tracker in the next frame
			detect_keypoints();
		}
	}

	//print_track_info();
	//write_tracking_info();
	//write_timing_info();
	//write_heatmap_info();
	//float total_time = get_time_per_frame();

	#ifdef DEBUG
	printf("\tcurr. pts: %lu\n", curr_pts_.size());
	//printf("time per frame: %f ms\n", total_time);
	#endif

	cv::swap(curr_frame_gray_, prev_frame_gray_);
	frame_idx_++;

	return true;
}

void KLTTracker::initialize_logger(const string timing_file_name, const string tracking_file_name,
		                            const string heatmap_file_name)
{
	timing_info_.open(timing_file_name.c_str());
	if(!timing_info_.is_open())
	{
		fprintf(stderr, "There is a problem with the supplied file for the timing information.\n");
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}

	tracking_info_.open(tracking_file_name.c_str());
	if(!tracking_info_.is_open())
	{
		fprintf(stderr, "There is a problem with the supplied file for the tracking information.\n");
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}

	heatmap_info_.open(heatmap_file_name.c_str());
	if(!heatmap_info_.is_open())
	{
		fprintf(stderr, "There is a problem with the supplied file for the heatmap information.\n");
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}

	printf("Saving tracking information to %s\n", tracking_file_name.c_str());
	printf("Saving timing information to %s\n", timing_file_name.c_str());
	printf("Saving heatmap information to %s\n", heatmap_file_name.c_str());
}