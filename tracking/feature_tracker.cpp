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
 *  Bruno Silva
 */

#include <cstdio>
#include <string>

#include <event_logger.h>
#include <feature_tracker.h>

using namespace std;

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

float FeatureTracker::get_avg_track_length()
{
	const size_t N = tracklets_.size();

	size_t sum = 0;
	for(size_t i = 0; i < N; i++)
	{
		sum += (size_t) tracklets_[i].pts2D_.size();
	}

	return float(sum)/N;
}

void FeatureTracker::print_track_info()
{
	const size_t N = tracklets_.size();

	for(size_t i = 0; i < N; i++)
	{
		const size_t M = tracklets_[i].pts2D_.size();

		printf("track[%lu]: (size: %lu)\n\t", i, M);
		for(size_t j = 0; j < M; j++)
		{
			printf("[%lu](%f,%f)->", tracklets_[i].start_ + j, tracklets_[i].pts2D_[j].x, tracklets_[i].pts2D_[j].y);
		}
		printf("\n");
	}
}

float FeatureTracker::get_time_per_frame()
{
	float total_time = 0.0;
	for(size_t i = 0; i < op_time_.size(); i++)
	{
		total_time += op_time_[i];
	}

	return total_time;
}

bool FeatureTracker::write_tracking_info()
{
	if(tracking_info_.is_open())
	{
		if(frame_idx_ == 0)
		{
			//Add comment to the first line of file
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

bool FeatureTracker::write_timing_info()
{
	if(timing_info_.is_open())
	{
		if(frame_idx_ == 0)
		{
			//Add comment to the first line of file
			timing_info_ << "#tracking_time ransac_time detection_time\n";
		}

		timing_info_ << op_time_[0] << " " << op_time_[1] << " " << op_time_[2] << "\n";
	}

	return timing_info_.good();
}

bool FeatureTracker::write_heatmap_info()
{
	if(heatmap_info_.is_open())
	{
		if(frame_idx_ == 0)
		{
			//Add comment to the first line of file
			heatmap_info_ << "#(kp0_x, kp0_y), (kp1_x, kp1_y),..., (kpMi_x, kpMi_y)\n";
		}

		for(size_t i = 0; i < curr_pts_.size(); i++)
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

FeatureTracker::FeatureTracker():
	initialized_(false),
	frame_idx_(0),
	min_pts_(600),
	max_pts_(5000),
	num_inliers_(0)
{

}

FeatureTracker::FeatureTracker(const int& min_pts, const int& max_pts, const bool& log_stats):
	initialized_(false),
	frame_idx_(0),
	min_pts_(min_pts),
	max_pts_(max_pts),
	num_inliers_(0)
{
	if(log_stats)
	{
		initialize_logger("timing_stats.txt", "tracking_stats.txt", "heatmap_stats.txt");
	}
}

void FeatureTracker::initialize_logger(const string& timing_file_name, const string& tracking_file_name,
		                               const string& heatmap_file_name)
{
	timing_info_.open(timing_file_name.c_str());
	if(!timing_info_.is_open())
	{
		MLOG_ERROR(EventLogger::M_TRACKING, "@FeatureTracker::initialize_logger: \
			                                there is a problem with the supplied \
			                                file for the timing information.\n");
		exit(-1);
	}

	tracking_info_.open(tracking_file_name.c_str());
	if(!tracking_info_.is_open())
	{
		MLOG_ERROR(EventLogger::M_TRACKING, "@FeatureTracker::initialize_logger: \
			                                there is a problem with the supplied \
			                                file for the tracking information.\n");
		exit(-1);
	}

	heatmap_info_.open(heatmap_file_name.c_str());
	if(!heatmap_info_.is_open())
	{
		MLOG_ERROR(EventLogger::M_TRACKING, "@FeatureTracker::initialize_logger: \
			                                there is a problem with the supplied \
			                                file for the heatmap information.\n");
		exit(-1);
	}

	MLOG_INFO(EventLogger::M_TRACKING, "@FeatureTracker::initialize_logger: saving \
		                               tracking information to %s\n",
		                               tracking_file_name.c_str());
	MLOG_INFO(EventLogger::M_TRACKING, "@FeatureTracker::initialize_logger: saving \
		                               timing information to %s\n",
		                               timing_file_name.c_str());
	MLOG_INFO(EventLogger::M_TRACKING, "@FeatureTracker::initialize_logger: saving \
		                               heatmap information to %s\n",
		                               heatmap_file_name.c_str());
}

FeatureTracker::TRACKER_TYPE FeatureTracker::strToType(const std::string &s)
{
	MLOG_DEBUG(EventLogger::M_TRACKING,
               "@FeatureTracker::strToType "
               "returning constant for %s\n",
               s.c_str());

	if(s == "KLTTracker")
		return FeatureTracker::TRACKER_TYPE::TRACKER_KLT;
	else if(s == "KLTTWTracker")
		return FeatureTracker::TRACKER_TYPE::TRACKER_KLTTW;
	else if(s == "WBTracker")
		return FeatureTracker::TRACKER_TYPE::TRACKER_WIDEBASELINE;
	else
		return FeatureTracker::TRACKER_TYPE::TRACKER_UNKNOWN;
}