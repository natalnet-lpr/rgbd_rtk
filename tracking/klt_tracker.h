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

#ifndef INCLUDE_KLT_TRACKER_H_
#define INCLUDE_KLT_TRACKER_H_

#include <vector>
#include <fstream>
#include <opencv2/core/core.hpp>

#include <common_types.h>

/*
 * Short Baseline Feature Tracker default implementation:
 * Kanade-Lucas-Tomasi (KLT) tracker using
 * OpenCV (Bouguet's) sparse multiscale optical flow.
 *
 * Author: Bruno Marques F. da Silva
 * brunomfs@gmail.com
 */
class KLTTracker
{

protected:

	//Flag used to inform if the tracker was initialized
	bool initialized_;

	//Current frame number
	int frame_idx_;

	//Previous greyscale frame
	cv::Mat prev_frame_gray_;

	//Current greyscale frame
	cv::Mat curr_frame_gray_;

	//Minimum allowed number of tracked points
	int min_pts_;

	//Maximum allowed number of tracked points
	int max_pts_;

	//(REMOVE) Count of inliers in the current frame
	int num_inliers_;

	//Time spent processing each operation (detection, tracking, motion estimation, etc.)
	std::vector<float> op_time_;

	//Output file with tracking information
	std::ofstream tracking_info_;

	//Output file with timing information
	std::ofstream timing_info_;

	//Output file with keypoint distribution information
	std::ofstream heatmap_info_;

	//Detects keypoints in the current frame
	virtual void detect_keypoints();

	//Adds keypoints detected in the previous frame to the tracker
	virtual void add_keypoints();

	//Updates internal buffers (updates the previous points (in the current frame)
	//with the current points (from the previous frame))
	virtual void update_buffers();

	//Returns the average track length
	virtual float get_avg_track_length();

	//Prints all data associated with the tracked keypoints
	virtual void print_track_info();

	//Sum the time spent in each operation and return the total time computing a single frame
	float get_time_per_frame();

	//Write tracking information (n. of points, inliers, etc.) to file
	virtual bool write_tracking_info();

	//Write timing information (ms spent in each operation) to file
	virtual bool write_timing_info();

	//Write all current keypoints to file
	virtual bool write_heatmap_info();

public:

	//Tracklets: history of each point as a vector of point2f
	std::vector<Tracklet> tracklets_;

	//Previous positions of the tracked points
	std::vector<cv::Point2f> prev_pts_;
	
	//Current positions of the tracked points
	std::vector<cv::Point2f> curr_pts_;

	//Points added in the previous frame
	std::vector<cv::Point2f> added_pts_;

	//Tells if a point is inlier or not
	std::vector<int> is_inlier_;

	//Default constructor
	KLTTracker();

	//Constructor with the minimum and maximum number of tracked points
	KLTTracker(int min_pts, int max_pts);

	virtual ~KLTTracker()
	{
		tracking_info_.close();
		timing_info_.close();
		heatmap_info_.close();
	}

	/*
	 * Main member function: tracks keypoints between the current frame and the previous.
	 * Returns true if the current frame is a keyframe.
	 */
	virtual bool track(cv::Mat img);

	/* Sets the output files for tracking and timing information.
	 * If not called, the system will not write any information to file.
	 */
	void initialize_logger(const std::string timing_file_name,
		                   const std::string tracking_file_name,
			               const std::string heatmap_file_name);
};

#endif /* INCLUDE_KLT_TRACKER_H_ */