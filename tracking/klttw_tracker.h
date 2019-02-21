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

#ifndef INCLUDE_KLTTW_TRACKER_H_
#define INCLUDE_KLTTW_TRACKER_H_

#include <vector>
#include <fstream>
#include <opencv2/core/core.hpp>

#include <feature_tracker.h>
#include <common_types.h>

 /*
 * Short Baseline Feature Tracker extension:
 * Kanade-Lucas-Tomasi (KLT) with Tracking Windows.
 *
 * Author: Bruno Marques F. da Silva
 * brunomfs@gmail.com
 * Author: Luiz Felipe Maciel Correia
 * y9luiz@hotmail.com
 */
class KLTTWTracker : public FeatureTracker
{

protected:

	//Number of points in the last keyframe
	size_t num_points_last_kf_;

	//Detects keypoints in the current frame
	void detect_keypoints();

	//Adds keypoints detected in the previous frame to the tracker
	void add_keypoints();

	//Updates internal buffers (updates the previous points (in the current frame)
	//with the current points (from the previous frame))
	void update_buffers();

	//Returns true if the current frame is a keyframe
	bool trigger_keyframe();

public:

	//Debug
	std::vector<cv::Point2f> rejected_points_;

	//Size of the rectangular area of each feature when adding new features.
	cv::Size window_size_;

	//Default constructor
	KLTTWTracker();

	//Constructor with the minimum number of tracked points, maximum number of tracked points, size of tracking window and flag to log statistics
	KLTTWTracker(const int min_pts, const int max_pts, const cv::Size sz, const bool log_stats = false);

	/*
	 * Main member function: tracks keypoints between the current frame and the previous.
	 * Returns true if the current frame is a keyframe.
	 */
	bool track(cv::Mat img);
};

#endif /* INCLUDE_KLTTW_TRACKER_H_ */