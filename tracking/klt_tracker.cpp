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
 *  Authors:
 *
 *  Bruno Silva
 */

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/video.hpp>

#include <event_logger.h>
#include <klt_tracker.h>

using namespace std;
using namespace cv;

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

void KLTTracker::detect_keypoints(const cv::Mat& mask)
{
    // Detect Shi-Tomasi keypoints and add them to a temporary buffer.
    // The buffer is erased at the end of add_keypoints()
    goodFeaturesToTrack(curr_frame_gray_, added_pts_, max_pts_, 0.01, 10.0, mask, 3, false, 0.04);
    
    MLOG_DEBUG(EventLogger::M_TRACKING, "@KLTTracker::detect_keypoints: detecting keypoints...\n");
    MLOG_DEBUG(EventLogger::M_TRACKING, "@KLTTracker::detect_keypoints: detected pts.: %lu\n",
               added_pts_.size());
}

void KLTTracker::add_keypoints()
{
    for (size_t i = 0; i < added_pts_.size(); i++)
    {
        prev_pts_.push_back(added_pts_[i]);

        // Create and add tracklet
        Tracklet tr(frame_idx_);
        tr.pts2D_.push_back(added_pts_[i]);
        tracklets_.push_back(tr);
    }
    
    MLOG_DEBUG(EventLogger::M_TRACKING, "@KLTTracker::add_keypoints: added pts.: %lu\n",
               added_pts_.size());
    MLOG_DEBUG(EventLogger::M_TRACKING, "@KLTTracker::add_keypoints: prev pts.: %lu\n",
               prev_pts_.size());

    // Erase buffer
    added_pts_.clear();
}

void KLTTracker::update_buffers() {
    std::swap(curr_pts_, prev_pts_);
}

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

KLTTracker::KLTTracker()
{
    // Calls FeatureTracker default constructor
}

KLTTracker::KLTTracker(const int &min_pts, const int &max_pts, const bool &log_stats)
    : FeatureTracker(min_pts, max_pts, log_stats)
{
}

KLTTracker::KLTTracker(const FeatureTracker::Parameters& param):
    FeatureTracker(param.min_pts_, param.max_pts_, param.log_stats_)
{
}

bool KLTTracker::track(const Mat &curr_frame, const cv::Mat& mask)
{
    bool is_keyframe = false;

    // Make a grayscale copy of the current frame if it is in color
    if (curr_frame.channels() > 1)
    {
        #if CV_MAJOR_VERSION < 4
            cvtColor(curr_frame, curr_frame_gray_, CV_BGR2GRAY);
        #else
            cvtColor(curr_frame, curr_frame_gray_, COLOR_BGR2GRAY);
        #endif
    }
    else
        curr_frame.copyTo(curr_frame_gray_);

    MLOG_DEBUG(EventLogger::M_TRACKING, "@KLTTracker::track: tracking frame %i\n",
               frame_idx_);

    // Swap buffers: prev_pts_ = curr_pts_
    update_buffers();

    // Adds keypoints detected in the previous frame to prev_pts_
    add_keypoints();

    // Tracker is not initialized
    if (!initialized_)
    {
        // Initialize tracker
        detect_keypoints(mask);
        initialized_ = true;
        is_keyframe = true;
    }
    // Tracker is initialized: track keypoints
    else
    {
        // Track points with Optical Flow
        vector<uchar> status;
        vector<float> err;
        Size win_size(53, 53); // def is 31x31

        #if CV_MAJOR_VERSION < 4
            TermCriteria crit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
        #else
            TermCriteria crit(TermCriteria::MAX_ITER | TermCriteria::EPS, 20, 0.03);
        #endif

        calcOpticalFlowPyrLK(prev_frame_gray_, curr_frame_gray_, prev_pts_, curr_pts_, status, err,
                             win_size, 3, crit, 0, 0.00001);

        // Update internal data according to the tracking result
        // Additional tests have to be applied to discard points outside the image boundaries.
        int tracked_pts = 0;
        int val = -1;
        for (size_t i = 0; i < curr_pts_.size(); i++)
        {
            if (!mask.empty())
                val = (int) mask.at<uchar>(curr_pts_[i].y,curr_pts_[i].x);

            if (!status[i] || curr_pts_[i].x < 0 || curr_pts_[i].y < 0 ||
                curr_pts_[i].x >= prev_frame_gray_.cols || curr_pts_[i].y >= prev_frame_gray_.rows || val == 0)
            {
                continue;
            }

            // logger.print(EventLogger::L_DEBUG, "[KLTTracker::track] DEBUG: tracked[%i]: (%f,%f)
            // -> (%f,%f)\n", i, prev_pts_[i].x, prev_pts_[i].y, curr_pts_[i].x, curr_pts_[i].y);

            prev_pts_[tracked_pts] = prev_pts_[i];
            curr_pts_[tracked_pts] = curr_pts_[i];
            tracklets_[tracked_pts] = tracklets_[i];

            tracklets_[tracked_pts].pts2D_.push_back(curr_pts_[i]);

            tracked_pts++;
        }
        prev_pts_.resize(tracked_pts);
        curr_pts_.resize(tracked_pts);
        tracklets_.resize(tracked_pts);
        
        MLOG_DEBUG(EventLogger::M_TRACKING, "@KLTTracker::track: \
                                             tracked points/max_points:  %i/%i\n",
                                             tracked_pts, max_pts_);

        // Insufficient number of points being tracked
        if (tracked_pts < min_pts_)
        {
            // Detect new features, hold them and add them to the tracker in the next frame
            detect_keypoints(mask);
            // Make the current frame a new keyframe
            is_keyframe = true;
        }
    }

    // print_track_info();
    // write_tracking_info();
    // write_timing_info();
    // write_heatmap_info();
    // float total_time = get_time_per_frame();

    MLOG_DEBUG(EventLogger::M_TRACKING, "@KLTTracker::track: curr_points:  %lu\n",
               curr_pts_.size());

    cv::swap(curr_frame_gray_, prev_frame_gray_);
    frame_idx_++;

    return is_keyframe;
}