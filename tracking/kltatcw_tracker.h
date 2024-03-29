/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2019, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or
 *     promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Authors:
 *
 *  Luiz Felipe Maciel Correia (y9luiz@hotmail.com)
 *  Bruno Silva (brunomfs@gmail.com)
 */

#ifndef INCLUDE_KLTATCW_TRACKER_H_
#define INCLUDE_KLTATCW_TRACKER_H_

#include <fstream>
#include <opencv2/core/core.hpp>
#include <vector>

#include <common_types.h>
#include <feature_tracker.h>

/*
* Short Baseline Feature Tracker extension:
* Kanade-Lucas-Tomasi (KLT) with Adaptive Tracking Circular Windows.
*/
class KLTATCWTracker : public FeatureTracker
{

protected:
    // Minimum radius of a tracking circular window
    float min_radius_;

    // Maximum radius of a tracking circular window
    float max_radius_;

    // Number of points in the last keyframe
    size_t num_points_last_kf_;

    /**
     * Detects keypoints in the current frame
     */
    void detect_keypoints(const cv::Mat& mask);

    /**
     * Adds keypoints detected in the previous frame to the tracker
     */
    void add_keypoints();

    /**
     * Updates internal buffers (updates the previous points (in the current frame)
     * With the current points (from the previous frame))
     */
    void update_buffers();

    /**
     * @return true if the current frame is a keyframe
     */
    bool trigger_keyframe();

    /**
     * Updates the radiuses of tracking circles
     */
    void update_tracking_circles();

public:
    // Debug
    std::vector<cv::Point2f> rejected_points_;

    // Radius of each circular window
    std::vector<float> radiuses_;

    // Default constructor
    KLTATCWTracker();

    /**
     * Constructor
     * @param min_pts minimum number of tracked points
     * @param max_pts maximum number of tracked points
     * @param log_stats boolean for log statistics state
     */
    KLTATCWTracker(const int &min_pts, const int &max_pts, const float &min_r, const float &max_r,
                   const bool &log_stats = false);

    /**
     * Main member function: tracks keypoints between the current frame and the previous.
     * @param img rgb image
     * @return true if the current frame is a keyframe.
     */
    bool track(const cv::Mat &img, const cv::Mat& mask = cv::Mat());
};

#endif /* INCLUDE_KLTATCW_TRACKER_H_ */