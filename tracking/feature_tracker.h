/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Natalnet Laboratory for Perceptual Robotics
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
 *  Author:
 *
 *  Bruno Silva
 */

#ifndef INCLUDE_FEATURE_TRACKER_H_
#define INCLUDE_FEATURE_TRACKER_H_

#include <fstream>
#include <opencv2/core/core.hpp>
#include <vector>

#include <common_types.h>
#include <event_logger.h>

/*
 * Abstract Base Class for Feature Trackers.
 * Contains members that are common to all implementations
 * and virtual member functions that are algorithm dependent.
 */
class FeatureTracker
{

protected:
    // Flag used to inform if the tracker was initialized
    bool initialized_;

    // Current frame number
    int frame_idx_;

    // Previous greyscale frame
    cv::Mat prev_frame_gray_;

    // Current greyscale frame
    cv::Mat curr_frame_gray_;

    // Minimum allowed number of tracked points
    int min_pts_;

    // Maximum allowed number of tracked points
    int max_pts_;

    // Number of inliers in the current frame
    int num_inliers_;

    // Time spent processing each operation (detection, tracking, motion estimation, etc.)
    std::vector<float> op_time_;

    // Output file with tracking information
    std::ofstream tracking_info_;

    // Output file with timing information
    std::ofstream timing_info_;

    // Output file with keypoint distribution information
    std::ofstream heatmap_info_;

    // Returns the average track length
    float get_avg_track_length();

    // Prints all data associated with the tracked keypoints
    void print_track_info();

    // Sum the time spent in each operation and return the total time computing a single frame
    float get_time_per_frame();

    // Writes tracking information (n. of points, inliers, etc.) to file
    bool write_tracking_info();

    // Writes timing information (ms spent in each operation) to file
    bool write_timing_info();

    // Writes all current keypoints to file
    bool write_heatmap_info();

    // Detects keypoints in the current frame
    virtual void detect_keypoints() = 0;

    // Adds keypoints detected in the previous frame to the tracker
    virtual void add_keypoints() = 0;

    /**
     * Updates internal buffers (updates the previous points (in the current frame)
     * with the current points (from the previous frame))
     */
    virtual void update_buffers() = 0;

public:

    //Most commonly used tracker types
    enum TRACKER_TYPE
    {
        TRACKER_UNKNOWN,
        TRACKER_KLT,
        TRACKER_KLTTW,
        TRACKER_WIDEBASELINE
    };

    //Struct having tracker parameters
    struct Parameters
    {
        std::string type_;
        bool log_stats_; //used in all trackers
        int min_pts_; //used in KLT*
        int max_pts_; //used in KLT*
        cv::Size window_size_; //used in KLTTW
        std::string feature_detector_; //used in WideBaselineTracker
        std::string descriptor_extractor_; //used in WideBaselineTracker
        std::string matcher_; //used in WideBaselineTracker
        float detection_threshold_; //used in WideBaselineTracker
    };

    // Tracklets: history of each point as a vector of point2f
    std::vector<Tracklet> tracklets_;

    // Previous positions of the tracked points
    std::vector<cv::Point2f> prev_pts_;

    // Current positions of the tracked points
    std::vector<cv::Point2f> curr_pts_;

    // Points added in the previous frame
    std::vector<cv::Point2f> added_pts_;

    // Tells if a point is inlier or not
    std::vector<int> is_inlier_;

    // Default constructor
    FeatureTracker();

    /**
     * Constructor
     * @param min_pts minimum number of tracked points
     * @param max_pts maximum number of tracked points
     * @param log_stats boolean for log statistics state default is false
     */
    FeatureTracker(const int &min_pts, const int &max_pts, const bool &log_stats = false);

    /**
     * Default Destructor
     */
    virtual ~FeatureTracker()
    {
        tracking_info_.close();
        timing_info_.close();
        heatmap_info_.close();
    }

    /**
     * Sets the output files for tracking and timing information.
     * If not called, the system will not write any information to file.
     * @param timing_file_name file name, @param tracking_file_name file name
     * @param heatmap_file_name file name
     */
    void initialize_logger(const std::string &timing_file_name,
                           const std::string &tracking_file_name,
                           const std::string &heatmap_file_name);

    /**
     * Main member function: tracks keypoints between the current frame and the previous.
     * @param img rgb image
     * @return boolean true if the current frame is a keyframe.
     */
    virtual bool track(const cv::Mat &img) = 0;

    /**
     * Converts a string to its equivalent tracker type
     * @param s string with a tracker type.
     * @return an int constant representing the tracker type
     */
    static FeatureTracker::TRACKER_TYPE strToType(const std::string &s);
};

#endif /* INCLUDE_FEATURE_TRACKER_H_ */
