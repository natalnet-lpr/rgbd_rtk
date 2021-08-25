/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2021, Natalnet Laboratory for Perceptual Robotics
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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Authors:
 *
 *  Bruno Silva
 *  Rodrigo Xavier
 */

#ifndef INCLUDE_MARKER_FINDER_H_
#define INCLUDE_MARKER_FINDER_H_

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <aruco/aruco.h>

/*
 * Artificial marker finder, used to detect loops
 * on controlled (equipped with artificial markers) environments.
 *
 */
class MarkerFinder
{

protected:
    //(ARUCO) Marker detector
    aruco::MarkerDetector marker_detector_;

    // Size of each artificial marker
    float marker_size_;

    /**
     * Set the pose of all detected markers w.r.t. the global ref. frame
     * @param camera pose as affine3f
     * @param aruco_max_distance minimum distance to aruco be considered valid
     */
    void setMarkerPoses(const Eigen::Affine3f& cam_pose, const float& aruco_max_distance);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //Utility struct to aid loading parameters from config. files
    struct Parameters
    {
        std::string calib_file_; //file name of the camera intrinsic parameters
        float marker_size_; //size of the ARUCO marker
        float max_marker_dist_; //maximum distance to consider a valid marker detection
        std::string aruco_dict_; //name of the ARUCO dictionary to be used to detect markers
    };

    //(ARUCO) Camera intrinsic parameters
    aruco::CameraParameters camera_params_;

    // Vector with each detected marker
    std::vector<aruco::Marker> markers_;

    // Vector with the pose of each detected marker
    std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>> marker_poses_;

    /** 
     * Constructs a marker detector with
     * default parameters.
     */
    MarkerFinder(){}

    /** 
     * Constructs a marker detector with
     * given camera calibration file, marker size
     * and marker dictionary.
     * @param calib_file_name: name of the camera calibration file
     * @param marker_size: size of the AR marker
     * @param aruco_dict: dictionary for the ARUCO markers
     */
    MarkerFinder(const std::string &calib_file, const float &marker_size,
                 const std::string &aruco_dict)
    {
        setParameters(calib_file, marker_size, aruco_dict);
    }

    /**
     * Sets the supplied camera calibration file, marker size
     * and marker dictionary.
     * @param calib_file_name: name of the camera calibration file
     * @param marker_size: size of the AR marker
     * @param aruco_dict: dictionary for the ARUCO markers
     */
    void setParameters(const std::string &calib_file, const float &marker_size,
                       const std::string &aruco_dict);

    /**
     * Detects ARUCO markers and their
     * respectives poses in the reference
     * frame of the camera
     * (just pass a identity matrix to cam_pose if you
     * want to detect the pose of the marker related to the camera)
     * @param rgb image: which the markers should be detected
     * @param cam_pose: current camera pose (in world ref. frame)
     * @param aruco_max_distance: max distance aruco will be saved
     */
    void detectMarkersPoses(const cv::Mat& img,
                            const Eigen::Affine3f& cam_pose,
                            const float& aruco_max_distance);

    /**
     * Returns true if a marker with given
     * id was found.
     * Must call detectMarkersPoses first.
     * @param id: id of marker to be checked.
     */
    bool isMarkerFound(const int &id);

    /**
     * Returns the pose of a marker with given
     * id in the reference frame of the camera.
     * Must call detectMarkersPoses first.
     * @param id: id of marker to be checked.
     * @return pose of marker with given id
     */
    Eigen::Affine3f markerPose(const int &id);

    /**
     * Draws the pose of a marker with given
     * id to the supplied image.
     * @param id: id of marker to be drawn.
     * @param img: RGB image in which the marker should be drawn
     */
    void drawMarker(const int &id, cv::Mat &img);
};
#endif /* INCLUDE_MARKER_FINDER_H_ */
