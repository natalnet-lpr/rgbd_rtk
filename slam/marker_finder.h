/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2019, Natalnet Laboratory for Perceptual Robotics
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
 *  Rodrigo Sarmento Xavier
 *  Bruno Silva
 */

#ifndef INCLUDE_MARKER_FINDER_H_
#define INCLUDE_MARKER_FINDER_H_

#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

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
	
	//Size of each artificial marker
	float marker_size_;
	
	/**
	 * Set the pose of all detected markers w.r.t. the local/camera ref. frame
	 * @param minimum distance(camera to marker) to aruco be considered valid
	 */	
	void setMarkerPosesLocal(float aruco_max_distance);
	
	/**
	 * Set the pose of all detected markers w.r.t. the global ref. frame
	 * @param camera pose as affine3f and minimum distance to aruco be considered valid
	 */	
	void setMarkerPosesGlobal(const Eigen::Affine3f& cam_pose, const float& aruco_max_distance);

public:
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//(ARUCO) Camera intrinsic parameters
	aruco::CameraParameters camera_params_;
	
	//Vector with each detected marker
	std::vector<aruco::Marker> markers_;
	
	//Vector with the pose of each detected marker (w.r.t. the local/camera ref. frame)
	std::vector<Eigen::Affine3f,Eigen::aligned_allocator<Eigen::Affine3f> > marker_poses_local_;
	
	//Vector with the pose of each detected marker 
	std::vector<Eigen::Affine3f,Eigen::aligned_allocator<Eigen::Affine3f> > marker_poses_;
	
	//Default constructor
	MarkerFinder();
	/**
	 * Constructor
	 * @param camera intrinsic and marker size
	 */
	MarkerFinder(string params, float size);

	void detectMarkers(const cv::Mat img, Eigen::Affine3f cam_pose, float aruco_minimum_distance);
	/**
	 * Detect ARUCO markers. Also sets the poses of all detected markers in the local and global ref. frames
	 * @param rgb image, camera pose, and  aruco max distance
	 */	
	void detectMarkers(const cv::Mat img, const Eigen::Affine3f& cam_pose, const float& aruco_max_distance);
};

#endif /* INCLUDE_MARKER_FINDER_H_ */