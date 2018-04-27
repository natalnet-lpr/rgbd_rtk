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

#ifndef INCLUDE_MARKER_FINDER_H_
#define INCLUDE_MARKER_FINDER_H_

#include <vector>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

#include <aruco/aruco.h>

/*
 * Artificial marker finder, used to detect loops
 * on controlled (equiped with artificial markers) environments.
 *
 */
class MarkerFinder
{

protected:
	
	//(ARUCO) Marker detector
    aruco::MarkerDetector marker_detector_;
	
	//Size of each artificial marker
	float marker_size_;
		
	//Set the pose of all detected markers w.r.t. the local/camera ref. frame
	void setMarkerPosesLocal();
	
	//Set the pose of all detected markers w.r.t. the global ref. frame
	void setMarkerPosesGlobal(Eigen::Affine3f cam_pose);

public:
	
	//(ARUCO) Camera intrinsic parameters
	aruco::CameraParameters camera_params_;
	
	//Vector with each detected marker
	std::vector<aruco::Marker> markers_;
	
	//Vector with the pose of each detected marker (w.r.t. the local/camera ref. frame)
	std::vector<Eigen::Affine3f> marker_poses_local_;
	
	//Vector with the pose of each detected marker 
	std::vector<Eigen::Affine3f> marker_poses_;
	
	//Default constructor
	MarkerFinder();
	
	//Constructor with camera intrinsic parameters and marker size
	MarkerFinder(char params[], float size);

	//Detect ARUCO markers. Also sets the poses of all detected markers in the local and global ref. frames
	void detectMarkers(const cv::Mat img, Eigen::Affine3f cam_pose);

};

#endif /* INCLUDE_MARKER_FINDER_H_ */