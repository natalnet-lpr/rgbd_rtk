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

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "marker_finder.h"

using namespace std;
using namespace cv;
using namespace aruco;

void MarkerFinder::setMarkerPosesLocal(float aruco_minimum_distance)
{	
	double x=0,y=0,z=0;
	marker_poses_local_.clear();
	for(size_t i = 0; i < markers_.size(); i++)
	{
		Mat R = Mat::eye(3, 3, CV_32FC1);
		Eigen::Affine3f P = Eigen::Affine3f::Identity();
		
		Rodrigues(markers_[i].Rvec, R);
	
		P(0,0) = R.at<float>(0,0); P(0,1) = R.at<float>(0,1); P(0,2) = R.at<float>(0,2);
		P(1,0) = R.at<float>(1,0); P(1,1) = R.at<float>(1,1); P(1,2) = R.at<float>(1,2);
		P(2,0) = R.at<float>(2,0); P(2,1) = R.at<float>(2,1); P(2,2) = R.at<float>(2,2);
		P(0,3) = markers_[i].Tvec.at<float>(0,0); P(1,3) = markers_[i].Tvec.at<float>(1,0); P(2,3) = markers_[i].Tvec.at<float>(2,0);
		
		x = pow(P(0,3),2);
		y = pow(P(1,3),2);
		z = pow(P(2,3),2);

		//getting the absolute distance between camera and marker
		///if their distance is closer then  aruco_max_distance meters save marker pose
		if(aruco_minimum_distance == -1){//infinite
			marker_poses_local_.push_back(P);
			continue ;
		}
		if(sqrt(x + y + z) < aruco_minimum_distance){
			marker_poses_local_.push_back(P);  //aruco is closer than the minimum distance
			continue;
		}
		if(sqrt(x +y +z) >= aruco_minimum_distance){ //aruco is further than the minimum distance
			continue;
		}
	}
}

void MarkerFinder::setMarkerPosesGlobal(Eigen::Affine3f cam_pose, float aruco_minimum_distance)
{
	double x=0,y=0,z=0;
	marker_poses_.clear();
	for(size_t i = 0; i < markers_.size(); i++)
	{
		Mat R = Mat::eye(3, 3, CV_32FC1);
		Eigen::Affine3f P = Eigen::Affine3f::Identity();
		
		Rodrigues(markers_[i].Rvec, R);
	
		P(0,0) = R.at<float>(0,0); P(0,1) = R.at<float>(0,1); P(0,2) = R.at<float>(0,2);
		P(1,0) = R.at<float>(1,0); P(1,1) = R.at<float>(1,1); P(1,2) = R.at<float>(1,2);
		P(2,0) = R.at<float>(2,0); P(2,1) = R.at<float>(2,1); P(2,2) = R.at<float>(2,2);
		P(0,3) = markers_[i].Tvec.at<float>(0,0); P(1,3) = markers_[i].Tvec.at<float>(1,0); P(2,3) = markers_[i].Tvec.at<float>(2,0);
		
		x = pow(P(0,3),2);
		y = pow(P(1,3),2);
		z = pow(P(2,3),2);
		
		//getting the absolute distance between camera and marker
		///if their distance is closer then  aruco_max_distance meters save marker pose
		if(aruco_minimum_distance == -1){//infinite
			marker_poses_.push_back(cam_pose.inverse() *P );
			continue ;
		}
		if(sqrt(x + y + z) < aruco_minimum_distance){
			marker_poses_.push_back(cam_pose.inverse() * P);  //aruco is closer than the minimum distance
			continue;
		}
		if(sqrt(x +y +z) >= aruco_minimum_distance){ //aruco is further than the minimum distance
			continue;
		}
	}
}

MarkerFinder::MarkerFinder()
{
	marker_detector_.setDictionary("ARUCO_MIP_36h12", 0);
}

MarkerFinder::MarkerFinder(string params, float size)
{
	marker_detector_.setDictionary("ARUCO_MIP_36h12", 0);
	camera_params_.readFromXMLFile(params);
	marker_size_ = size;
}

void MarkerFinder::detectMarkers(const cv::Mat img, Eigen::Affine3f cam_pose, float aruco_minimum_distance)
{
	markers_.clear();
	marker_detector_.detect(img, markers_, camera_params_, marker_size_);
	
	setMarkerPosesLocal(aruco_minimum_distance);
	setMarkerPosesGlobal(cam_pose, aruco_minimum_distance);
}