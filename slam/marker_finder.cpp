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

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "marker_finder.h"

using namespace std;
using namespace cv;
using namespace aruco;

void MarkerFinder::setMarkerPoses(const Eigen::Affine3f& cam_pose, const float& aruco_max_distance)
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
		if(aruco_max_distance == -1) //infinite
		{
			marker_poses_.push_back(cam_pose.inverse() *P);
		}
		else if(sqrt(x + y + z) < aruco_max_distance) //marker is closer than the max distance
		{
			marker_poses_.push_back(cam_pose.inverse() * P);
		}
		else //marker is further than the max distance
		{
			markers_.clear();
			continue;
		}
	}
}
void MarkerFinder::setMarkerPointPoses(const Eigen::Affine3f& cam_pose, const float& aruco_max_distance, const float& aruco_close_distance)
{/* This function save the marker pose where the robot need to go.
 It's the same aruco pose but with a value added in order to the robot always find a place inside of the map
 In Some situations the aruco marker can be detected outside of the map, since it is oftenly
 placed in a wall(Precision erros can place the aruco marker outside of the map)
 */  
	marker_point_poses_.clear();
	for(size_t i = 0; i < markers_.size(); i++)
	{
		Mat R = Mat::eye(3, 3, CV_32FC1); // Orientation 
		Eigen::Affine3f P = Eigen::Affine3f::Identity();// Marker pose
		Eigen::Affine3f resultado = Eigen::Affine3f::Identity();// Marker pose resultado
		Eigen::Vector4f F = Eigen::Vector4f(); // Distance between aruco and the 3D point we want
		Eigen::Vector4f V = Eigen::Vector4f(); // 3D point pose 
		double x=0,y=0,z=0;
		F(0,0) = 0.0;
		F(1,0) = 0.0;
		F(2,0) = aruco_close_distance;
		F(3,0) = 1.0;

		Rodrigues(markers_[i].Rvec, R);
	
		P(0,0) = R.at<float>(0,0); P(0,1) = R.at<float>(0,1); P(0,2) = R.at<float>(0,2);
		P(1,0) = R.at<float>(1,0); P(1,1) = R.at<float>(1,1); P(1,2) = R.at<float>(1,2);
		P(2,0) = R.at<float>(2,0); P(2,1) = R.at<float>(2,1); P(2,2) = R.at<float>(2,2);
		P(0,3) = markers_[i].Tvec.at<float>(0,0); P(1,3) = markers_[i].Tvec.at<float>(1,0); P(2,3) = markers_[i].Tvec.at<float>(2,0);

		x = pow(P(0,3),2);
		y = pow(P(1,3),2);
		z = pow(P(2,3),2);

		V = P * F;  //Find a vector pose in the Aruco ref frame
		V = cam_pose.inverse() * V; //Find a vector pose in global ref frame
		resultado = cam_pose.inverse() * P; //Find the Rotation matrix
		resultado(0,3) = V(0,0); resultado(1,3) = V(1,0); resultado(2,3) = V(2,0); //Setting the last column

		//getting the absolute distance between camera and marker
		//if their distance is closer then aruco_max_distance save marker pose
		if(aruco_max_distance == -1){//infinite
			marker_point_poses_.push_back(resultado);
		}
		else if(sqrt(x + y + z) < aruco_max_distance){//aruco is closer than the minimum distance
			marker_point_poses_.push_back(resultado);  
		}
		else {//aruco is further than the minimum distance
			markers_.clear();
			continue;
		}
	}
}
void MarkerFinder::markerParam(const string& params, const float& size, const string& aruco_dic)
{//Load params 
	marker_detector_.setDictionary(aruco_dic,0);
	camera_params_.readFromXMLFile(params);
	marker_size_ = size;
}

void MarkerFinder::detectMarkersPoses(const cv::Mat& img, const Eigen::Affine3f& cam_pose, const float& aruco_max_distance)
{
	markers_.clear();
	marker_detector_.detect(img, markers_, camera_params_, marker_size_); //detec markers
	
	setMarkerPoses(cam_pose, aruco_max_distance); //set the pose
}

void MarkerFinder::detectMarkersPointPoses(const cv::Mat& img, const Eigen::Affine3f& cam_pose, const float& aruco_max_distance, const float& aruco_close_distance)
{
	markers_.clear();
	marker_detector_.detect(img, markers_, camera_params_, marker_size_); //detec markers
	
    setMarkerPointPoses(cam_pose, aruco_max_distance, aruco_close_distance); //set the pose
}
