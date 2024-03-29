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
 *  Rodrigo Xavier
 */

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include <marker_finder.h>
#include <event_logger.h>

using namespace std;
using namespace cv;
using namespace aruco;

void MarkerFinder::setMarkerPoses(const Eigen::Affine3f& cam_pose, const float& aruco_max_distance)
{ /* This function save the marker pose related to the cam_pose, of course, you can always use a
 identity matrix in order to get the pose of the aruco marker assuming that the camera is the
 initial pose. there is an max distance that a marker can be related to the camera in order to be a
 valid marker this is helpfull when you want to decrease error due to detect a marker that is too
 far away.
 */

    double x = 0, y = 0, z = 0;
    marker_poses_.clear();
    for (size_t i = 0; i < markers_.size(); i++)
    {
        Mat R = Mat::eye(3, 3, CV_32FC1);
        Eigen::Affine3f P = Eigen::Affine3f::Identity();

        Rodrigues(markers_[i].Rvec, R);

        P(0, 0) = R.at<float>(0, 0);
        P(0, 1) = R.at<float>(0, 1);
        P(0, 2) = R.at<float>(0, 2);
        P(1, 0) = R.at<float>(1, 0);
        P(1, 1) = R.at<float>(1, 1);
        P(1, 2) = R.at<float>(1, 2);
        P(2, 0) = R.at<float>(2, 0);
        P(2, 1) = R.at<float>(2, 1);
        P(2, 2) = R.at<float>(2, 2);
        P(0, 3) = markers_[i].Tvec.at<float>(0, 0);
        P(1, 3) = markers_[i].Tvec.at<float>(1, 0);
        P(2, 3) = markers_[i].Tvec.at<float>(2, 0);

        x = pow(P(0, 3), 2);
        y = pow(P(1, 3), 2);
        z = pow(P(2, 3), 2);

        /* Getting the absolute distance between camera and marker
        // If their distance is closer then  aruco_max_distance meters save marker pose
        // Multiplying the cam_pose at the left of operation will transform the right pose
        // To the left pose orientation, we add a inverse operation in cam_pose because the
        // way aruco uses orientation.
        */
        // Infinite

        if (aruco_max_distance == -1) marker_poses_.push_back(cam_pose * P);

        // Marker is closer than the max distance
        else if (sqrt(x + y + z) < aruco_max_distance)
            marker_poses_.push_back(cam_pose * P);

        else // Marker is further than the max distance
        {
            markers_.clear();
            continue;
        }
    }
}

void MarkerFinder::setParameters(const string &calib_file, const float &marker_size,
                                 const string &aruco_dict)
{
    marker_detector_.setDictionary(aruco_dict, 0);
    camera_params_.readFromXMLFile(calib_file);
    marker_size_ = marker_size;
}

void MarkerFinder::detectMarkersPoses(
    const cv::Mat& img,
    const Eigen::Affine3f& cam_pose,
    const float& aruco_max_distance)
{
    markers_.clear();
    marker_detector_.detect(img, markers_, camera_params_, marker_size_); // detect markers
    MLOG_DEBUG(EventLogger::M_SLAM, "@MarkerFinder::detectMarkersPoses:"
               " detected %lu markers.\n",
               markers_.size());

    setMarkerPoses(cam_pose, aruco_max_distance); // set the pose
}

bool MarkerFinder::isMarkerFound(const int &id)
{
    for(size_t j = 0; j < markers_.size(); j++)
    {
        if(markers_[j].id == id)
        {
            return true;
        }
    }
    MLOG_WARN(EventLogger::M_SLAM, "@MarkerFinder::isMarkerFound: "
              "marker %i not found.\n",
              id);
    return false;
}

Eigen::Affine3f MarkerFinder::markerPose(const int &id)
{
    for(size_t j = 0; j < markers_.size(); j++)
    {
        if(markers_[j].id == id)
        {
            return marker_poses_[j];
        }
    }

    MLOG_WARN(EventLogger::M_SLAM, "@MarkerFinder::markerPose: "
              "marker %i not found. Returning identity pose.\n",
              id);
    return Eigen::Affine3f::Identity();
}

void MarkerFinder::drawMarker(const int &id, cv::Mat &img)
{
    int marker_idx = -1;

    for(size_t j = 0; j < markers_.size(); j++)
    {
        if(markers_[j].id == id)
        {
            marker_idx = j;
        }
    }

    if(marker_idx >= 0)
    {
        markers_[marker_idx].draw(img, cv::Scalar(0, 0, 255), 1.5);
        CvDrawingUtils::draw3dAxis(img, markers_[marker_idx], camera_params_, 2);
    }
    else
    {
        MLOG_WARN(EventLogger::M_SLAM, "@MarkerFinder::isMarkerFound: "
                  "marker %i not found.\n",
                  id);
    }
}