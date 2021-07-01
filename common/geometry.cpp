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
 *  Rodrigo Sarmento Xavier
 */

#include <cmath>

#include <geometry.h>
#include <event_logger.h>

using namespace std;
using namespace cv;

bool is_valid(const PointT& p)
{
	if(isnan(p.x) || isnan(p.y) || isnan(p.z))
	{
		return false;
	}
	else
	{
		return true;
	}
}

PointT get3Dfrom2D(const Point2f& point, const pcl::PointCloud<PointT>::Ptr& dense_cloud)
{
	//Grab 2D coord. of the feature
	float x = point.x;
	float y = point.y;

	//Grab the integer (pixel) indices of the point
	int j = int(std::floor(x));
	int i = int(std::floor(y));

	//Grab the 3D coordinate associated with the point
	float X = dense_cloud->at(j,i).x;
	float Y = dense_cloud->at(j,i).y;
	float Z = dense_cloud->at(j,i).z;
		
	PointT pt;
	pt.x = X;
	pt.y = Y;
	pt.z = Z;
	return pt;
}

pcl::PointCloud<PointT> getPointCloud(const Mat& rgb, const Mat& depth, const Intrinsics& intr)
{
	float fx = intr.fx_, fy = intr.fy_;
	float cx = intr.cx_, cy = intr.cy_;
	float f = intr.scale_; //factor used in the depth data

	pcl::PointCloud<PointT> cloud;
	cloud.width = depth.cols;
	cloud.height = depth.rows;
	cloud.is_dense = false;
	cloud.resize(cloud.width*cloud.height);

	for(int i = 0; i < depth.rows; i++)
	{
		for(int j = 0; j < depth.cols; j++)
		{
			float Z = depth.at<unsigned short int>(i,j)/f;
			float X = (j - cx)*Z/fx;
			float Y = (i - cy)*Z/fy;

			PointT pt;

			//Set RGB values
			uint b = rgb.at<cv::Vec3b>(i,j)[0];
			uint g = rgb.at<cv::Vec3b>(i,j)[1];
			uint r = rgb.at<cv::Vec3b>(i,j)[2];
			pt.r = r; pt.g = g; pt.b = b;

			//Invalid coordinate: set to NaN for PCL compatibility
			if(Z == 0)
			{
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
			}
			//Set 3D point coordinates
			else
			{
				pt.x = X;
				pt.y = Y;
				pt.z = Z;
			}

			cloud.at(j,i) = pt;
		}
	}

	return cloud;
}

Eigen::Affine3f newPoseOffset(const Eigen::Affine3f& pose, const float& offset_distance)
{/* This function gets a 3d pose and an offset distance, this offset will be added as a distance
  updating the pose to a new pose with the same orientation but a new position in space.
 */  
	Eigen::Vector4f F = Eigen::Vector4f(); // Distance between aruco and the 3D point we want
	Eigen::Affine3f resultado = Eigen::Affine3f::Identity();// Marker pose resultado
	Eigen::Vector4f V = Eigen::Vector4f(); // 3D point pose 
	F(0,0) = 0.0;
	F(1,0) = 0.0;
	F(2,0) = offset_distance;
	F(3,0) = 1.0;

	//In order to get a new position related to a pose, we should make the operatioin with the pose
	//in the left and the point vector at the right.

		
	V = pose * F;  //Find the xyz position of the new 3d pose 
	resultado(0,3) = V(0,0); resultado(1,3) = V(1,0); resultado(2,3) = V(2,0); //Setting the last column

	return resultado;
}

double distanceBetween(const Eigen::Affine3f &first_rf, const Eigen::Affine3f& second_rf)
{
    double x0 = first_rf(0,3);
    double y0 = first_rf(1,3);
    double z0 = first_rf(2,3);
    double x1 = second_rf(0,3);
    double y1 = second_rf(1,3);
    double z1 = second_rf(2,3);

    return sqrt((x0 - x1)*(x0 - x1) + (y0 - y1)*(y0 - y1) + (z0 - z1)*(z0 - z1));
}

bool zAxisAngleBetween(const Eigen::Affine3f& first_rf, const Eigen::Affine3f& second_rf)
{
	double angle = acos((first_rf(0,2) * second_rf(0,2)) +
		                (first_rf(1,2) * second_rf(1,2)) +
		                (first_rf(2,2) * second_rf(2,2))) * 180.0/M_PI;

    return angle;
}

Eigen::Isometry3d affineToIsometry(const Eigen::Affine3f &m)
{
    Eigen::Isometry3d r;
    r.linear() = m.linear().cast<double>();
    r.translation() = m.translation().cast<double>();
    return r;
}

Eigen::Affine3f relativeTransform(const Eigen::Affine3f &from_pose, const Eigen::Affine3f &to_pose)
{
	return from_pose.inverse()*to_pose;
}

Eigen::Isometry3d relativeTransform(const Eigen::Isometry3d &from_pose, const Eigen::Isometry3d &to_pose)
{
	return from_pose.inverse()*to_pose;
}

void printTransform(const Eigen::Affine3f& t)
{
	MLOG_INFO(EventLogger::M_COMMON, "%f, %f. %f, %f\n",
              t(0,0), t(0,1), t(0,2), t(0,3));
	MLOG_INFO(EventLogger::M_COMMON, "%f, %f. %f, %f\n",
              t(1,0), t(1,1), t(1,2), t(1,3));
	MLOG_INFO(EventLogger::M_COMMON, "%f, %f. %f, %f\n",
              t(2,0), t(2,1), t(2,2), t(2,3));
	MLOG_INFO(EventLogger::M_COMMON, "%f, %f. %f, %f\n",
              t(3,0), t(3,1), t(3,2), t(3,3));
}

void printTransform(const Eigen::Isometry3d& t)
{
	MLOG_INFO(EventLogger::M_COMMON, "%f, %f. %f, %f\n",
              t(0,0), t(0,1), t(0,2), t(0,3));
	MLOG_INFO(EventLogger::M_COMMON, "%f, %f. %f, %f\n",
              t(1,0), t(1,1), t(1,2), t(1,3));
	MLOG_INFO(EventLogger::M_COMMON, "%f, %f. %f, %f\n",
              t(2,0), t(2,1), t(2,2), t(2,3));
	MLOG_INFO(EventLogger::M_COMMON, "%f, %f. %f, %f\n",
              t(3,0), t(3,1), t(3,2), t(3,3));	
}