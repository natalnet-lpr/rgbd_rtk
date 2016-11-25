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

#include <geometry.h>

using namespace std;
using namespace cv;

bool is_valid(PointT p)
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

PointT get3Dfrom2D(const Point2f point, const pcl::PointCloud<PointT>::Ptr dense_cloud)
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

pcl::PointCloud<PointT> getPointCloud(const Mat rgb, const Mat depth, const Intrinsics intr)
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