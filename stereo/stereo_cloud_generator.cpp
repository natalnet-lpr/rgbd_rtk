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
 *  Felipe Ferreira Barbosa
 *  Vanessa Dantas de Souto Costa
 *  Bruno Silva
 */

#include <limits>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"

#include "stereo_cloud_generator.h"

using namespace cv;

void StereoCloudGenerator::setParameters(const int& min_disparity, const int& block_size,
                                         const int& disp12_max_diff, const int& pre_filter_cap,
                                         const int& uniqueness_ratio, const int& speckle_window_size,
                                         const int& speckle_range)
{
    min_disparity_ = min_disparity;
    block_size_ = block_size;
    disp12_max_diff_ = disp12_max_diff;
    pre_filter_cap_ = pre_filter_cap;
    uniqueness_ratio_ = uniqueness_ratio;
    speckle_window_size_ = speckle_window_size;
    speckle_range_ = speckle_range;
}

void StereoCloudGenerator::generateDisparityMap(const cv::Mat& left_img, const cv::Mat& right_img)
{
    Mat left_gray, right_gray, disparity, true_disparity;

    //Convert images to grey scale if using color images
    if(num_channels_ == 3)
    {
        cvtColor(left_img, left_gray, cv::COLOR_BGR2GRAY);
        cvtColor(right_img, right_gray, cv::COLOR_BGR2GRAY);
        
        //Compute disparity map according to parameters set
        stereo_bm_->compute(left_gray, right_gray, disparity);
    }
    else
    {
        //Compute disparity map according to parameters set
        stereo_bm_->compute(left_img, right_img, disparity);
    }

    //Convert disparity to float
    disparity.convertTo(disparity_, CV_32F, 1.0/16.0, 0.0);
}

void StereoCloudGenerator::generatePointCloud(const cv::Mat& left_img, const cv::Mat& right_img)
{
    Mat mat_cloud;

    //Generate disparity image
    generateDisparityMap(left_img, right_img);

    //Reproject image to 3D using geometric stereo parameters set in matrix Q
    reprojectImageTo3D(disparity_, mat_cloud, Q_, true);

    //Set point cloud properties
    cloud_->clear();
    cloud_->width = mat_cloud.cols;
    cloud_->height = mat_cloud.rows;
    cloud_->is_dense = false;
    cloud_->resize(cloud_->width * cloud_->height);

    //Assemble PCL point cloud
    for(int i = 0; i < mat_cloud.rows; i++)
    {
        for(int j = 0; j < mat_cloud.cols; j++)
        {
            pcl::PointXYZRGB pt;

            //Get 3D info from depth image
            Vec3f point = mat_cloud.at<Vec3f>(i,j);

            //Set point to invalid (NaN) if any coordinate is larger than 10000
            if(point[0] >= 10000)
                pt.x = std::numeric_limits<double>::quiet_NaN();
            else
                pt.x = point[0];

            if(point[1] >= 10000)
                pt.y = std::numeric_limits<double>::quiet_NaN();
            else
                pt.y = point[1];

            if(point[2] >= 10000)
                pt.z = std::numeric_limits<double>::quiet_NaN();
            else
                pt.z = point[2];

            //Get color info from the left image
            if(num_channels_ == 3)
            {
                uint b = left_img.at<Vec3b>(i,j)[0];
                uint g = left_img.at<Vec3b>(i,j)[1];
                uint r = left_img.at<Vec3b>(i,j)[2];
                pt.r = r; pt.g = g; pt.b = b;
            }
            else
            {
                uint gray = left_img.at<uchar>(i,j);
                pt.r = gray;; pt.g = gray; pt.b = gray;
            }
            
            cloud_->at(j,i) = pt;
        }
    }
}
