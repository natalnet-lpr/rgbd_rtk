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

#ifndef STEREO_CLOUDGENERATOR_H
#define STEREO_CLOUDGENERATOR_H

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <pcl/point_cloud.h>

#include <common_types.h>

class StereoCloudGenerator
{

private:

    // Stereo block matching algorithm
    cv::Ptr<cv::StereoSGBM> stereo_bm_;

    // Number of image channels (used for grayscale, color images)
    int num_channels_;

    // Parameters used in the point cloud generation
    int min_disparity_;
    int block_size_;
    int disp12_max_diff_;
    int pre_filter_cap_;
    int uniqueness_ratio_;
    int speckle_window_size_;
    int speckle_range_;

    // Perspective transform matrix
    cv::Mat Q_;

public:

    // Disparity image
    cv::Mat disparity_;

    // Generated point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    /**
     * @param intr matrix of intrinsic parameters
     * @param num_channels numbers of image channels (1 or 3)
     */
	StereoCloudGenerator(const Intrinsics& intr, const int& num_channels)
    {
        cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        num_channels_ = num_channels;

        // Stereo configuration (default values: used in KITTI visual odometry)
        min_disparity_ = 0;
        block_size_ = 11;
        disp12_max_diff_ = 1;
        pre_filter_cap_ = 63;
        uniqueness_ratio_ = 15;
        speckle_window_size_ = 200;
        speckle_range_ = 2;

        int num_disparities = 64 - min_disparity_;
        int P1 = 8*num_channels_*block_size_*block_size_;
        int P2 = 32*num_channels_*block_size_*block_size_;

        stereo_bm_ = cv::StereoSGBM::create(
            min_disparity_, num_disparities, block_size_, 
            P1, P2, disp12_max_diff_, pre_filter_cap_,  
            uniqueness_ratio_, speckle_window_size_, 
            speckle_range_, cv::StereoSGBM::MODE_SGBM_3WAY
        );

        Q_ = (cv::Mat_<float>(4,4) << 1, 0, 0, -intr.cx_, 
                                      0, 1, 0, -intr.cy_,
                                      0, 0, 0,  intr.fx_,
                                      0, 0, 1/intr.baseline_, 0);
    }

    /**
     * Set parameters used in disparity map generation
     * @param min_disparity @param block_size
     * @param disp12_max_diff @param pre_filter_cap
     * @param uniqueness_ratio @param speckle_window_size
     * @param speckle_range
     */
    void setParameters(const int& min_disparity, const int& block_size,
                       const int& disp12_max_diff, const int& pre_filter_cap,
                       const int& uniqueness_ratio, const int& speckle_window_size,
                       const int& speckle_range);

    /**
     * Generate disparity map from calibrated image pair
     * (the result is stored in the disparity_ attribute)
     * @param left_img @param right_img 
     */
	void generateDisparityMap(const cv::Mat& left_img, const cv::Mat& right_img);
                               

    /**
     * Generate 3D point cloud from calibrated image pair
     * (the result is stored in the cloud_ attribute)
     * @param left_img @param right_img 
     */
	void generatePointCloud(const cv::Mat& left_img, const cv::Mat& right_img); 
	
    /**
     * Default destructor
     */
    ~StereoCloudGenerator()
    {
        cloud_->clear();
    }
};

#endif //STEREO_CLOUDGENERATOR_H
