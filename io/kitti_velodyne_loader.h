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

#ifndef KITTI_VELODYNE_LOADER_H
#define KITTI_VELODYNE_LOADER_H

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class KITTIVelodyneLoader
{

private:

    //Path of the root directory of the sequence
    std::string root_path_;

    //Path of the root dir. + "/sequences/" + seq_number_;
    std::string prefix_path_;

    //Scan index
    int next_;

    //Point cloud with XYZ and Intensity
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi_;

    //Point cloud with XYZ and a fixed RGB color
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_; 
    
    //Strings of all left images of a sequence
    std::vector<std::string> lidar_scan_names_;

public:

    //Total number of laser scans in the sequence
	int num_scans_;

    //Default constructor
	KITTIVelodyneLoader();

    //Loads a KITTI sequence number with given path
	void loadLIDARSequence(const std::string& sequence_path, const int& sequence_num);

    //Point cloud having XYZ and Intensity
	pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloudXYZI();

    //Point cloud having XYZ and a constant RGB color
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudXYZRGB();

    ~KITTIVelodyneLoader();
};

#endif /* KITTI_VELODYNE_LOADER_H */

