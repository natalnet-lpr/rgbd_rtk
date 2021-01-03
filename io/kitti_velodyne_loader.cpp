/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Natalnet Laboratory for Perceptual Robotics
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

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <iterator>
#include <algorithm>
#include <opencv2/core/core.hpp>

#include <event_logger.h>
#include <kitti_velodyne_loader.h>

using namespace std;
using namespace cv;


KITTIVelodyneLoader::KITTIVelodyneLoader()
{
    next_ = 0;
    num_scans_ = 0;
	cloud_xyzi_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	cloud_xyzrgb_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void KITTIVelodyneLoader::loadLIDARSequence(const string& sequence_path, const int& sequence_num)
{
    int exec_status;
    char tmp[3];
    string seq_number_str, cmd, index_path;

    //set the root directory (common to all LIDAR data)
    root_path_ = sequence_path;

    //set the sequence number
    sprintf(tmp, "%02d", sequence_num);
    seq_number_str = string(tmp);

    //set the prefix path
    prefix_path_ = root_path_ + "sequences/" + seq_number_str;

    cmd = "ls " + prefix_path_ + "/velodyne >"
                + prefix_path_ + "/velodyne/index.txt";
    index_path = prefix_path_ + "/velodyne/index.txt";

	exec_status = system(cmd.c_str());

    if(exec_status == 0)
    {
        //Read file and insert each entry into the vector of strings
        ifstream myfile(index_path.c_str()); 
        copy(istream_iterator<string>(myfile),
             istream_iterator<string>(),
             back_inserter(lidar_scan_names_));
        myfile.close();

        lidar_scan_names_.erase(lidar_scan_names_.begin() + lidar_scan_names_.size() - 1);

        num_scans_ = lidar_scan_names_.size();
    }
    else
    {
        logger.print(EventLogger::L_ERROR, "[kitti_velodyne_loader.cpp] ERROR: Could not create index file (error on system command).\nExiting\n");
        exit(0);
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KITTIVelodyneLoader::getPointCloudXYZI()
{
    string path;

	cloud_xyzi_->clear();

    path = prefix_path_ + "/velodyne/" + lidar_scan_names_[next_];
	next_++;

	fstream input(path.c_str(), ios::in | ios::binary);

	for(size_t i = 0; input.good() && !input.eof(); i++)
    {
		pcl::PointXYZI point;
		input.read((char *) &point.x, sizeof(float));
		input.read((char *) &point.y, sizeof(float));
		input.read((char *) &point.z, sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		cloud_xyzi_->push_back(point);
	}
	input.close();
	
	return cloud_xyzi_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr KITTIVelodyneLoader::getPointCloudXYZRGB()
{
    string path;

    cloud_xyzrgb_->clear();

    path = prefix_path_ + "/velodyne/" + lidar_scan_names_[next_];
    next_++;

    fstream input(path.c_str(), ios::in | ios::binary);

    for(size_t i = 0; input.good() && !input.eof(); i++)
    {
        pcl::PointXYZRGB point;
        float trash_i;
        input.read((char *) &point.x, sizeof(float));
        input.read((char *) &point.y, sizeof(float));
        input.read((char *) &point.z, sizeof(float));
        input.read((char *) &trash_i, sizeof(float));

        point.r = 0;
        point.g = 0;
        point.b = 255;

        cloud_xyzrgb_->push_back(point);
    }
    input.close();
    
    return cloud_xyzrgb_;
}

KITTIVelodyneLoader::~KITTIVelodyneLoader()
{

}






