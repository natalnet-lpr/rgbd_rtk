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
#include <opencv2/highgui/highgui.hpp>

#include <event_logger.h>
#include <kitti_stereo_loader.h>

using namespace cv;
using namespace std;

void KITTIStereoLoader::loadStereoPair(const std::string& left_path, const std::string& right_path)
{
    left_image_ = imread(left_path);
    right_image_ = imread(right_path);
}

void KITTIStereoLoader::loadStereoSequence(const std::string& sequence_path, const int& sequence_num, const bool& use_color)
{
    int exec_status;
    char tmp[3];
    string seq_number_str, cmd, index_path, needed_slash = "";

    //set the root directory (common to all gray/color left/right images)
    root_path_ = sequence_path;

    //set the sequence number
    sprintf(tmp, "%02d", sequence_num);
    seq_number_str = string(tmp);

    //add last '/'' if the path does not have one
    if(sequence_path.back() != '/')
    {
        needed_slash = "/";
    }

    //set the prefix path
    prefix_path_ = root_path_ + needed_slash + seq_number_str;
    MLOG_INFO(EventLogger::M_IO, "Loading KITTI sequence: %s\n", prefix_path_.c_str());

    if(use_color)
    {
        cmd = "ls " + prefix_path_ + "/image_2 >"
                    + prefix_path_ + "/image_2/index.txt";
        index_path = prefix_path_ + "/image_2/index.txt";
    }
    else
    {
        cmd = "ls " + prefix_path_ + "/image_0 >"
                    + prefix_path_ + "/image_0/index.txt";
        index_path = prefix_path_ + "/image_0/index.txt";
    }

    //execute system command (to generate *.txt files)
    exec_status = system(cmd.c_str());

    if(exec_status == 0)
    {
        //Read file and insert each entry into the vector of strings
        ifstream myfile(index_path.c_str()); 
        copy(istream_iterator<string>(myfile),
             istream_iterator<string>(),
             back_inserter(left_image_names_));
        myfile.close();

        left_image_names_.erase(left_image_names_.begin() + left_image_names_.size() - 1);
        right_image_names_ = left_image_names_;

        num_pairs_ = left_image_names_.size();
    }
    else
    {
        MLOG_ERROR(EventLogger::M_IO, "@KITTIStereoLoader::loadStereoSequence: Could not create index file (error on system command).\n");
        exit(0);
    }
}

Mat KITTIStereoLoader::getLeftImage()
{
    return left_image_;
}
 
Mat KITTIStereoLoader::getRightImage()
{
    return right_image_;
}
 
Mat KITTIStereoLoader::getNextLeftImage(const bool& use_color)
{
    string path;
    Mat left_img;

    if(use_color)
    {
        path = prefix_path_ + "/image_2/" + left_image_names_[next_left_];
        left_img = imread(path, IMREAD_COLOR);
    }
    else
    {
        path = prefix_path_ + "/image_0/" + left_image_names_[next_left_];
        left_img = imread(path, CV_LOAD_IMAGE_UNCHANGED);
    }

    next_left_++;
    return left_img;
}

Mat KITTIStereoLoader::getNextRightImage(const bool& use_color)
{
    string path;
    Mat right_img;

    if(use_color)
    {
        path = prefix_path_ + "/image_3/" + right_image_names_[next_right_];
        right_img = imread(path, IMREAD_COLOR);
    }
    else
    {
        path = prefix_path_ + "/image_1/" + right_image_names_[next_right_];
        right_img = imread(path, CV_LOAD_IMAGE_UNCHANGED);
    }

    next_right_++;
    return right_img;
}

KITTIStereoLoader::~KITTIStereoLoader()
{
    left_image_.release();
    right_image_.release();
    root_path_.clear();
    left_image_names_.clear();
    right_image_names_.clear();
}
