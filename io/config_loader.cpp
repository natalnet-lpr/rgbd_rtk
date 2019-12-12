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

#include <cstdio>
#include <cstdlib>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <config_loader.h>

using namespace std;
using namespace cv;

/**
 * Load configuration file parameters.
 * @Params filename: path where the config. file is located
 */
void ConfigLoader::loadParams(const string& filename)
{
    FileStorage fs(filename,FileStorage::READ);  //Reading config file
    if(fs.isOpened() == false)
    {
        cout << "ConfigFile couldn't be opened, check if your path is right\n";
        exit(0);
    }

    //Loading Params
    try
    {
        fs["camera_calibration_file"] >> camera_calibration_file_;
        fs["index_file"] >> index_file_;
        fs["aruco_dic"] >> aruco_dic_;
        fs["aruco_max_distance"] >> aruco_max_distance_;
        fs["aruco_marker_size"] >> aruco_marker_size_;

        //Check if any of the params was not loaded
        if(camera_calibration_file_.empty() || index_file_.empty() ||aruco_dic_.empty() ||
           aruco_max_distance_ == 0 || aruco_marker_size_ == 0) throw 1;

        cout << "Params load successfully\n" << "camera_calibration_file: " << camera_calibration_file_ << endl;
        cout << "index_file: " << index_file_<<endl << "aruco_dic: " << aruco_dic_<< endl;
        cout << "aruco_max_distance: " << aruco_max_distance_<<endl << "aruco_marker_size: " << aruco_marker_size_ << endl;
    }
    //If a param was not loaded, use the default
    catch(int e)
    {
        cout << "Coudn't load the params, at least one of the param names is wrong\n\n";
        cout << "Using default values\ncamera_calibration_file: \"kinect_default.yaml\"\n";
        cout << "index_file: \"index\"\n";
        cout << "aruco_dic: \"ARUCO\"\n";
        cout << "aruco_max_distance: 4\n";
        cout << "aruco_marker_size: 0.1778\n";
        camera_calibration_file_ = "../kinect_default.yaml";
        index_file_ = "index";
        aruco_dic_ = "ARUCO";
        aruco_max_distance_ = 4;
        aruco_marker_size_ = 0.1778;
    }
}