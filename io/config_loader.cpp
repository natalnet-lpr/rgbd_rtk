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
        fs["ransac_distance_threshold"] >> ransac_distance_threshold_;
        fs["ransac_inliers_ratio"] >> ransac_inliers_ratio_;
        fs["icp_radius"] >> icp_radius_;
        fs["icp_max_correspondence_distance"] >> icp_max_correspondence_distance_;
        fs["icp_maximum_iteration"] >> icp_maximum_iteration_;
        fs["icp_transformation_epsilon"] >> icp_transformation_epsilon_;
        fs["icp_euclidean_fitness_epsilon"] >> icp_euclidean_fitness_epsilon_;
        //Check if any of the params was not loaded
        if(camera_calibration_file_.empty()) throw 1;
        if(index_file_.empty()) throw 2;
        if(aruco_dic_.empty()) throw 3;
        if(aruco_max_distance_ == 0) throw 4;
        if(aruco_marker_size_ == 0) throw 5;
        if(ransac_distance_threshold_ == 0) throw 6;
        if(ransac_inliers_ratio_ == 0) throw 7;
        if(icp_radius_ == 0) throw 8;
        if(icp_max_correspondence_distance_ == 0) throw 9;
        if(icp_maximum_iteration_ == 0) throw 10;
        if(icp_transformation_epsilon_ == 0) throw 11;
        if(icp_euclidean_fitness_epsilon_ == 0) throw 12;
       
    }
    //If a param was not loaded, use the default
    catch(int e)
    {
        cout << "Couldn't load params\n";
        switch (e)
        {
            case 1:
                cout << "calibration_file is empty\nTrying to use the default path: ../config_files/kinect_default.yaml\n";
                camera_calibration_file_ = "../config_files/kinect_default.yaml";
                break;
            case 2: 
                cout << "index_file is empty\nTrying to use the default: index.txt\n";
                index_file_ = "index.txt";       
                break;
            case 3:
                cout << "aruco_dic is empty\nTrying to use the default dictionary: ARUCO\n";
                aruco_dic_ = "ARUCO";
                break;
            case 4:
                cout << "aruco_max_distance is 0\nTrying to use the default value: 4\n";
                aruco_max_distance_ = 4;
                break;
            case 5:
                cout << "aruco_marker_size is 0\nTrying to use the default value: 0.1778\n";
                aruco_marker_size_ = 0.1778;
                break;
            case 6:
                cout << "ransac_distance_threshold is 0\nTrying to use the default value: 0.008\n";
                ransac_distance_threshold_ = 0.008;
                break;
            case 7:
                cout << "ransac_inlierts_ratio is 0\nTrying to use the default value: 0.8\n";
                ransac_inliers_ratio_ = 0.8;
                break;
            case 8:
                cout << "icp_radius is 0\nTrying to use the default value: 0.05\n";
                icp_radius_ = 0.05;
                break;
            case 9:
                cout << "icp_max_correspondence_distance is 0\nTrying to use the default value: 0.1\n";
                icp_max_correspondence_distance_ = 0.1;
                break;
            case 10:
                cout << "icp_maximum_iteration is 0\nTrying to use the default value: 50\n";
                icp_maximum_iteration_ = 50;
                break;
            case 11:
                cout << "icp_transformation_epsilon is 0\nTrying to use the default value: 1e-9\n";
                icp_transformation_epsilon_ = 0.001;
                break;
            case 12:
                cout << "icp_euclidean_fitness_epsilon is 0\nTrying to use the default value: 0.001\n";
                icp_euclidean_fitness_epsilon_ = 0.001;
                break;

        }
    }
}