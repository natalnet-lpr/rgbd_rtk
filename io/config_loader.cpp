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
 *  Rodrigo Sarmento Xavier
 *  Bruno Silva
 */

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>


#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

#include <config_loader.h>
#include <event_logger.h>

using namespace std;
using namespace cv;

void ConfigLoader::loadFile(const string& filename)
{

    fs.open(filename, FileStorage::READ);  //Reading config file
    if(fs.isOpened() == false)
    {
        logger.print(pcl::console::L_ERROR, "[config_loader.cpp] ERROR: ConfigFile couldn't be opened, check if your path is right\n");
        exit(0);
    }
}

bool ConfigLoader::checkAndGetInt(const string& parameter, int& parameter_int){
    if(fs[parameter].empty()){
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    } 
    else{
        fs[parameter] >> parameter_int;
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %i\n", parameter.c_str(), parameter_int);
        return true;
    }
}

bool ConfigLoader::checkAndGetFloat(const string& parameter, float& parameter_float){
    if(fs[parameter].empty()){
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    }     
    else{
        fs[parameter] >> parameter_float;
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %f\n", parameter.c_str(), parameter_float);
        return true;
    }
}

bool ConfigLoader::checkAndGetString(const string& parameter, string& parameter_string){
    if(fs[parameter].empty()){
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    }     
    else{
        fs[parameter] >> parameter_string;
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %s\n", parameter.c_str(), parameter_string.c_str());
        return true;
    }
}

bool ConfigLoader::checkAndGetFeatureDetector(const string& parameter, Ptr<cv::FeatureDetector>& feature_detector){
     if(fs[parameter].empty()){
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    }     
    else{
        string feature_detector_text;
        fs[parameter] >> feature_detector_text;
        boost::to_upper(feature_detector_text);
        
        if (feature_detector_text == "ORB"){
            feature_detector = cv::ORB::create();
        }
        else if (feature_detector_text == "AKAZE"){
            feature_detector = cv::AKAZE::create();
        }
        else{
            logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, insert a valid feature detector\n", feature_detector_text.c_str());
            return false;
        }
    
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %s\n", parameter.c_str(), feature_detector_text.c_str());
        return true;
    }
}

bool ConfigLoader::checkAndGetDescriptorExtractor(const string& parameter, Ptr<cv::DescriptorExtractor>& descriptor_extractor){
     if(fs[parameter].empty()){
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    }     
    else{
        string descriptor_extractor_text;
        fs[parameter] >> descriptor_extractor_text;
        boost::to_upper(descriptor_extractor_text);
        
        if (descriptor_extractor_text == "ORB"){
            descriptor_extractor = cv::ORB::create();
        }
        else if (descriptor_extractor_text == "AKAZE"){
            descriptor_extractor = cv::AKAZE::create();
        }
        else{
            logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, insert a valid descriptor extractor\n", descriptor_extractor_text.c_str());
            return false;
        }
    
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %s\n", parameter.c_str(), descriptor_extractor_text.c_str());
        return true;
    }
}

bool ConfigLoader::checkAndGetDescriptorMatcher(const string& parameter, Ptr<cv::DescriptorMatcher>& descriptor_matcher){
     if(fs[parameter].empty()){
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    }     
    else{
        string descriptor_matcher_text;
        fs[parameter] >> descriptor_matcher_text;
        boost::to_upper(descriptor_matcher_text);
        
        if (descriptor_matcher_text == "BRUTEFORCE"){
            descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
        }
        else{
            logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, insert a valid descriptor mstcher\n", descriptor_matcher_text.c_str());
            return false;
        }
    
        logger.print(pcl::console::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %s\n", parameter.c_str(), descriptor_matcher_text.c_str());
        return true;
    }
}