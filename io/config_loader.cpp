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
 *  Rodrigo Sarmento Xavier
 *  Bruno Silva
 *  Marcos Henrique F. Marcone
 */

#include <cstdio>
#include <cstdlib>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <config_loader.h>
#include <event_logger.h>

using namespace std;
using namespace cv;

void ConfigLoader::loadFile(const string& filename)
{

    fs_.open(filename, FileStorage::READ);  //Reading config file
    if(fs_.isOpened() == false)
    {
        logger.print(EventLogger::L_ERROR, "[config_loader.cpp] ERROR: ConfigFile couldn't be opened, check if your path is right\n");
        exit(0);
    }
}

bool ConfigLoader::checkAndGetInt(const string& parameter, int& parameter_int){
    if(fs_[parameter].empty())
    {
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    } 
    else
    {
        fs_[parameter] >> parameter_int;
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %i\n", parameter.c_str(), parameter_int);
        return true;
    }
}

bool ConfigLoader::checkAndGetFloat(const string& parameter, float& parameter_float){
    if(fs_[parameter].empty())
    {
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    }     
    else
    {
        fs_[parameter] >> parameter_float;
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %f\n", parameter.c_str(), parameter_float);
        return true;
    }
}

bool ConfigLoader::checkAndGetString(const string& parameter, string& parameter_string){
    if(fs_[parameter].empty())
    {
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    }     
    else
    {
        fs_[parameter] >> parameter_string;
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %s\n", parameter.c_str(), parameter_string.c_str());
        return true;
    }
}

bool ConfigLoader::checkAndGetBool(const string& parameter, bool& parameter_bool){
    if(fs_[parameter].empty())
    {
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    }     
    else
    {
        string parameter_string;
        fs_[parameter] >> parameter_string;
        parameter_bool = parameter_string == "true" ? 1 : 0;
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s = %i\n", parameter.c_str(), parameter_bool);
        return true;
    }
}

bool ConfigLoader::checkAndGetMat(const string& parameter, Mat& parameter_mat)
{
    if(fs_[parameter].empty())
    {
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s couldn't be loaded, check your ConfigFile.yaml\n", parameter.c_str());
        return false;
    }     
    else
    {
        fs_[parameter] >> parameter_mat;
        logger.print(EventLogger::L_INFO, "[config_loader.cpp] INFO: Parameter %s loaded.\n", parameter.c_str());
        return true;
    }
}

