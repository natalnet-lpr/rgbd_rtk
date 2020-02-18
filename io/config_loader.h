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

#ifndef INCLUDE_CONFIG_LOADER_H_
#define INCLUDE_CONFIG_LOADER_H_

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <event_logger.h>

#include <opencv2/opencv.hpp>

using namespace std;

class ConfigLoader
{
public:
    cv::FileStorage fs;
    /**
     * Load configuration file.
     * Uses the default path "../config_files/tum_odometry.yaml"
     */
    ConfigLoader();
    /**
     * Load configuration file.
     * @Params filename: path where the config. file is located
     */
    ConfigLoader(string filename){
        loadFile(filename);
    }
    void loadFile(const string& filename);
    /**
    * get a int parameter in ConfigFile
    * @Params string with the name of the parameter in configfile, and a variable int where the value will be returned
    * @Return boolean, false if the parameter is not in configfile and true if it is
    */
    bool checkAndGetInt(const string& parameter, int& parameter_int);
    /**
     * get a int parameter in ConfigFile
     * @Params string with the name of the parameter in configfile, and a variable float where the value will be returned
     * @Return boolean, false if the parameter is not in configfile and true if it is
     */
    bool checkAndGetFloat(const string& parameter, float& parameter_float);
    /**
     * get a int parameter in ConfigFile
     * @Params string with the name of the parameter in configfile, and a variable string where the value will be returned
     * @Return boolean, false if the parameter is not in configfile and true if it is
     */
    bool checkAndGetString(const string& parameter, string& parameter_string);
};

#endif
