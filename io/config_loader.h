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

#ifndef INCLUDE_CONFIG_LOADER_H_
#define INCLUDE_CONFIG_LOADER_H_

#include <cstdio>
#include <cstdlib>
#include <event_logger.h>
#include <fstream>

#include <opencv2/opencv.hpp>

using namespace std;

class ConfigLoader
{
public:

    /**
     * Load configuration file.
     * Uses the default path "../config_files/tum_odometry.yaml"
     */
    ConfigLoader();

    /**
     * Load configuration file.
     * @param filename: path where the config. file is located
     */
    ConfigLoader(string filename) { loadFile(filename); }

    void loadFile(const string &filename);

    /**
    * Get a int parameter in ConfigFile
    * @param parameter with the name of the parameter in configfile
    * @param parameter_int int where the value will be returned
    * @return boolean, false if the parameter is not in configfile and true if it is
    */
    bool checkAndGetInt(const string &parameter, int &parameter_int);

    /**
     * Get a float parameter in ConfigFile
     * @param parameter with the name of the parameter in configfile
     * @param parameter_float float where the value will be returned
     * @return boolean, false if the parameter is not in configfile and true if it is
     */
    bool checkAndGetFloat(const string &parameter, float &parameter_float);

    /**
     * Get a string parameter in ConfigFile
     * @param parameter with the name of the parameter in configfile
     * @param parameter_string string where the value will be returned
     * @return boolean, false if the parameter is not in configfile and true if it is
     */
    bool checkAndGetString(const string &parameter, string &parameter_string);

    /**
     * Get a bool parameter in ConfigFile
     * @param parameter with the name of the parameter in configfile
     * @param parameter_bool string where the value will be returned
     * @return boolean, false if the parameter is not in configfile and true if it is
     */
    bool checkAndGetBool(const string &parameter, bool &parameter_bool);

    /**
     * Get a cv::Mat parameter in ConfigFile
     * @param parameter with the name of the parameter in configfile
     * @param parameter_mat cv::Mat where the value will be returned
     * @return boolean, false if the parameter is not in configfile and true if it is
     */
    bool checkAndGetMat(const string &parameter, cv::Mat &parameter_mat);

private:

    cv::FileStorage fs_;

};

#endif
