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

#ifndef KITTI_STEREOLOADER_H
#define KITTI_STEREOLOADER_H

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>

class KITTIStereoLoader
{

private:

    //Left image of the stereo pair
	cv::Mat left_image_;

    //Right image of the stereo pair
	cv::Mat right_image_;

    //Path of the root directory of the sequence
	std::string root_path_;

    //Path of the root dir. + "dataset/sequences/" + seq_number_;
    std::string prefix_path_;

    //Image indices
	size_t next_left_;
 	size_t next_right_;

    //Strings of all left images of a sequence
	std::vector<std::string> left_image_names_;

    //Strings of all right images of a sequence
	std::vector<std::string> right_image_names_; 

public:

    //Default constructor
    KITTIStereoLoader() : next_left_(0), next_right_(0), num_pairs_(0)
    {}

	//Number of stereo pairs of the sequence
    size_t num_pairs_;

    //Loads a stereo pair from two given paths
	void loadStereoPair(const std::string& left_path, const std::string& right_path);

    /* Loads a sequence of stereo pairs from a given path.
     * The second parameter informs the sequence number,
     * the third parameter informs if the color cameras should be used.
     */
	void loadStereoSequence(const std::string& sequence_path, const int& sequence_num, const bool& use_color);

    //Returns the left image
    cv::Mat getLeftImage();

    //Returns the right image
    cv::Mat getRightImage();

    //Returns the next left image
    cv::Mat getNextLeftImage(const bool& use_color);

    //Returns the next right image
    cv::Mat getNextRightImage(const bool& use_color);

    ~KITTIStereoLoader();
};

#endif // KITTI_STEREOLOADER_H 

