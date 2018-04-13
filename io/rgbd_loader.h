/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Natalnet Laboratory for Perceptual Robotics
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
 */

#ifndef INCLUDE_RGBD_LOADER_H_
#define INCLUDE_RGBD_LOADER_H_

#include <vector>
#include <opencv2/core/core.hpp>

class RGBDLoader
{
private:
	//Vector position of the current RGB-D image
	int curr_img_;

	//Path of the index file
	std::string path_;

	//RGB file names to be loaded
	std::vector<std::string> rgb_img_names_;

	//Depth file names to be loaded
	std::vector<std::string> depth_img_names_;

public:
	//Number of images of the sequence
	int num_images_;

	/*
	 * Default constructor
	 */
	RGBDLoader()
	{
		num_images_ = 0;
		curr_img_ = 0;
	}

	/* 
	 * Constructor with the file name of the image sequence.
	 */ 
	RGBDLoader(const std::string index_file_name)
	{
		num_images_ = 0;
		curr_img_ = 0;
		processFile(index_file_name);
	}

	/*
	 * Main function: scans the index file and inserts the RGB/depth file names into two vectors,
	 * assuming the index file has a 4-tuple <timestamp rgb_name timestamp depth_name> per line. 
     * The function is public and thus can be called directly.
     */
	void processFile(const std::string index_file_name);

	/*
	 * Returns the next RGB-D image of the sequence.
	 */ 
	void getNextImage(cv::Mat& rgb_img, cv::Mat& depth_img);
};

#endif /* INCLUDE_RGBD_LOADER_H_ */