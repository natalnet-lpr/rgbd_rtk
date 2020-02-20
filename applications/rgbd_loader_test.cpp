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
 *  Author:
 *
 *  Bruno Silva
 */

#include <cstdio>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <config_loader.h>
#include <rgbd_loader.h>
#include <event_logger.h>

using namespace std;
using namespace cv;

/** This program shows how to use RGBDLoader class to process a sequence of RGB-D images.
 * @param .yml config. file (from which index_file is used)
 */
int main(int argc, char **argv)
{
	EventLogger& logger = EventLogger::getInstance();
	logger.setVerbosityLevel(pcl::console::L_INFO);

	RGBDLoader loader;
	Mat frame, depth;
	string index_file;

	if(argc != 2)
	{
		logger.print(pcl::console::L_INFO, "[rgbd_loader_test.cpp] Usage: %s <path/to/config_file.yaml>\n", argv[0]);
		exit(0);
	}
	ConfigLoader param_loader(argv[1]);
	param_loader.checkAndGetString("index_file", index_file);
	loader.processFile(index_file);

	for(int i = 0; i < loader.num_images_; i++)
	{
		loader.getNextImage(frame, depth);
		imshow("Image view", frame);
		imshow("Depth view", depth);
		char key = waitKey(16);
		if(key == 'q' || key == 'Q' || key == 27)
		{
			logger.print(pcl::console::L_INFO, "[rgbd_loader_test.cpp] Exiting\n", argv[0]);
			break;
		}
	}

	return 0;
}