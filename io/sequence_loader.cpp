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

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "sequence_loader.h"

using namespace std;
using namespace cv;

void SequenceLoader::processFile(const string file_name)
{
	ifstream index_file(file_name.c_str());
	if(!index_file.is_open())
	{
		fprintf(stderr, "ERROR: file %s was not found.\nExiting.\n", file_name.c_str());
		exit(0);
	}
	printf("Opening index file: %s\n", file_name.c_str());

	//Extract path from the supplied argument
	int p = file_name.rfind('/');
	path_ = file_name.substr(0, p+1);

	while(!index_file.eof())
	{
		//Skip comments (lines starting with a '#')
		int c = index_file.peek();
		if(c != int('#'))
		{
			string ts, img_file;
			index_file >> ts >> img_file;

			//Compatiblity with index files ended with \n
			if(img_file != "")
			{
				num_images_++;
				img_names_.push_back(img_file);
			}
		}
		else
		{
			index_file.ignore(numeric_limits<streamsize>::max(), '\n');
		}
	}
	index_file.close();
}

Mat SequenceLoader::getNextImage()
{
	if(curr_img_ < num_images_)
	{
		string file_name = path_ + img_names_[curr_img_++];
		Mat img = imread(file_name, CV_LOAD_IMAGE_UNCHANGED);
		if(img.empty())
		{
			fprintf(stderr, "ERROR: image file %s not found.\nExiting.\n", file_name.c_str());
			exit(0);
		}
		return img;
	}
	else
	{
		fprintf(stderr, "ERROR: all images of the sequence have been loaded.\nExiting.\n");
		exit(0);
	}
}