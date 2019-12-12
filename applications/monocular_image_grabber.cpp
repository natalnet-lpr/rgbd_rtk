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

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

double get_time_in_microseconds(){
	struct timeval now;
	gettimeofday(&now, NULL);
	return ((long long) (1000000 *now.tv_sec + now.tv_usec))/1000000.0;
}

void save_data(const string time_stamp, const cv::Mat rgb)
{
	//Save RGB image as PNG
	string rgb_img_name = time_stamp + ".png";
	imwrite(rgb_img_name, rgb);
}

int main(int argc, char** argv)
{
	cv::Mat rgb_img;

	cv::VideoCapture cam(0);
	if(!cam.isOpened())
	{
		fprintf(stderr, "There's a problem opening the capture device.\n");
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}

	//Program variables
	bool record = false;

	ofstream rgb_index("rgb.txt");
	if(!rgb_index.is_open())
	{
		fprintf(stderr, "There is a problem with the supplied file for the image index.\n");
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}

	while(true)
	{
		//Get current timestamp (same format as RGB-D TUM datasets)
		std::stringstream time_stamp;
		time_stamp << std::fixed << std::setprecision(6) << get_time_in_microseconds();

		//Grab current image from the camera
		cam >> rgb_img;
		if(rgb_img.empty())
		{
			fprintf(stderr, "There is a problem capturing the video frame\n");
			exit(-1);
		}
		cv::Mat frame_c;
		rgb_img.copyTo(frame_c); //deep copy of the RGB frame for image info and debug

		if(!record)
			cv::putText(frame_c,"Press 's' to start capturing data.", cv::Point2i(50, 460), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
		else
			cv::putText(frame_c,"Capturing data...", cv::Point2i(50, 460), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);

		//Save image data
		if(record)
		{
			save_data(time_stamp.str(), rgb_img);

			rgb_index << time_stamp.str() << " rgb/" << time_stamp.str() << ".png\n";
		}
		
		//Show rgb, depth and point cloud
		cv::imshow("RGB Data", frame_c);
		
		char key = cv::waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting\n");
			break;
		}
		if(key == 's' || key == 'S')
		{
			record = !record;
		}
		
	}

	return 0;
}


