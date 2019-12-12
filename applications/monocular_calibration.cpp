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
#include <ctime>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	if(argc != 4)
	{
		fprintf(stderr, "usage: %s <n_corners_horizontal_dir> <n_corners_vertical_dir>"
				        " <0 for webcamera, 1 for kinect>\n", argv[0]);
		exit(-1);
	}

	int kinect = atoi(argv[3]);

	clock_t last_step = 0;
	Mat K, D;
	Size board_size(atoi(argv[1]),atoi(argv[2]));
	Size im_size;

	const char* output_file = "cd.yml"; //calibration file
	int delta_t = 2.0; //time between frames in seconds
	int flags = 0; //calibration flags
	int n_images = 10; //number of images
	float sq_size = 1.0; //chessboard square size

	//Set each corner world coordinate
	vector<vector<Point3f> > corners_3d;
	for(int k = 0; k < n_images; k++)
	{
		vector<Point3f> tmp;
		for(int i = 0; i < board_size.height; i++)
		{
			for(int j = 0; j < board_size.width; j++)
			{
				Point3f pt = Point3f((float) j*sq_size, (float) i*sq_size, 0);
				tmp.push_back(pt);
			}
		}
		corners_3d.push_back(tmp);
	}

	vector<vector<Point2f> > corners_2d;

	VideoCapture capture(0);
	if(!capture.isOpened())
	{
		fprintf(stderr, "There's a problem with the capture device.\n");
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}

	last_step = clock();
	int count = 0;
	while(count < n_images)
	{
		//Capture frame
		Mat frame, g_frame;
		if(!kinect)
		{
			capture >> frame; //webcamera
		}
		else
		{
			capture.grab();
			capture.retrieve(frame, CV_CAP_OPENNI_BGR_IMAGE); //Kinect
		}
		cvtColor(frame, g_frame, CV_BGR2GRAY);

		im_size = frame.size();
		vector<Point2f> curr_corners;

		//Find chessboard corners
		bool found = findChessboardCorners(frame, board_size, curr_corners,
				     CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		time_t curr_step = clock();
		if(found && ((curr_step - last_step)/CLOCKS_PER_SEC >= delta_t))
		{
			//Refine chessboard corners
			cornerSubPix(g_frame, curr_corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			//Add refined corners to the list of corners
			corners_2d.push_back(curr_corners);

			last_step = curr_step;
			count++;
			printf("%u/%i images captured\n", count, n_images);
		}

		drawChessboardCorners(frame, board_size, Mat(curr_corners), found);

		imshow("Camera Calibration", frame);
		int k = (char) waitKey(16);
		if(k == 'q' || k == 'Q')
		{
			exit(-1);
		}
	}

	//Run calibration procedure
	K = Mat::eye(3,3, CV_64F);
	D = Mat::zeros(4,1, CV_64F);
	vector<Mat> rvecs, tvecs;
	double rms = calibrateCamera(corners_3d, corners_2d, im_size, K, D, rvecs, tvecs);
	printf("Calibration Error (RMS): %f\n", rms);

	//Print calibration data and save to file
	printf("%f %f %f\n", K.at<double>(0,0), K.at<double>(0,1), K.at<double>(0,2));
	printf("%f %f %f\n", K.at<double>(1,0), K.at<double>(1,1), K.at<double>(1,2));
	printf("%f %f %f\n", K.at<double>(2,0), K.at<double>(2,1), K.at<double>(2,2));
	printf("%f %f %f %f\n", D.at<double>(0,0), D.at<double>(1,0), D.at<double>(2,0), D.at<double>(3,0));
	FileStorage fs(output_file, FileStorage::WRITE);
	if(!fs.isOpened())
	{
		fprintf(stderr, "There's a problem with the output calibration file: %s.\n", output_file);
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}
	fs << "camera_matrix" << K;
	fs << "distortion_coefficients" << D;
	fs.release();

	bool show_calibrated = false;
	//Show undistorted image
	while(1)
	{
		//Capture frame
		Mat frame, u_frame;
		if(!kinect)
		{
			capture >> frame; //webcamera
		}
		else
		{
			capture.grab();
			capture.retrieve(frame, CV_CAP_OPENNI_BGR_IMAGE); //Kinect
		}

		int k = (char) waitKey(16);
		if(k == 'q' || k == 'Q')
			break;
		if(k == 'u')
			show_calibrated = !show_calibrated;


		if(show_calibrated)
		{
			undistort(frame, u_frame, K, D);
			imshow("Undistorted Image", u_frame);
		}
		else
		{
			imshow("Undistorted Image", frame);
		}
	}

	return 0;
}

