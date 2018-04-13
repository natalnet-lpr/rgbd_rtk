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

/*
 * Kinect RGB/Depth/Point cloud grabber.
 * Uses OpenCV to provide a synchronized grabber for all Kinect streams (depth, RGB and point cloud).
 *
 *
 * Author: Bruno Marques F. da Silva
 * brunomfs@gmail.com
 */

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <common_types.h>
#include <geometry.h>
#include <reconstruction_visualizer.h>

using namespace std;

void save_data(const string time_stamp, const cv::Mat rgb, const cv::Mat depth)
{
	//Save RGB image as PNG
	string rgb_img_name = time_stamp + ".png";
	imwrite(rgb_img_name, rgb);

	//Save depth as a PNG image (depth is an array of SHORT/16 bits)
	//Scale factor is used to compatibilize dataset with the RGB-D TUM:
	//http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
	int scale_factor = 5;
	cv::Mat depth_scaled = depth*scale_factor;
	string depth_buffer_name = time_stamp + "_depth.png";
	imwrite(depth_buffer_name, depth_scaled);
}

int main(int argc, char** argv)
{
	//Program variables
	bool record = false;
	cv::Mat rgb_img, depth_buffer, cv_cloud;
	Intrinsics intr(0);
	ReconstructionVisualizer visualizer;
	pcl::PointCloud<PointT>::Ptr cloud_ptr;

	cv::VideoCapture cam(CV_CAP_OPENNI);
	if(!cam.isOpened())
	{
		fprintf(stderr, "There's a problem opening the capture device.\n");
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}

	ofstream rgb_index("rgb.txt");
	if(!rgb_index.is_open())
	{
		fprintf(stderr, "There is a problem with the supplied file for the image index.\n");
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}
	ofstream depth_index("depth.txt");
	if(!depth_index.is_open())
	{
		fprintf(stderr, "There is a problem with the supplied file for the depth buffer index.\n");
		fprintf(stderr, "Exiting.\n");
		exit(-1);
	}

	/*
	 * OpenCV Kinect interface is used to retrive the following synchronized streams:
	 * 1. RGB Image
	 * 2. Depth Image
	 * 3. Point cloud
	 */
	while(true)
	{
		//Get current timestamp (same format as RGB-D TUM datasets)
		std::stringstream time_stamp;
		time_stamp << std::setprecision(16) << pcl::getTime();

		//Grab Kinect streams
		cam.grab();
		cam.retrieve(rgb_img, CV_CAP_OPENNI_BGR_IMAGE);
		cam.retrieve(depth_buffer, CV_CAP_OPENNI_DEPTH_MAP);
		cam.retrieve(cv_cloud, CV_CAP_OPENNI_POINT_CLOUD_MAP);
		if(rgb_img.empty() || depth_buffer.empty() || cv_cloud.empty())
		{
			fprintf(stderr, "There is a problem capturing the video frame\n");
			exit(-1);
		}
		cv::Mat frame_c;
		rgb_img.copyTo(frame_c); //deep copy of the RGB frame for image info and debug

		//Assemble PCL point cloud from OpenCV
		pcl::PointCloud<PointT> pcl_cloud = getPointCloud(rgb_img, depth_buffer, intr);
		*cloud_ptr = pcl_cloud;

		//Show point cloud data
		Eigen::Affine3f identity = Eigen::Affine3f::Identity();
		visualizer.viewPointCloud(cloud_ptr, identity);

		/*** Save data (depth/RGB buffers) ***/
		if(!record)
		{
			cv::putText(frame_c,"Press 's' to start capturing data.", cv::Point2i(50, 460), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
		}
		else
		{
			cv::putText(frame_c,"Capturing data...", cv::Point2i(50, 460), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);
			save_data(time_stamp.str(), rgb_img, depth_buffer);

			rgb_index << time_stamp.str() << " rgb/" << time_stamp.str() << ".png\n";
			depth_index << time_stamp.str() << " depth/" << time_stamp.str() << "_depth.png\n";
		}

		//Save depth as a .yml file (NOT USED)
		//string depth_buffer_name = time_stamp.str() + ".yml";
		//cv::FileStorage fs(depth_buffer_name, cv::FileStorage::WRITE);
		//if(!fs.isOpened())
		//{
		//	fprintf(stderr, "There's a problem with the OpenCV file storage.\n");
		//	fprintf(stderr, "Exiting.\n");
		//	exit(-1);
		//}
		//fs << "depth_buffer" << depth_buffer;
		//fs.release();

		//Save depth as a PCD data (NOT USED)
		//pcl::PCDWriter pcd_writer;
		//string pcd_name = timeStamp.str() + ".pcd";
		//pcd_writer.writeBinaryCompressed(pcd_name, pcl_cloud);
		
		//Show rgb, depth and point cloud
		cv::imshow("Kinect RGB Data", frame_c);

		//approximately the minimum sensed depth in milimeters (to be encoded in 0-255)
		//0.05 = 255 (number of values in 8 bits) / 5000 (max sensed depth for Kinect)
		const float s = 0.051;
		cv::Mat show;
		depth_buffer.convertTo(show, CV_8UC1, s);
		cv::imshow("Kinect Depth Data", show);

		visualizer.spinOnce();
		
		int key = (char) cv::waitKey(1);
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


