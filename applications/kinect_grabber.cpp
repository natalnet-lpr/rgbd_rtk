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

#include <vector>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

#include <common_types.h>
#include <geometry.h>
#include <event_logger.h>
#include <reconstruction_visualizer.h>

using namespace std;

/* 
 * Utility function: returns true
 * if a directory with given name
 * exists.
 * (from https://stackoverflow.com/questions/18100097/portable-way-to-check-if-directory-exists-windows-linux-c
 */
bool dirExists(const char *path)
{
    struct stat info;

    if(stat( path, &info ) != 0)
        return false;
    else if(info.st_mode & S_IFDIR)
        return true;
    else
        return false;
}

void saveRgbDepthImages(const string& prefix, const string& time_stamp, const cv::Mat& rgb, const cv::Mat& depth)
{
	//Save RGB image as PNG
	string rgb_img_name = prefix + "/rgb/" + time_stamp + ".png";
	imwrite(rgb_img_name, rgb);

	//Save depth as a PNG image (depth is an array of SHORT/16 bits)
	//Scale factor is used to compatibilize dataset with the RGB-D TUM:
	//http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
	int scale_factor = 5;
	cv::Mat depth_scaled = depth*scale_factor;
	string depth_buffer_name = prefix + "/depth/" + time_stamp + "_depth.png";
	imwrite(depth_buffer_name, depth_scaled);
}
	//Save depth as a PCD data (NOT USED)
	//pcl::PCDWriter pcd_writer;
	//string pcd_name = timeStamp.str() + ".pcd";
	//pcd_writer.writeBinaryCompressed(pcd_name, pcl_cloud);

void help(const char* prog_name)
{
	printf("Captures RGB-D data saving them to \"rgb\" and \"depth\" directories\n");
	printf("in the same format as the one used in TUM RGB-D datasets.\n\n");
	printf("Usage: %s <dataset_prefix>\n", prog_name);
	printf("For example, ./kinect_grabber my_data will save RGB/depth images to \"my_data\" directory.\n");
	printf("If run without arguments, RGB-D data is saved to \"kinect_dataset\" directory.\n\n");
	printf("To start recording, press 's' with the window showing the RGB image as active.\n");
	printf("Press 's' again to stop recording.\n");
}

/*
 * Kinect RGB/Depth/Point cloud grabber.
 * Uses OpenCV to provide a synchronized grabber for all Kinect streams (depth, RGB and point cloud).
 */
int main(int argc, char** argv)
{
	EventLogger& logger = EventLogger::getInstance();
	logger.setVerbosityLevel(pcl::console::L_INFO);

	//Program variables
	bool record = false;
	cv::Mat rgb_img, depth_buffer, cv_cloud;
	Intrinsics intr(0);
	ReconstructionVisualizer visualizer;
	pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
	string prefix, rgb_dir, rgb_index_name, depth_dir, depth_index_name;

	help(argv[0]);

	//Assign directory name
	if(argc == 1)
	{
		prefix = "kinect_dataset";
	}
	if(argc == 2)
	{
		prefix = argv[1];
	}

	rgb_dir = prefix + "/rgb";
	depth_dir = prefix + "/depth";
	rgb_index_name = prefix + "/rgb.txt";
	depth_index_name = prefix + "/depth.txt";

	//Check if directory exists and create it if it does not
	if(!dirExists(prefix.c_str()))
	{
		logger.print(pcl::console::L_INFO, "[kinect_grabber.cpp] INFO: creating directory %s.\n", prefix.c_str());
		boost::filesystem::create_directory(prefix.c_str());
		//Create rgb dir.
		if(!dirExists(rgb_dir.c_str()))
		{
			boost::filesystem::create_directory(rgb_dir.c_str());
		}

		//Create depth dir.
		if(!dirExists(depth_dir.c_str()))
		{
			boost::filesystem::create_directory(depth_dir.c_str());
		}
	}
	
	logger.print(pcl::console::L_INFO, "[kinect_grabber.cpp] INFO: saving data to %s directory.\n", prefix.c_str());

	//Open Kinect capture device
	cv::VideoCapture cam(CV_CAP_OPENNI);
	if(!cam.isOpened())
	{
		logger.print(pcl::console::L_ERROR, "[kinect_grabber.cpp] ERROR: there's a problem opening the capture device.\n");
		exit(-1);
	}

	//Open file streams for RGB-D image names
	ofstream rgb_index(rgb_index_name.c_str());
	if(!rgb_index.is_open())
	{
		logger.print(pcl::console::L_ERROR, "[kinect_grabber.cpp] ERROR: there's a problem opening the RGB index file.\n");
		exit(-1);
	}
	ofstream depth_index(depth_index_name.c_str());
	if(!depth_index.is_open())
	{
		logger.print(pcl::console::L_ERROR, "[kinect_grabber.cpp] ERROR: there's a problem opening the depth index file.\n");
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
			logger.print(pcl::console::L_ERROR, "[kinect_grabber.cpp] ERROR: there's a problem capturing the RGB-D data.\n");
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

		if(!record)
		{
			cv::putText(frame_c,"Press 's' to start capturing data.", cv::Point2i(50, 460), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
		}
		else
		{
			cv::putText(frame_c,"Capturing data...", cv::Point2i(50, 460), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);

			/*** Save data (index files and depth/RGB buffers) ***/
			saveRgbDepthImages(prefix, time_stamp.str(), rgb_img, depth_buffer);
			rgb_index << time_stamp.str() << " rgb/" << time_stamp.str() << ".png\n" << std::flush;
			depth_index << time_stamp.str() << " depth/" << time_stamp.str() << "_depth.png\n" << std::flush;
		}
		
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

	rgb_index.close();
	depth_index.close();

	return 0;
}


