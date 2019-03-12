#ifndef VELODYNE_LOADER_H
#define VELODYNE_LOADER_H


#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;


class VelodyneLoader
{
private:
	string s_path;
	string s_num;
	int next=0;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_velodyne;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_velodyne_rgb;
	vector<string> lidarBinNameList; 

    
public:
	int num_bins=0;
	VelodyneLoader();
	void loaderPathSequence(string sequence_path,int sequence_num);
	pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudRGB();
    ~VelodyneLoader();


};

#endif /* VELODYNE_LOADER_H */

