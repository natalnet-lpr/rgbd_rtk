#ifndef KITTICLOUDGENERATOR_H
#define KITTICLOUDGENERATOR_H

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>
#include <limits>

using namespace cv;


class KITTICloudGenerator
{
private:
	Mat depth_map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbxyz;

public:
	KITTICloudGenerator();
	void disparityMapGenerator(Mat leftImage, Mat rightImage,int minDisparity=0,
		int blockSize=11,int disp12MaxDiff=1,int preFilterCap=63, int uniquenessRatio=15,int speckleWindowSize=200,int speckleRange=2); 
	void cloudGenerator(Mat leftImage, Mat rightImage,Mat Q,int minDisparity=0,
		int blockSize=11,int disp12MaxDiff=1,int preFilterCap=63, int uniquenessRatio=15,int speckleWindowSize=200,int speckleRange=2); 
	Mat getDepthMap();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clearPointCloud(); 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud();
	//void cloudAlign(cloud1,cloud2,Q1,Q2)
    ~KITTICloudGenerator();


};

#endif //KITTICLOUDGENERATOR_H

