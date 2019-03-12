#include "kitti_cloud_generator.h"

#define FLT_EPSILON __FLT_EPSILON__

using namespace cv;

KITTICloudGenerator::KITTICloudGenerator(){

	cloud_rgbxyz = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void KITTICloudGenerator::disparityMapGenerator(Mat leftImage, Mat rightImage,int minDisparity,
	int blockSize,int disp12MaxDiff,int preFilterCap, int uniquenessRatio,int speckleWindowSize,int speckleRange){
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>);
	    int numDisparities=64-minDisparity;
		Mat leftImage_gray, rightImage_gray, disparity, true_disparity;
		int image_channels = leftImage_gray.channels();
		int P1 = 8*image_channels*blockSize*blockSize;
  		int P2 = 32*image_channels*blockSize*blockSize;
  		cv::cvtColor(leftImage, leftImage_gray, cv::COLOR_BGR2GRAY);
  		cv::cvtColor(rightImage, rightImage_gray, cv::COLOR_BGR2GRAY);
		cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
        minDisparity, numDisparities, blockSize, 
        P1, P2, disp12MaxDiff, preFilterCap,  
        uniquenessRatio, speckleWindowSize, 
        speckleRange, cv::StereoSGBM::MODE_SGBM_3WAY
    );
		stereo->compute(leftImage_gray,rightImage_gray, disparity);

		disparity.convertTo(true_disparity, CV_32F, 1.0/16.0, 0.0);
		
		//imwrite("disparity.png",true_disparity);
		
		imshow("Disparity", true_disparity);
		waitKey();
		

}
	void KITTICloudGenerator::cloudGenerator(Mat leftImage, Mat rightImage,Mat Q,int minDisparity,
	int blockSize,int disp12MaxDiff,int preFilterCap, int uniquenessRatio,int speckleWindowSize,int speckleRange){
	
	    
	int numDisparities=64-minDisparity;
	Mat leftImage_gray, rightImage_gray, disparity, true_disparity;
  Mat cloud_xyz;
	 
  uchar r, g, b;

	int image_channels = leftImage_gray.channels();
	int P1 = 8*image_channels*blockSize*blockSize;
	int P2 = 32*image_channels*blockSize*blockSize;
		
	cv::cvtColor(leftImage, leftImage_gray, cv::COLOR_BGR2GRAY);
	cv::cvtColor(rightImage, rightImage_gray, cv::COLOR_BGR2GRAY);
	
	cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
      minDisparity, numDisparities, blockSize, 
      P1, P2, disp12MaxDiff, preFilterCap,  
      uniquenessRatio, speckleWindowSize, 
      speckleRange, cv::StereoSGBM::MODE_SGBM_3WAY
      );
	
	stereo->compute(leftImage_gray,rightImage_gray, disparity);

	disparity.convertTo(true_disparity, CV_32F, 1.0/16.0, 0.0);

	cv::reprojectImageTo3D(true_disparity, cloud_xyz, Q, true);
  
  depth_map.create(cloud_xyz.rows,cloud_xyz.cols,CV_32F);
    
  for(int i = 0; i < cloud_xyz.rows; i++)
  {
    for(int j = 0; j < cloud_xyz.cols; j++)
    {
      cv::Vec3f point = cloud_xyz.at<cv::Vec3f>(i, j);
      cv::Vec3b color = leftImage.at<cv::Vec3b>(i, j);

      //if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;

      pcl::PointXYZRGB point3D;

      //compatibilizar opencv e pcl    
      if(point[0]>=10000)
        point3D.x =   std::numeric_limits<double>::quiet_NaN();
      else
        point3D.x = point[0];

      if(point[1]>=10000)
        point3D.y = std::numeric_limits<double>::quiet_NaN();
      else
        point3D.y = point[1];

      if(point[2]>=10000)
        point3D.z = std::numeric_limits<double>::quiet_NaN();
      else
        point3D.z = point[2];
      //fim: compatibilizar opencv e pcl
      
      

      depth_map.at<float>(i,j)=point[2];

      b = color[0];
      g = color[1];
      r = color[2];
      
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                      static_cast<uint32_t>(g) << 8 | 
                      static_cast<uint32_t>(b));

      point3D.rgb = *reinterpret_cast<float*>(&rgb);

      cloud_rgbxyz->points.push_back (point3D);
    }
}
  //imwrite("depth.png",depth_map);
  cloud_rgbxyz->width=cloud_xyz.cols;
  cloud_rgbxyz->height=cloud_xyz.rows; 
  //pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZRGB> ("out_stereo_cloud.pcd", *cloud_rgbxyz, false);
}
Mat KITTICloudGenerator::getDepthMap(){
  return depth_map;  
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr KITTICloudGenerator::clearPointCloud(){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud_rgbxyz = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  *current_cloud_rgbxyz = *cloud_rgbxyz;
  cloud_rgbxyz->clear();
  return current_cloud_rgbxyz;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr KITTICloudGenerator::getPointCloud(){
  return cloud_rgbxyz;
}


KITTICloudGenerator::~KITTICloudGenerator(){


}






