//export LD_LIBRARY_PATH=/home/vanessadantas/Downloads/opencv-3.4.1/Build/lib
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "kitti_cloud_generator.h"

using namespace std;
using namespace cv;


int main()
{
  KITTICloudGenerator kcg;
  Mat leftimage, rightimage;
  Mat Q = (cv::Mat_<float>(4,4) <<1, 0, 0,       -607.1928, 
                                      0, 1, 0,       -185.2157,
                                      0, 0, 0,       718.856,
                                      0, 0, 1/0.54, 0);


  leftimage= imread("/home/ferreira/Documentos/DATASETS/Color/dataset/sequences/04/image_2/000000.png");
  rightimage=imread("/home/ferreira/Documentos/DATASETS/Color/dataset/sequences/04/image_3/000000.png");

  kcg.cloudGenerator(leftimage,rightimage,Q);




  pcl::PointXYZRGB minPt, maxPt;
  pcl::getMinMax3D (*kcg.clearPointCloud(), minPt, maxPt);

  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;     

 kcg.disparityMapGenerator(leftimage,rightimage);



    return 0;
}

