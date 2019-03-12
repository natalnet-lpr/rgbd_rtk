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

#include <iostream>
#include <math.h>
#include <Eigen/Geometry>
#include <pcl/filters/frustum_culling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include "velodyne_loader.h"
#include "kitti_cloud_generator.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
  string path_lidar =  argv[1];
  string path_leftImage =  argv[2];
  string path_rightImage =  argv[3];
  VelodyneLoader vl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cv = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cv_cpy = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);  
  float T[3]={-0.004069766,-0.00631618,-0.2717806};
  float R[3][3]={0.007533745,     -0.999714,    -0.0006166020,
           0.01489249,      0.0007280733, -0.998902,
           0.9998621,       0.007523790,  0.01480755
  };

  
  vl.loaderPathSequence(path_lidar,0);
  *cv=*vl.getPointCloudRGB();



  //Rotation and Translation: P_s = R*(P_l - T)
  float x,y,z;
  for(int i=0;i<cv->points.size();i++){
      x=cv->points[i].x;
      y=cv->points[i].y;
      z=cv->points[i].z;
      cv->points[i].x= R[0][0]*(x-T[0]) + R[0][1]*(y-T[1]) + R[0][2]*(z-T[2]);  
      cv->points[i].y= R[1][0]*(x-T[0])+ R[1][1]*(y-T[1]) + R[1][2]*(z-T[2]);  
      cv->points[i].z= R[2][0]*(x-T[0])+ R[2][1]*(y-T[1]) + R[2][2]*(z-T[2]);  
  }
    


  pcl::FrustumCulling<pcl::PointXYZRGB> fc;
  Eigen::Matrix4f pose; //Reference pose of PCL coordinate system where X is forward, Y is up, and Z is right.
  pose << 0, 0, 1, 0,
               0,-1, 0, 0,
               1, 0, 0, 0,
               0, 0, 0, 1;
  fc.setInputCloud (cv);
  fc.setCameraPose(pose);
  fc.setVerticalFOV (105);
  fc.setHorizontalFOV (105);
  fc.setNearPlaneDistance (5);
  fc.setFarPlaneDistance (120);
  fc.filter (*cv_cpy);


  KITTICloudGenerator kcg;
  Mat leftimage, rightimage;
  Mat Q = (cv::Mat_<float>(4,4) <<1, 0, 0,       -607.1928, 
                                      0, 1, 0,       -185.2157,
                                      0, 0, 0,       718.856,
                                      0, 0, 1/0.54, 0);

  leftimage= imread(path_leftImage);
  rightimage=imread(path_rightImage);

  kcg.cloudGenerator(leftimage,rightimage,Q);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cs = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cs_cpy = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  *cs=*kcg.clearPointCloud();
   pcl::PCDWriter writer1;
  writer1.write<pcl::PointXYZRGB> ("out_stereo.pcd", *cs, false); 
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

  kdtree.setInputCloud (cv_cpy);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int i=0;i<cs->points.size();i++){
  	if(!isnan(cs->points[i].z) || !isnan(cs->points[i].x) || !isnan(cs->points[i].y)){
        vector<int> pointIdxNKNSearch(1);
        vector<float> pointNKNSquaredDistance(1);
        kdtree.nearestKSearch(cs->points[i],1,pointIdxNKNSearch,pointNKNSquaredDistance);
        if(pointNKNSquaredDistance[0]<1.0){
          //out_cloud->points.push_back(cv->points[pointIdxNKNSearch[0]]);
        	cs->points[i].z=cv_cpy->points[pointIdxNKNSearch[0]].z;
        }
        else{
          cs->points[i].z=0.0;
        }
    }
    else{
    	//out_cloud->points.push_back(cs->points[i]);
    }
    

  }
  //out_cloud->width=cs->width;
  //out_cloud->height= cs->height;


  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("out_filtered.pcd", *cs, false);  
    return 0;
}
