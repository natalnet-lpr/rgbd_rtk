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
#include <Eigen/Geometry>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include "velodyne_loader.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
  string path =  argv[1];
  VelodyneLoader vl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cv = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  //T_velo_2_cam = | R T | ; R: 3x3 Rotation Matrix;
  //     | 0 1 | ; T: 1x3 Translation Matrix from calib_velo_2_cam.txt	
  float T[3]={-0.004069766,-0.00631618,-0.2717806};
  float R[3][3]={0.007533745,			-0.999714,		-0.0006166020,
  				 0.01489249,			0.0007280733,	-0.998902,
  				 0.9998621,				0.007523790,	0.01480755
  };
  //Load name of .bin files on dataset
  vl.loaderPathSequence(path,0);
  //Load XYZRGB monochromatic cloud from .bin file
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
  //Write image on velodyne_loader_test.pcd
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("velodyne_loader_test.pcd", *cv, false);



  
    return 0;
}
