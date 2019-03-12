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
#include <cstdlib>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry.h>
#include <rgbd_loader.h>
#include <optical_flow_visual_odometry.h>
#include <reconstruction_visualizer.h>
#include <kitti_stereo_loader.h>
#include <kitti_cloud_generator.h>
#include <stereo_optical_flow_visual_odometry.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	KITTIStereoLoader loader; //StereoLoader
	KITTICloudGenerator generator;
	string path =  argv[1];
	int sequenceNum =atoi(argv[2]);
	Intrinsics intr(718.856,718.856,607.1928,185.2157);
	Mat Q = (cv::Mat_<float>(4,4) <<1, 0, 0,       -607.1928, 
                                      0, 1, 0,       -185.2157,
                                      0, 0, 0,       718.856,
                                      0, 0, 1/0.54, 0); //0.54 diz que a unidade é metro
	StereoOpticalFlowVisualOdometry vo(intr);
	ReconstructionVisualizer visualizer;
	Mat frame, depth;

	if(argc != 3)
	{
		fprintf(stderr, "Usage: %s <index file> <index sequence>\n ", argv[0]);
		exit(0);
	}
	//Load stereo sequence
	loader.loaderImageSequence(path,sequenceNum,true);
	//Prepare to save poses
	double vect_poses[loader.num_images][12];
    char s_num[3];
    sprintf(s_num, "%02d", sequenceNum);
    cout<<"The poses file will be write into stereo_pose_"<<s_num<<".txt"<<endl;
    string pose_s= "stereo_pose_"+string(s_num)+".txt";
    ofstream outputFile(pose_s);
    outputFile<<std::scientific;

	//Compute visual odometry on each image
	for(int i = 0; i < loader.num_images; i++)
	{
		//Load Stereo image 
		frame=loader.getNextLeftSequence(true);
		generator.cloudGenerator(frame,loader.getNextRightSequence(true),Q);
		depth=generator.getDepthMap();
		//Estimate current camera pose
		vo.computeCameraPose(frame, generator.clearPointCloud()); 
		vect_poses[i][0]=vo.pose_.matrix()(0,0);
		vect_poses[i][1]=vo.pose_.matrix()(0,1);
		vect_poses[i][2]=vo.pose_.matrix()(0,2);
		vect_poses[i][3]=vo.pose_.matrix()(0,3);
		vect_poses[i][4]=vo.pose_.matrix()(1,0);
		vect_poses[i][5]=vo.pose_.matrix()(1,1);
		vect_poses[i][6]=vo.pose_.matrix()(1,2);
		vect_poses[i][7]=vo.pose_.matrix()(1,3);
		vect_poses[i][8]=vo.pose_.matrix()(2,0);
		vect_poses[i][9]=vo.pose_.matrix()(2,1);
		vect_poses[i][10]=vo.pose_.matrix()(2,2);
		vect_poses[i][11]=vo.pose_.matrix()(2,3);

			
		//View tracked points
		
		for(size_t k = 0; k < vo.tracker_.curr_pts_.size(); k++)
		{
			Point2i pt1 = vo.tracker_.prev_pts_[k];
			Point2i pt2 = vo.tracker_.curr_pts_[k];
			circle(frame, pt1, 1, CV_RGB(255,0,0), -1);
			circle(frame, pt2, 3, CV_RGB(0,0,255), -1);
			line(frame, pt1, pt2, CV_RGB(0,0,255));
		}
	
		if(i == 0) visualizer.addReferenceFrame(vo.pose_, "origin");
		visualizer.addQuantizedPointCloud(vo.curr_dense_cloud_, 5.0, vo.pose_);
		visualizer.viewReferenceFrame(vo.pose_);
		visualizer.viewPointCloud(vo.curr_dense_cloud_, vo.pose_);
		//visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.02, vo.pose_);

		visualizer.spinOnce();

		//Show RGB-D image
		imshow("Image view", frame);
		//imshow("Depth view", depth);
		char key = waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}
	}
	for(int j=0;j<loader.num_images;j++){
		if(j!=loader.num_images-1){
			outputFile << vect_poses[j][0] <<" "<<vect_poses[j][1]<<" "<<vect_poses[j][2]<<" "<<vect_poses[j][3]<<" "<<vect_poses[j][4]<<" "<<vect_poses[j][5]<<" "<<
			vect_poses[j][6]<<" "<<vect_poses[j][7]<<" "<<vect_poses[j][8]<<" "<<vect_poses[j][9]<<" "<<vect_poses[j][10]<<" "<<vect_poses[j][11]<<"\n";}
		else{
			outputFile << vect_poses[j][0] <<" "<<vect_poses[j][1]<<" "<<vect_poses[j][2]<<" "<<vect_poses[j][3]<<" "<<vect_poses[j][4]<<" "<<vect_poses[j][5]<<" "<<
			vect_poses[j][6]<<" "<<vect_poses[j][7]<<" "<<vect_poses[j][8]<<" "<<vect_poses[j][9]<<" "<<vect_poses[j][10]<<" "<<vect_poses[j][11];
		}
		
	}	
	outputFile.close();

	return 0;
}