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
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>
#include <surf_detector.h>
#include <visual_memory.h>
using namespace std;


int main(int argc, char **argv)
{
	
	string index_file_name;
	RGBDLoader loader;
	Intrinsics intr(0);	
	MotionEstimatorRANSAC motion_estimator(intr);
	ReconstructionVisualizer visualizer;
    cv::Mat frame, depth,prev_frame;
	Eigen::Affine3f pose = Eigen::Affine3f::Identity();
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr curr_cloud(new pcl::PointCloud<PointT>);
	SurfDetector detector_surf(400);
	VisualMemory memory;
	if(argc != 2)
	{
		fprintf(stderr, "Usage: %s <index file>\n", argv[0]);
		exit(0);
	}

	index_file_name = argv[1];
	loader.processFile(index_file_name);

	cv::Mat saved_descriptor;

	for(int i = 0; i < loader.num_images_; i++)
	{
	
		//Load RGB-D image and point cloud 
		loader.getNextImage(frame, depth);

		*curr_cloud = getPointCloud(frame, depth, intr);
		
		
		
		if(i > 0)
		{
			detector_surf.detectAndMatch(frame,prev_frame);
	

			detector_surf.drawSurfMatches(prev_frame,frame);
			trans = motion_estimator.estimate(detector_surf.prev_good_Pts, prev_cloud,
				                            detector_surf.curr_good_Pts, curr_cloud);
			pose = pose*trans;
		


		}
		
		if(i == 0) visualizer.addReferenceFrame(pose, "origin");
		visualizer.addQuantizedPointCloud(curr_cloud, 0.3, pose);
		visualizer.viewReferenceFrame(pose);
		visualizer.viewPointCloud(curr_cloud, pose);
		visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, pose);

		visualizer.spinOnce();


		
        cv::imshow("Depth view", depth);
		/*
		if(i == 5)
			detector_surf.SearchInTree(saved_frame);
		*/
		char key = waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}
		
	
		//Let the prev. cloud in the next frame be the current cloud
		*prev_cloud = *curr_cloud;
		prev_frame = frame;


	}

	return 0;
}
