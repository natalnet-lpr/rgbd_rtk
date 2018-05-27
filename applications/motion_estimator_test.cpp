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
#include <klttacw_tracker.h>
#include <klt_tracker.h>
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>


#include <fstream>
#include <ctime>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	string index_file_name;
	RGBDLoader loader;

	cout<<"this code is running using the KLTTACW\n";
		KLTTrackerACW tracker;
	tracker.radius_min = 20;
	tracker.radius_max = 50;
	
	//KLTTracker tracker;
	Intrinsics intr(0);
	MotionEstimatorRANSAC motion_estimator(intr);
	ReconstructionVisualizer visualizer;
	Mat frame, depth;
	Eigen::Affine3f pose = Eigen::Affine3f::Identity();
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr curr_cloud(new pcl::PointCloud<PointT>);
	ofstream cam_path;
	

	ofstream time;
	clock_t ti,tf;	

	time.open("tempo_frame.txt");
	
	cam_path.open("pos_relativa.txt");
	if(argc != 2)
	{
		fprintf(stderr, "Usage: %s <index file>\n", argv[0]);
		exit(0);
	}

	index_file_name = argv[1];
	loader.processFile(index_file_name);

	//Track points on each image
	for(int i = 0; i < loader.num_images_; i++)
	{
		//Load RGB-D image and point cloud 
		loader.getNextImage(frame, depth);
		*curr_cloud = getPointCloud(frame, depth, intr);

		//Track feature points in the current frame
		tracker.track(frame);

		//Estimate motion between the current and the previous frame/point clouds
		if(i > 0)
		{	ti=clock();
			trans = motion_estimator.estimate(tracker.prev_pts_, prev_cloud,
				                              tracker.curr_pts_, curr_cloud);
			pose = pose*trans;
			tf= clock();
			time<<(tf-ti)*1000/CLOCKS_PER_SEC<<endl;
		}

		//View tracked points
		for(size_t k = 0; k < tracker.curr_pts_.size(); k++)
		{
			Point2i pt1 = tracker.prev_pts_[k];
			Point2i pt2 = tracker.curr_pts_[k];
			circle(frame, pt1, 1, CV_RGB(255,0,0), -1);
			circle(frame, pt2, 3, CV_RGB(0,0,255), -1);
			line(frame, pt1, pt2, CV_RGB(0,0,255));
		}

		if(i == 0) visualizer.addReferenceFrame(pose, "origin");
		//visualizer.addQuantizedPointCloud(curr_cloud, 0.3, pose);
		//esse visualizer.viewReferenceFrame(pose);
		//visualizer.viewPointCloud(curr_cloud, pose);
		//esse visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, pose);

		//esse visualizer.spinOnce();

		//Show RGB-D image
		imshow("Image view", frame);
		//imshow("Depth view", depth);
		char key = waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}

		//Let the prev. cloud in the next frame be the current cloud
		*prev_cloud = *curr_cloud;
	
			 Eigen::Matrix3f R;
	      		    R(0,0) = pose(0,0); R(0,1) = pose(0,1); R(0,2) = pose(0,2);
      			  R(1,0) = pose(1,0); R(1,1) = pose(1,1); R(1,2) = pose(1,2);
      			  R(2,0) = pose(2,0); R(2,1) = pose(2,1); R(2,2) = pose(2,2);
      			  Eigen::Quaternionf q(R);
      			  cam_path<< loader.tstamps[i] << " " << pose(0,3) << " "
                                      << pose(1,3) << " "
                                      << pose(2,3) << " "
                                      << q.x() << " "
                                      << q.y() << " "
                                      << q.z() << " "
                                      << q.w() << "\n";						
		
	}

	return 0;
}
