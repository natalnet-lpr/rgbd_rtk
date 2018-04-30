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

#include <geometry.h>
#include <rgbd_loader.h>
#include <icp_odometry.h>
#include <reconstruction_visualizer.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	string index_file_name;
	RGBDLoader loader;
	Intrinsics intr(0);
	ICPOdometry icpo(intr);
	ReconstructionVisualizer visualizer;
	Mat frame, depth;

	if(argc != 2)
	{
		fprintf(stderr, "Usage: %s <index file>\n", argv[0]);
		exit(0);
	}

	index_file_name = argv[1];
	loader.processFile(index_file_name);

	//Compute ICP odometry on each image
	for(int i = 0; i < loader.num_images_; i++)
	{
		//Load RGB-D image 
		loader.getNextImage(frame, depth);

		//Estimate current camera pose
		icpo.computeCameraPose(frame, depth);

		if(i == 0) visualizer.addReferenceFrame(icpo.pose_, "origin");
		//visualizer.addQuantizedPointCloud(icpo.curr_dense_cloud_, 0.3, icpo.pose_);
		visualizer.viewReferenceFrame(icpo.pose_);
		//visualizer.viewPointCloud(icpo.curr_dense_cloud_, icpo.pose_);
		//visualizer.viewQuantizedPointCloud(icpo.curr_dense_cloud_, 0.02, icpo.pose_);

		visualizer.spinOnce();

		//Show RGB-D image
		imshow("Image view", frame);
		imshow("Depth view", depth);
		char key = waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}
	}

	return 0;
}