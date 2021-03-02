/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2019, Natalnet Laboratory for Perceptual Robotics
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
 *  Authors:
 *
 *  Felipe Ferreira Barbosa
 *  Vanessa Dantas de Souto Costa
 *  Bruno Silva
 */

#include <cstdio>
#include <cstdlib>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry.h>
#include <config_loader.h>
#include <kitti_stereo_loader.h>
#include <stereo_cloud_generator.h>
#include <reconstruction_visualizer.h>
#include <stereo_optical_flow_visual_odometry.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    EventLogger& logger = EventLogger::getInstance();
    logger.setVerbosityLevel(EventLogger::L_DEBUG);
    logger.activateLoggingOnlyFor(EventLogger::M_MOTION_ESTIMATION);

	KITTIStereoLoader loader;
	ReconstructionVisualizer visualizer;

    if (argc != 2)
    {
        logger.print(EventLogger::L_ERROR,
                     "[stereo_optical_flow_visual_odometry.cpp] ERROR: "
                     "Usage: %s <path/to/config_file.yaml>\n", argv[0]);
        exit(0);
    }

    Mat left, right;
    float fx, fy, cx, cy, baseline, ransac_thr;
    string root_path;
    int kitti_seq;
    bool use_color_camera;

    ConfigLoader param_loader(argv[1]);
    param_loader.checkAndGetFloat("fx", fx);
    param_loader.checkAndGetFloat("fy", fy);
    param_loader.checkAndGetFloat("cx", cx);
    param_loader.checkAndGetFloat("cy", cy);
    param_loader.checkAndGetFloat("baseline", baseline);
    param_loader.checkAndGetString("kitti_root_dir", root_path);
    param_loader.checkAndGetInt("kitti_seq", kitti_seq);
    param_loader.checkAndGetBool("use_color_camera", use_color_camera);
    param_loader.checkAndGetFloat("ransac_thr", ransac_thr);
    
    Intrinsics intr(fx, fy, cx, cy, baseline);
    StereoOpticalFlowVisualOdometry vo(intr, ransac_thr);

    loader.loadStereoSequence(root_path, kitti_seq, use_color_camera);

    for(size_t i = 0; i < loader.num_pairs_; i++)
    {
    	//Load stereo pair
        left = loader.getNextLeftImage(false);
        right = loader.getNextRightImage(false);

        //Estimate current camera pose
        vo.computeCameraPose(left, right);

        //View tracked points
		for(size_t k = 0; k < vo.tracker_.curr_pts_.size(); k++)
		{
			Point2i pt1 = vo.tracker_.prev_pts_[k];
			Point2i pt2 = vo.tracker_.curr_pts_[k];
			circle(left, pt1, 1, CV_RGB(255,0,0), -1);
			circle(left, pt2, 3, CV_RGB(0,0,255), -1);
			line(left, pt1, pt2, CV_RGB(0,0,255));
		}

		if(i == 0) visualizer.addReferenceFrame(vo.pose_, "origin");
		//visualizer.addQuantizedPointCloud(vo.curr_dense_cloud_, 0.1, vo.pose_);
		//visualizer.addPointCloud(vo.curr_dense_cloud_, vo.pose_);
		visualizer.viewReferenceFrame(vo.pose_);
		//visualizer.viewPointCloud(vo.curr_dense_cloud_, vo.pose_);
		visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.5, vo.pose_);
		//visualizer.addCameraPath(vo.pose_);

		visualizer.spinOnce();

        //Show images
		imshow("Left Image", left);
		
		char key = waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}
    }

    visualizer.close();

	return 0;
}