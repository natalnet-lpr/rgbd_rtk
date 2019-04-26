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

#include <kitti_stereo_loader.h>
#include <stereo_cloud_generator.h>
#include <reconstruction_visualizer.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    string root_path;
    KITTIStereoLoader loader;
    StereoCloudGenerator cg(1);
    Mat left, right, true_disp;
    Mat Q = (cv::Mat_<float>(4,4) <<1, 0, 0, -607.1928, 
                                    0, 1, 0, -185.2157,
                                    0, 0, 0,   718.856,
                                    0, 0, 1/0.54, 0);
    ReconstructionVisualizer visualizer;
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();

    if(argc != 4)
    {
        fprintf(stderr, "Usage: %s <kitti datasets root dir.> <sequence_number> <use_color_camera>\n", argv[0]);
        fprintf(stderr, "Exiting.\n");
        exit(0);
    }

    root_path = argv[1];
    loader.loadStereoSequence(root_path, 10, false);

    for(size_t i = 0; i < loader.num_pairs_; i++)
    {
        left = loader.getNextLeftImage(false);
        right = loader.getNextRightImage(false);

        true_disp = cg.generateDisparityMap(left, right);
        cg.generatePointCloud(left, right, Q);

        //Adjust disparity image for visualization purposes
        double min, max;
        minMaxLoc(true_disp, &min, &max);
        true_disp = true_disp/max;

        //View images
        imshow("Left Image", left);
        imshow("Right Image", right);
        imshow("Disparity", true_disp);

        //View point cloud
        if(i == 0) visualizer.addReferenceFrame(pose, "origin");
        visualizer.viewQuantizedPointCloud(cg.cloud_, 0.5, pose);

        visualizer.spinOnce();

        char key = waitKey(16);
        if(key == 'q' || key == 'Q' || key == 27)
        {
            break;
        }
    }

    return 0;
}

