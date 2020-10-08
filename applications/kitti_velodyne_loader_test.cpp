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

#include <iostream>
#include <Eigen/Geometry>

#include <kitti_velodyne_loader.h>
#include <reconstruction_visualizer.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    string root_path;
    KITTIVelodyneLoader loader;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    ReconstructionVisualizer visualizer;
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();

    if(argc != 3)
    {
        fprintf(stderr, "Usage: %s <kitti datasets root dir.> <sequence_number>\n", argv[0]);
        fprintf(stderr, "Example: /path/to/parent/directory/of/sequences 11 - loads the KITTI LIDAR sequence number 11.\n\n");
        fprintf(stderr, "That is, if the 00, 01, 02, ... folders are within /Datasets/KITTI_Datasets_LIDAR/sequences,\n");
        fprintf(stderr, "the LIDAR sequence 11 is loaded with:\n");
        fprintf(stderr, "%s /Datasets/KITTI_Datasets_LIDAR/ 11\n", argv[0]);
        fprintf(stderr, "(the 'sequences' part of the path is added automatically and must be omitted).\n");
        exit(0);
    }

    root_path = argv[1];

    loader.loadLIDARSequence(root_path, atoi(argv[2]));

    for(size_t i = 0; i < loader.num_scans_; i++)
    {
        //Load XYZRGB monochromatic cloud from .bin file
        cloud = loader.getPointCloudXYZRGB();
        printf("Point cloud %lu has %lu points\n", i, cloud->size());

        //View point cloud
        if(i == 0) visualizer.addReferenceFrame(pose, "origin");
        visualizer.viewPointCloud(cloud, pose);

        visualizer.spinOnce();
    }

    visualizer.close();

    return 0;
}
