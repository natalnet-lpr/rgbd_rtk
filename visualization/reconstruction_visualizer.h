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

#ifndef INCLUDE_RECONSTRUCTION_VISUALIZER_H_
#define INCLUDE_RECONSTRUCTION_VISUALIZER_H_

#include <cstring>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/boost.h>

#include <common_types.h>

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLVisualizerPtr;

class ReconstructionVisualizer
{
private:
	//Number of point clouds added to the visualizer
	int num_clouds_;

	//Number of lines added to the visualizer
	int num_lines_;

	//Number of reference frames added to the visualizer
	int num_ref_frames_;

	//PointCloudsLibrary 3D visualizer (only works with a pointer to the object)
	PCLVisualizerPtr viewer_;

public:
	//Default constructor
	ReconstructionVisualizer();

	//Constructor with the window title as a string
	ReconstructionVisualizer(const std::string title);

	//Adds a ref. frame with the given pose to the 3D reconstruction
	void addReferenceFrame(const Eigen::Affine3f pose, const std::string text);

	//Adds a point cloud with the given pose to the 3D reconstruction
	void addPointCloud(const pcl::PointCloud<PointT>::Ptr cloud, const Eigen::Affine3f pose);

	//Adds a point cloud after quantization (uniform sampling) with the given pose to the 3D reconstruction
	void addQuantizedPointCloud(const pcl::PointCloud<PointT>::Ptr cloud, const float radius, const Eigen::Affine3f pose);

	//Views a ref. frame with the given pose in the 3D reconstruction
	void viewReferenceFrame(const Eigen::Affine3f pose, const std::string text="cam");

	//Views a point cloud in the 3D reconstruction
	void viewPointCloud(const pcl::PointCloud<PointT>::Ptr cloud, const Eigen::Affine3f pose);

	//Views a point cloud after quantization (uniform sampling) in the 3D reconstruction
	void viewQuantizedPointCloud(const pcl::PointCloud<PointT>::Ptr cloud, const float radius, const Eigen::Affine3f pose);

	//Wrapper to the interactor function of pcl::visualizer
	void spin();

	//Wrapper to the interactor function of pcl::visualizer
	void spinOnce();
};

#endif /* INCLUDE_RECONSTRUCTION_VISUALIZER_H_ */