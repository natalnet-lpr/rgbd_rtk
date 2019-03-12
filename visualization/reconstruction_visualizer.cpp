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
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>

#include <reconstruction_visualizer.h>

using namespace std;

ReconstructionVisualizer::ReconstructionVisualizer()
{
	num_clouds_ = 0;
	num_lines_ = 0;
	num_ref_frames_ = 0;

	//Setup PCL visualizer
	viewer_ = PCLVisualizerPtr(new pcl::visualization::PCLVisualizer("3D Reconstruction"));
	viewer_->setBackgroundColor(0.6, 0.6, 0.6);
	viewer_->initCameraParameters();
	viewer_->setSize(640, 480);
	/*viewer_->setCameraPosition(0.5, 0.5, -1.5, //pos
							   0.5, 0.5, 0.0, //view
	                           0.0, -1.0, 0.0); //up;*/
	 viewer_->setCameraPosition(0.0,-400.0,0.1,0.0,0.0,0.0,0.0,0.0,0.0);
}

ReconstructionVisualizer::ReconstructionVisualizer(const std::string title)
{
	num_clouds_ = 0;
	num_lines_ = 0;
	num_ref_frames_ = 0;

	//Setup PCL visualizer
	viewer_ = PCLVisualizerPtr(new pcl::visualization::PCLVisualizer(title));
	viewer_->setBackgroundColor(0.6, 0.6, 0.6);
	viewer_->initCameraParameters();
	viewer_->setSize(640, 480);
	viewer_->setCameraPosition(0.5, 0.5, -1.5, //pos
							   0.5, 0.5, 0.0, //view
	                           0.0, -1.0, 0.0); //up;
}

void ReconstructionVisualizer::addReferenceFrame(const Eigen::Affine3f pose, const std::string text)
{
	stringstream frame_name;
	frame_name << "ref" << num_ref_frames_;
	viewer_->addCoordinateSystem(0.3, pose, frame_name.str());
	num_ref_frames_++;
	PointT pos;
	pos.x = pose(0,3) + 0.02;
	pos.y = pose(1,3) + 0.05;
	pos.z = pose(2,3);
	frame_name << "_text";
	viewer_->addText3D(text, pos, 0.025, 1, 1, 1, frame_name.str());
}

void ReconstructionVisualizer::addPointCloud(const pcl::PointCloud<PointT>::Ptr cloud, const Eigen::Affine3f pose)
{
	//Create a new point cloud
	pcl::PointCloud<PointT>::Ptr transf_cloud(new pcl::PointCloud<PointT>);

	//Transform point cloud
	pcl::transformPointCloud(*cloud, *transf_cloud, pose);

	stringstream cloud_name;
	cloud_name << "cloud" << num_clouds_++;

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(transf_cloud);
	if(!viewer_->updatePointCloud<PointT>(transf_cloud, rgb, cloud_name.str()))
	{
		viewer_->addPointCloud<PointT>(transf_cloud, rgb, cloud_name.str());
	}
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(curr_cloud, 0, 0, 255);
}

void ReconstructionVisualizer::addQuantizedPointCloud(const pcl::PointCloud<PointT>::Ptr cloud, const float radius, const Eigen::Affine3f pose)
{
	//Create a new point cloud
	pcl::PointCloud<PointT>::Ptr quant_cloud(new pcl::PointCloud<PointT>);

	//Quantize the point cloud
	pcl::UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(radius);
	uniform_sampling.filter(*quant_cloud);
	addPointCloud(quant_cloud, pose);
}

void ReconstructionVisualizer::viewReferenceFrame(const Eigen::Affine3f pose, const std::string text)
{
	/*
	//THIS DOES NOT WORK AS EXPECTED
	if(!viewer_->updateCoordinateSystemPose(text, pose))
	{
		viewer_->addCoordinateSystem(0.3, pose, text);
	}
	*/
	viewer_->removeCoordinateSystem(text);
	viewer_->addCoordinateSystem(0.3, pose, text);
	PointT pos;
	pos.x = pose(0,3) + 0.02;
	pos.y = pose(1,3) + 0.05;
	pos.z = pose(2,3);
	stringstream ss;
	ss << text << "_text";
	viewer_->removeText3D(ss.str());
	viewer_->addText3D(text, pos, 0.025, 1, 1, 1, ss.str());
}

void ReconstructionVisualizer::viewPointCloud(const pcl::PointCloud<PointT>::Ptr cloud, const Eigen::Affine3f pose)
{
	//Create a new point cloud
	pcl::PointCloud<PointT>::Ptr transf_cloud(new pcl::PointCloud<PointT>);

	//Transform point cloud
	pcl::transformPointCloud(*cloud, *transf_cloud, pose);

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(transf_cloud);
	if(!viewer_->updatePointCloud<PointT>(transf_cloud, rgb, "cloud"))
	{
		viewer_->addPointCloud<PointT>(transf_cloud, rgb, "cloud");
	}
}

void ReconstructionVisualizer::viewQuantizedPointCloud(const pcl::PointCloud<PointT>::Ptr cloud, const float radius, const Eigen::Affine3f pose)
{
	//Create a new point cloud
	pcl::PointCloud<PointT>::Ptr quant_cloud(new pcl::PointCloud<PointT>);

	//Quantize the point cloud
	pcl::UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(radius);
	uniform_sampling.filter(*quant_cloud);
	viewPointCloud(quant_cloud, pose);
}

void ReconstructionVisualizer::spin()
{
	viewer_->spin();
}

void ReconstructionVisualizer::spinOnce()
{
	viewer_->spinOnce();
}