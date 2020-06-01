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
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>

#include <reconstruction_visualizer.h>

using namespace std;

ReconstructionVisualizer::ReconstructionVisualizer()
{
	num_clouds_ = 0;
	num_lines_ = 0;
	num_ref_frames_ = 0;
	prev_pos_ = pcl::PointXYZ();
	curr_pos_ = pcl::PointXYZ();

	//Setup PCL visualizer
	viewer_ = PCLVisualizerPtr(new pcl::visualization::PCLVisualizer("3D Reconstruction"));
	viewer_->setBackgroundColor(0.6, 0.6, 0.6);
	viewer_->initCameraParameters();
	viewer_->setSize(640, 480);
	viewer_->setCameraPosition(0.5, 0.5, -1.5,	//pos
							   0.5, 0.5, 0.0,	//view
							   0.0, -1.0, 0.0); //up;
}

ReconstructionVisualizer::ReconstructionVisualizer(const std::string &title)
{
	num_clouds_ = 0;
	num_lines_ = 0;
	num_ref_frames_ = 0;

	//Setup PCL visualizer
	viewer_ = PCLVisualizerPtr(new pcl::visualization::PCLVisualizer(title));
	viewer_->setBackgroundColor(0.6, 0.6, 0.6);
	viewer_->initCameraParameters();
	viewer_->setSize(640, 480);
	viewer_->setCameraPosition(0.5, 0.5, -1.5,	//pos
							   0.5, 0.5, 0.0,	//view
							   0.0, -1.0, 0.0); //up;
}

void ReconstructionVisualizer::addCameraPath(const Eigen::Affine3f &pose)
{
	curr_pos_.x = pose(0, 3);
	curr_pos_.y = pose(1, 3);
	curr_pos_.z = pose(2, 3);

	stringstream line_name;
	line_name << "line" << num_lines_++;
	viewer_->addLine(prev_pos_, curr_pos_, 1.0, 0.0, 0.0, line_name.str());
	viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str());

	prev_pos_ = curr_pos_;
}

void ReconstructionVisualizer::addReferenceFrame(const Eigen::Affine3f &pose, const std::string &text)
{
	stringstream frame_name;
	frame_name << "ref" << num_ref_frames_;
	viewer_->addCoordinateSystem(0.3, pose, frame_name.str());
	num_ref_frames_++;
	PointT text_pose;
	text_pose.x = pose(0, 3) + 0.02;
	text_pose.y = pose(1, 3) + 0.05;
	text_pose.z = pose(2, 3);
	frame_name << "_text";
	viewer_->addText3D(text, text_pose, 0.025, 1, 1, 1, frame_name.str());
}

void ReconstructionVisualizer::addKeyFrame(const Eigen::Affine3f &pose, const std::string &text)
{
	stringstream frame_name;
	frame_name << "key" << num_keyframes_;
	viewer_->addCoordinateSystem(0.3, pose, frame_name.str());
	num_keyframes_++;
	PointT text_pose;
	text_pose.x = pose(0, 3) + 0.02;
	text_pose.y = pose(1, 3) + 0.05;
	text_pose.z = pose(2, 3);
	frame_name << "_text";
	viewer_->addText3D(text, text_pose, 0.025, 0, 0, 1, frame_name.str());
}

void ReconstructionVisualizer::addPointCloud(const pcl::PointCloud<PointT>::Ptr &cloud, const Eigen::Affine3f &pose)
{
	//Create a new point cloud
	pcl::PointCloud<PointT>::Ptr transf_cloud(new pcl::PointCloud<PointT>);

	//Transform point cloud
	pcl::transformPointCloud(*cloud, *transf_cloud, pose);

	stringstream cloud_name;
	cloud_name << "cloud" << num_clouds_++;

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(transf_cloud);
	if (!viewer_->updatePointCloud<PointT>(transf_cloud, rgb, cloud_name.str()))
		viewer_->addPointCloud<PointT>(transf_cloud, rgb, cloud_name.str());
	
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(curr_cloud, 0, 0, 255);
}

void ReconstructionVisualizer::addQuantizedPointCloud(const pcl::PointCloud<PointT>::Ptr &cloud, const float &radius, const Eigen::Affine3f &pose)
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

void ReconstructionVisualizer::add_edge(const Graph_Edge &edge, const Eigen::Vector3f &color)
{
	pcl::PointXYZ from_pt(edge.m_pos0(0, 0), edge.m_pos0(1, 0), edge.m_pos0(2, 0));
	pcl::PointXYZ to_pt(edge.m_pos1(0, 0), edge.m_pos1(1, 0), edge.m_pos1(2, 0));
	cout<<"antes de arrow\n";
	viewer_->addArrow(to_pt, from_pt, color(0, 0), color(1, 0), color(2, 0), false, edge.m_name);
	cout<<"dps de arrow\n";

	num_edges_++;
}

void ReconstructionVisualizer::add_edges(const std::vector<Graph_Edge> &edges, const Eigen::Vector3f &color)
{
	for (size_t i = 0; i < edges.size(); i++)
	{
		add_edge(edges[i], color);
	}
}

void ReconstructionVisualizer::add_optimized_edges(const std::vector<Graph_Edge> &edges, const Eigen::Vector3f &color)
{
	for (size_t i = 0; i < edges.size(); i++)
	{
		const int idx0 = edges[i].m_id0;
		const int idx1 = edges[i].m_id1;

		Graph_Edge opt_edge(idx0, idx1, edges[i].m_pos0, edges[i].m_pos1);
		opt_edge.m_name = edges[i].m_name + "_opt";

		//Check if the edge is "odometry" or "loop"
		if (idx1 - idx0 == 1) add_edge(opt_edge, color);
		else add_edge(opt_edge, Eigen::Vector3f(0.0, 1.0, 1.0));

	}
}

void ReconstructionVisualizer::viewReferenceFrame(const Eigen::Affine3f &pose, const std::string &text)
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
	PointT pose_txt;
	pose_txt.x = pose(0, 3) + 0.02;
	pose_txt.y = pose(1, 3) + 0.05;
	pose_txt.z = pose(2, 3);
	stringstream ss;
	ss << text << "_text";
	viewer_->removeText3D(ss.str());
	viewer_->addText3D(text, pose_txt, 0.025, 1, 1, 1, ss.str());
}

void ReconstructionVisualizer::viewPointCloud(const pcl::PointCloud<PointT>::Ptr &cloud, const Eigen::Affine3f &pose)
{
	//Create a new point cloud
	pcl::PointCloud<PointT>::Ptr transf_cloud(new pcl::PointCloud<PointT>);

	//Transform point cloud
	pcl::transformPointCloud(*cloud, *transf_cloud, pose);

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(transf_cloud);
	if (!viewer_->updatePointCloud<PointT>(transf_cloud, rgb, "cloud"))
		viewer_->addPointCloud<PointT>(transf_cloud, rgb, "cloud");
	
}

void ReconstructionVisualizer::viewQuantizedPointCloud(const pcl::PointCloud<PointT>::Ptr &cloud, const float &radius, const Eigen::Affine3f &pose)
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
/**
void update_keyframes(const std::vector<Keyframe>& keyframes)
{
	viewer_->removePointCloud("cloud");

	printf("Updating keyframe visualization...\n");
	for(size_t i = 0; i < keyframes.size(); i++)
	{
		//Grab coord. system and cloud names
		stringstream ss;
		ss << "kf" << keyframes[i].m_idx;
		string coord_sys_name = ss.str();
		ss << "_cloud";
		string cloud_name = ss.str();

//		viewer_->removeText3D(coord_sys_name);

		//THIS SOLUTION WORKS BUT IS VERY SLOW
		//Updates ref. frame and point clouds
		
		//viewer_->removeCoordinateSystem(coord_sys_name);
		//add_coord_frame(keyframes[i].m_pose, coord_sys_name);
		//viewer_->updatePointCloud(keyframes[i].m_global_cloud, cloud_name);
		

		//THIS REMOVES AND ADDS BACK CLOUDS/REF. FRAMES
		//viewer_->removeCoordinateSystem(coord_sys_name);
		//viewer_->removePointCloud(cloud_name);
		//add_keyframe(keyframes[i]);

		Eigen::Matrix4f camera = keyframes[i].m_pose_update;
		Eigen::Affine3f rel_transf;
		rel_transf.linear()(0,0) = camera(0,0); rel_transf.linear()(0,1) = camera(0,1); rel_transf.linear()(0,2) = camera(0,2);
		rel_transf.linear()(1,0) = camera(1,0); rel_transf.linear()(1,1) = camera(1,1); rel_transf.linear()(1,2) = camera(1,2);
		rel_transf.linear()(2,0) = camera(2,0); rel_transf.linear()(2,1) = camera(2,1); rel_transf.linear()(2,2) = camera(2,2);
		rel_transf.translation() = Eigen::Vector3f(camera(0,3), camera(1,3), camera(2,3));

//		viewer_->updateCoordinateSystemPose(coord_sys_name, rel_transf);
		viewer_->updatePointCloudPose(cloud_name, rel_transf);
		PointT pos; pos.x = camera(0,3); pos.y = camera(1,3); pos.z = camera(2,3);
//		viewer_->addText3D(coord_sys_name, pos, 0.025);
	}
	printf("...done\n");
}
*/

void ReconstructionVisualizer::remove_edge(const Graph_Edge &edge)
{
	viewer_->removeShape(edge.m_name);
}

void ReconstructionVisualizer::remove_edges(const std::vector<Graph_Edge> &edges)
{
	for (size_t i = 0; i < edges.size(); i++)
	{
		remove_edge(edges[i]);
	}
}

void ReconstructionVisualizer::setCameraPosition(const float &pos_x, const float &pos_y, const float &pos_z)
{
	viewer_->setCameraPosition(pos_x, pos_y, pos_z, //eye
							   0.0, 0.0, 50.0,		//center
							   0.0, -1.0, 0.0);		//up
}

void ReconstructionVisualizer::spin()
{
	viewer_->spin();
}

void ReconstructionVisualizer::spinOnce()
{
	viewer_->spinOnce();
}
