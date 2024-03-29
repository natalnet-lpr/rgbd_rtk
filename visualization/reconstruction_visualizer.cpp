/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2021, Natalnet Laboratory for Perceptual Robotics
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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Authors:
 *
 *  Bruno Silva
 *  Rodrigo Xavier
 */

#include <cstdio>
#include <string>
#include <sstream>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/point_cloud.h>

#include <geometry.h>
#include <reconstruction_visualizer.h>

using namespace std;

ReconstructionVisualizer::ReconstructionVisualizer()
{
    // Initialize attributs
    num_clouds_ = 0, num_lines_ = 0, num_ref_frames_ = 0, num_edges_ = 0;
    prev_pos_ = pcl::PointXYZ();
    curr_pos_ = pcl::PointXYZ();

    // Setup PCL visualizer
    viewer_ = PCLVisualizerPtr(new pcl::visualization::PCLVisualizer("3D Reconstruction"));
    viewer_->setBackgroundColor(0.6, 0.6, 0.6);
    viewer_->initCameraParameters();
    viewer_->setSize(640, 480);
    viewer_->setCameraPosition(
        0.5,
        0.5,
        -1.5, // pos
        0.5,
        0.5,
        0.0, // view
        0.0,
        -1.0,
        0.0); // up;
}

ReconstructionVisualizer::ReconstructionVisualizer(const std::string& title)
{
    // Initialize attributs
    num_clouds_ = 0, num_lines_ = 0, num_ref_frames_ = 0, num_edges_ = 0;

    // Setup PCL visualizer
    viewer_ = PCLVisualizerPtr(new pcl::visualization::PCLVisualizer(title));
    viewer_->setBackgroundColor(0.6, 0.6, 0.6);
    viewer_->initCameraParameters();
    viewer_->setSize(640, 480);
    viewer_->setCameraPosition(
        0.5,
        0.5,
        -1.5, // pos
        0.5,
        0.5,
        0.0, // view
        0.0,
        -1.0,
        0.0); // up;
}

void ReconstructionVisualizer::addCameraPath(const Eigen::Affine3f& pose)
{
    // Set camera pose
    curr_pos_.x = pose(0, 3);
    curr_pos_.y = pose(1, 3);
    curr_pos_.z = pose(2, 3);

    stringstream line_name;
    line_name << "line" << num_lines_++;
    viewer_->addLine(prev_pos_, curr_pos_, 1.0, 0.0, 0.0, line_name.str());
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str());

    prev_pos_ = curr_pos_;
}

void ReconstructionVisualizer::addReferenceFrame(const Eigen::Affine3f& pose, const std::string& text)
{
    // Add frame
    stringstream frame_name;
    frame_name << "ref" << num_ref_frames_;
    viewer_->addCoordinateSystem(0.2, pose, frame_name.str());
    num_ref_frames_++;
    // Add text
    PointT text_pos;
    text_pos.x = pose(0, 3) + 0.02;
    text_pos.y = pose(1, 3) + 0.05;
    text_pos.z = pose(2, 3);
    frame_name << "_text";
    viewer_->addText3D(text, text_pos, 0.025, 1, 1, 1, frame_name.str());
}



void ReconstructionVisualizer::addPointCloud(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const Eigen::Affine3f& pose,
    const std::string& cloud_name)
{
    // Create a new point cloud
    pcl::PointCloud<PointT>::Ptr transf_cloud(new pcl::PointCloud<PointT>);

    // Transform point cloud
    pcl::transformPointCloud(*cloud, *transf_cloud, pose);

    string name;
    stringstream cloud_name_ss;
    cloud_name_ss << "cloud" << num_clouds_++;
    if (cloud_name == "")
        name = cloud_name_ss.str();
    else
        name = cloud_name;

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(transf_cloud);
    if (!viewer_->updatePointCloud<PointT>(transf_cloud, rgb, name))
        viewer_->addPointCloud<PointT>(transf_cloud, rgb, name);

    // pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(curr_cloud, 0, 0, 255);
}

void ReconstructionVisualizer::addQuantizedPointCloud(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const float& radius,
    const Eigen::Affine3f& pose,
    const std::string& cloud_name)
{
    // Create a new point cloud
    pcl::PointCloud<PointT>::Ptr quant_cloud(new pcl::PointCloud<PointT>);

    // Quantize the point cloud
    pcl::UniformSampling<PointT> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(radius);
    uniform_sampling.filter(*quant_cloud);
    addPointCloud(quant_cloud, pose, cloud_name);
}

void ReconstructionVisualizer::addEdge(
    const Eigen::Vector3d& vertex_from,
    const Eigen::Vector3d& vertex_to,
    const string& name,
    const Eigen::Vector3f& color)
{
    // Create the "from" and "to" pcl points
    pcl::PointXYZ from_pt(vertex_from(0), vertex_from(1), vertex_from(2));
    pcl::PointXYZ to_pt(vertex_to(0), vertex_to(1), vertex_to(2));

    // Add an arrow that connects from_pt -> to_pt
    viewer_->addArrow(to_pt, from_pt, color(0, 0), color(1, 0), color(2, 0), false, name);

    // Updating the number of edges
    num_edges_++;
}

void ReconstructionVisualizer::viewReferenceFrame(const Eigen::Affine3f& pose, const std::string& text)
{
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

void ReconstructionVisualizer::viewPointCloud(
     const pcl::PointCloud<PointT>::Ptr& cloud,
     const Eigen::Affine3f& pose,
     const std::string& cloud_name)
{
    // Create a new point cloud
    pcl::PointCloud<PointT>::Ptr transf_cloud(new pcl::PointCloud<PointT>);

    // Transform point cloud
    pcl::transformPointCloud(*cloud, *transf_cloud, pose);

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(transf_cloud);
    // this line triggers a runtime error due to undefined behavior on PCL code
    if (!viewer_->updatePointCloud<PointT>(transf_cloud, rgb, cloud_name))
        viewer_->addPointCloud<PointT>(transf_cloud, rgb, cloud_name);
}

void ReconstructionVisualizer::viewQuantizedPointCloud(
     const pcl::PointCloud<PointT>::Ptr& cloud,
     const float& radius,
     const Eigen::Affine3f& pose,
     const std::string& cloud_name)
{
    // Create a new point cloud
    pcl::PointCloud<PointT>::Ptr quant_cloud(new pcl::PointCloud<PointT>);

    // Quantize the point cloud
    pcl::UniformSampling<PointT> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(radius);
    uniform_sampling.filter(*quant_cloud);

    // View the quantized point cloud
    viewPointCloud(quant_cloud, pose, cloud_name);
}

void ReconstructionVisualizer::viewKeyframe(const Keyframe &kf)
{
    // Get name of coord. system and point cloud
    string frame_name = "kf" + to_string(kf.idx_);

    // Add/update ref. frame of the keyframe
    if(!viewer_->updateCoordinateSystemPose(frame_name, kf.pose_))
    {
        // Add to origin, transform it later
        viewer_->addCoordinateSystem(0.2, Eigen::Affine3f::Identity(), frame_name);
        viewer_->updateCoordinateSystemPose(frame_name, kf.pose_);
    }

    // Add/update point cloud of the keyframe
    if(!viewer_->updatePointCloudPose(frame_name + "_cloud", kf.pose_))
    {
        // Add to origin, transform it later
        addQuantizedPointCloud(kf.local_cloud_, 0.01, Eigen::Affine3f::Identity(), frame_name + "_cloud");
        viewer_->updatePointCloudPose(frame_name + "_cloud", kf.pose_);
    }

    // Add/update text of the keyframe ---> THIS IS SLOW
    /*
    PointT text_pos;
    text_pos.x = kf.pose_(0, 3) + 0.02;
    text_pos.y = kf.pose_(1, 3) + 0.05;
    text_pos.z = kf.pose_(2, 3);
    viewer_->removeText3D(frame_name + "_text");
    viewer_->addText3D(frame_name, text_pos, 0.015, 1, 1, 1, frame_name + "_text");
    */
}

void ReconstructionVisualizer::viewKeyframes(const std::map<size_t, Keyframe> &keyframes)
{
    for(std::map<size_t, Keyframe>::const_iterator it = keyframes.begin();
        it != keyframes.end(); it++)
    {
        viewKeyframe(it->second);
    }
}

void ReconstructionVisualizer::removeEdge(const string& name)
{ 
    viewer_->removeShape(name);
}

void ReconstructionVisualizer::setCameraPosition(const float& pos_x, const float& pos_y, const float& pos_z)
{
    // Given the x y and z, set the camera position
    viewer_->setCameraPosition(
        pos_x,
        pos_y,
        pos_z, // eye
        0.0,
        0.0,
        50.0, // center
        0.0,
        -1.0,
        0.0); // up
}

void ReconstructionVisualizer::spin() { viewer_->spin(); }

void ReconstructionVisualizer::spinOnce() { viewer_->spinOnce(); }

void ReconstructionVisualizer::close() { viewer_->close(); }

void ReconstructionVisualizer::resetVisualizer()
{
    viewer_->removeAllShapes();
    viewer_->removeAllCoordinateSystems();
    viewer_->removeAllPointClouds();
}

void ReconstructionVisualizer::removePointClouds() { viewer_->removeAllPointClouds(); }

void ReconstructionVisualizer::addKeyframe(const Keyframe& kf)
{
    stringstream frame_name;
    frame_name << "kf" << kf.idx_;
    viewer_->addCoordinateSystem(0.2, kf.pose_, frame_name.str());
    addQuantizedPointCloud(kf.local_cloud_, 0.01, kf.pose_, frame_name.str() + "_cloud");
    // Add text
    PointT text_pos;
    text_pos.x = kf.pose_(0, 3) + 0.02;
    text_pos.y = kf.pose_(1, 3) + 0.05;
    text_pos.z = kf.pose_(2, 3);
    viewer_->addText3D(frame_name.str(), text_pos, 0.015, 1, 1, 1, frame_name.str() + "_text");
}

void ReconstructionVisualizer::updateKeyframe(const Keyframe& kf)
{
    // Grab names of coord. system and point cloud
    stringstream frame_name;
    frame_name << "kf" << kf.idx_;

    MLOG_DEBUG(
        EventLogger::M_VISUALIZATION,
        "@ReconstructionVisualizer::updateKeyframe: "
        "Updating keyframe %lu\n",
        kf.idx_);

    // Grab transformation from the original kf pose to its optimized kf pose
    // (point transformation and not pose transformation)
    Eigen::Affine3f rel_transf = kf.opt_pose_ * kf.pose_.inverse();

    // Update coord. system
    viewer_->updateCoordinateSystemPose(frame_name.str(), rel_transf);

    // Update point cloud
    viewer_->updatePointCloudPose(frame_name.str() + "_cloud", rel_transf);

    // Update text
    PointT text_pos;
    text_pos.x = kf.opt_pose_(0, 3) + 0.02;
    text_pos.y = kf.opt_pose_(1, 3) + 0.05;
    text_pos.z = kf.opt_pose_(2, 3);
    viewer_->removeText3D(frame_name.str() + "_text");
    viewer_->addText3D(frame_name.str(), text_pos, 0.015, 1, 0, 0, frame_name.str() + "_text");
}