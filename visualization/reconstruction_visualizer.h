/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or
 *     promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Authors:
 *
 *  Bruno Silva
 *  Rodrigo Sarmento Xavier
 */

#ifndef INCLUDE_RECONSTRUCTION_VISUALIZER_H_
#define INCLUDE_RECONSTRUCTION_VISUALIZER_H_

#include <Eigen/Geometry>
#include <cstring>
#include <pcl/common/boost.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <common_types.h>
#include <event_logger.h>

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLVisualizerPtr;

class ReconstructionVisualizer
{
private:
    // Number of point clouds added to the visualizer
    int num_clouds_;

    // Number of lines added to the visualizer
    int num_lines_;

    // Number of reference frames added to the visualizer
    int num_ref_frames_;

    // Number of edged added to the visualizer
    int num_edges_;

    // PointCloudsLibrary 3D visualizer (only works with a pointer to the object)
    PCLVisualizerPtr viewer_;

    // Previous position of the camera
    pcl::PointXYZ prev_pos_;

    // Current position of the camera
    pcl::PointXYZ curr_pos_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Default constructor
     */
    ReconstructionVisualizer();

    /**
     * Constructor with the window title as a string
     * @param title Title as String
     */
    ReconstructionVisualizer(const std::string& title);

    /**
     * Adds a line between the previous and the current camera position
     * @param pose current camera pose
     */
    void addCameraPath(const Eigen::Affine3f& pose);

    /** Adds an edge from the graph to the PCLVisualizer as an arrow.
     * @param vertex_from position
     * @param vertex_to position
     * @param name name of the vertex
     * @param color of the arrow(blue as default)
     */
    void addEdge(
        const Eigen::Vector3d& vertex_from,
        const Eigen::Vector3d& vertex_to,
        const std::string& name,
        const Eigen::Vector3f& color = Eigen::Vector3f(0.0, 0.0, 1.0));

    /**
     * Adds a ref. frame with the given pose to the 3D reconstruction
     * @param pose Pose to be added
     * @param text text of the pose in visualization
     */
    void addReferenceFrame(const Eigen::Affine3f& pose, const std::string& text);

    /**
     * Adds a keyframe with the given pose to the 3D reconstruction
     * @param kf keyframe to be added
     */
    void addKeyFrame(const Keyframe& kf);

    /**
     * Adds a point cloud with the given pose to the 3D reconstruction
     * @param cloud PointCloud to be added
     * @param pose pose of the 3d Reconstruction
     * @param cloud_name name of the point cloud
     */
    void addPointCloud(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        const Eigen::Affine3f& pose,
        const std::string& cloud_name = "");

    /**
     * Receives a point cloud and quantize it (uniform sampling) with the given pose to the 3D
     * reconstruction
     * @param cloud point cloud after quantization
     * @param radius radius search of uniform sampling
     * @param pose of the point cloud
     * @param cloud_name name of the point cloud
     */
    void addQuantizedPointCloud(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        const float& radius,
        const Eigen::Affine3f& pose,
        const std::string& cloud_name = "");

    /**
     * Removes an edge from the PCLVisualizer.
     * @param name of the edge to be removed as a Edge(common_types.h)
     */
    void removeEdge(const std::string& name);

    /**
     * Views a ref. frame with the given pose in the 3D reconstruction
     * @param pose pose of the frame
     * @param text pose text in visualization ("cam" as default)
     */
    void viewReferenceFrame(const Eigen::Affine3f& pose, const std::string& text = "cam");

    /**
     * Views a point cloud in the 3D reconstruction
     * @param cloud PointCloud
     * @param pose pose of the 3d Reconstruction
     */
    void viewPointCloud(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3f& pose);

    /**
     * Views a point cloud after quantization (uniform sampling) with the given pose to the 3D
     * reconstruction
     * @param cloud point cloud after quantization
     * @param radius radius search of uniform sampling
     * @param pose of the point cloud
     */
    void viewQuantizedPointCloud(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        const float& radius,
        const Eigen::Affine3f& pose);

    /**
     * Updates a given keyframe.
     * @param keyframe keyframe to be updated
     * (assumed to be with modified pose, pointcloud, keypoints, etc.)
     */
    void updateKeyFrame(const Keyframe& kf);

    /**
     *Sets the virtual camera position in the virtual world
     * @param pos_x x camera position
     * @param pos_y y camera position
     * @param pos_z z camera position
     */
    void setCameraPosition(const float& pos_x, const float& pos_y, const float& pos_z);

    /**
     * Wrapper to the interactor function of pcl::visualizer
     */
    void spin();

    /**
     * Wrapper to the interactor function of pcl::visualizer
     */
    void spinOnce();

    /**
     * Closes the visualizer
     */
    void close();

    void resetVisualizer();

    void removeAllVertexesAndEdges();
};

#endif /* INCLUDE_RECONSTRUCTION_VISUALIZER_H_ */