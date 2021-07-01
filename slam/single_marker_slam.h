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
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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

#ifndef INCLUDE_SingleMarkerSlam_H_
#define INCLUDE_SingleMarkerSlam_H_

#include <vector>

#include <Eigen/Geometry>

#include <g2o/core/sparse_optimizer.h>

#include <common_types.h>
#include <event_logger.h>

class SingleMarkerSlam
{
private:
    // Id of the last added vertex (used when adding edges)
    int last_added_id_;

    // G2O nonlinear optimizer
    g2o::SparseOptimizer optimizer_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Number of vertices in the graph
    int num_vertices_;

    // Number of loop edges
    int num_loop_edges_;

    // Vertices added
    g2o::HyperGraph::VertexSet vertices_;

    // Edges added
    g2o::HyperGraph::EdgeSet edges_;

    // Loop closure edges name
    std::vector<std::string> loop_closure_edges_name_;

    std::vector<Eigen::Affine3f> optimized_estimates_;

    // Default constructor
    SingleMarkerSlam();

    /**
     * Adds a vertex to the graph with the given pose and id
     * and also an "odometry" edge connecting it to the previous
     * vertex in the graph
     * @param pose pose to be added
     * @param id pose id
     */
    void addVertexAndEdge(const Eigen::Affine3f& pose, const int& id);

    /**
     * Adds a "loop closing" edge connecting the vertex with given id to the
     * origin (vertex with id = 0)
     * @param vertex_to_origin_transformation transformation from the vertex to origin
     * @param id vertex id
     */
    void addLoopClosingEdge(const Eigen::Affine3f& vertex_to_origin_transf, const int& id);

    /**
     * Returns an Optimized Edge (transformation between two keyframes if it was already optimized by graph)
     * @param from_id edge from id
     * @param to_id edge to id
     * @param from (output parameter) 3D position of the edge "from" endpoint
     * @param to (output parameter) 3D position of the edge "to" endpoint
     * @param name (output parameter) string where the name of edge will be saved
     */
    void getOptimizedEdge(
        const int& from_id,
        const int& to_id,
        Eigen::Vector3d& from,
        Eigen::Vector3d& to,
        std::string& name);

    /**
     * Returns an edge (transformation between two keyframes)
     * @param from_id the id of the "back" of the edge arrow
     * @param to_id the id of the "point" of the edge arrow
     * @param from (output parameter) 3D position of the edge "from" endpoint
     * @param to (output parameter) 3D position of the edge "to" endpoint
     * @param name (output parameter) string where the name of edge will be saved
     */
    void getEdge(const int& from_id, const int& to_id, Eigen::Vector3d& from, Eigen::Vector3d& to, std::string& name);
    Eigen::Affine3f getVertex(const int& id);

    Eigen::Affine3f getOptimizedVertex(const int& id);

    /**
     * Returns the the last edge, if it is an odometry it will return the edge from positions_.size() - 2 to
     * positions.size() - 1 If it is an loop it will return 0 to positions.size() - 1
     * @return Edge
     */
    // void getLastEdge(Eigen::Vector3d& from, Eigen::Vector3d& to, std::string& name);

    /**
     * Optimizes the graph with Levenberg-Marquardt for the given n. of iterations
     * @param k number of iterations type int
     */
    void optimizeGraph(const int& k);

    /**
     * Reset graph
     */
    void resetGraph();

    /**
     * Prints all vertices and edges currently in
     * the optimizer.
     * Useful for debugging.
     */
    void printGraph();
};

#endif /* INCLUDE_SLAM_H_ */
