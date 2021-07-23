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

#ifndef INCLUDE_POSEGRAPHSLAM_H_
#define INCLUDE_POSEGRAPHSLAM_H_

#include <vector>

#include <Eigen/Geometry>

#include <g2o/core/sparse_optimizer.h>

/**
 * Class used to solve incarnations of
 * SLAM modeled with PoseGraphs.
 */
class PoseGraphSLAM
{
private:

    // G2O nonlinear optimizer
    g2o::SparseOptimizer optimizer_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Vertices added
    g2o::HyperGraph::VertexSet vertices_;

    // Edges added
    g2o::HyperGraph::EdgeSet edges_;

    /** 
     * Constructs a pose graph SLAM with
     * default parameters.
     */
    PoseGraphSLAM();

    /**
     * Adds a vertex to the graph with the given pose and id.
     * The vertex can be optionally set to fixed.
     * @param pose: vertex pose
     * @param id: vertex id
     * @param fixed_vertex: if the vertex should be fixed
     */
    void addVertex(const Eigen::Affine3f &pose, const int &id,
                   const bool &fixed_vertex = false);

    /**
     * Adds an edge to the graph that connects the
     * vertices of given ids by the given relative transform
     * (measured in the from_vertex ref. frame)
     * @param from_id: vertex id of the from_vertex
     * @param to_id: vertex id of the to_vertex
     * @param transform: 3D rigid transformation from the from_vertex
     *                   to the to_vertex
     */
    void addEdge(const int &from_id, const int &to_id,
                 const Eigen::Isometry3d &transform);

    /**
     * Adds an odometry edge to the graph.
     * An odometry edge connects two nodes with consecutive ids.
     * @param id: vertex id of the to_vertex (the vertex id of the
                  from_vertex is obtained from this value)
     */
    void addOdometryEdge(const int &id);

    /**
     * Returns the pose of a vertex of given id.
     * @param id: vertex id of the vertex
     */
    Eigen::Affine3f getVertexPose(const int &id);

    /**
     * Returns the two endpoints (3D positions of "from" and "to" vertices)
     * of an edge.
     * @param from_id: id of the "from" endpoint
     * @param to_id: id of the "to" endpoint
     * @param from (output parameter): 3D position of the "from" vertex
     * @param to (output parameter): 3D position of the "to" vertex
     */
    void getEdgeEndpoints(const int &from_id, const int &to_id,
                          Eigen::Vector3d& from, Eigen::Vector3d& to);

    /**
     * Optimizes the graph with Levenberg-Marquardt for the given n. of iterations
     * @param k number of iterations type int
     */
    void optimizeGraph(const int& k);

    /**
     * Prints all vertices and edges currently in
     * the optimizer.
     * Useful for debugging.
     */
    void printGraph();
};

#endif /* INCLUDE_POSEGRAPHSLAM_H_ */
