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

#include <iostream>
#include <sstream>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include <common_types.h>
#include <single_marker_slam.h>

using namespace std;
using namespace g2o;

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

void SingleMarkerSlam::updateState()
{
    optimized_estimates_.clear();
    optimized_estimates_.resize(positions_.size());

    MLOG_DEBUG(EventLogger::M_SLAM, "@SingleMarkerSlam::updateState: Updating %lu vertices\n", positions_.size());

    // Update estimates of all vertices and positions
    for (OptimizableGraph::VertexIDMap::const_iterator it = optimizer_.vertices().begin();
         it != optimizer_.vertices().end();
         it++)
    {
        int v_id = static_cast<int>(it->first);
        VertexSE3* v = static_cast<VertexSE3*>(it->second);

        // Get a new estimation
        Eigen::Isometry3f new_estimate_tmp = v->estimate().cast<float>();
        Eigen::Affine3f new_estimate = Eigen::Affine3f::Identity();

        // Save the new estimation to new_estimate
        new_estimate(0, 0) = new_estimate_tmp(0, 0);
        new_estimate(0, 1) = new_estimate_tmp(0, 1);
        new_estimate(0, 2) = new_estimate_tmp(0, 2);
        new_estimate(1, 0) = new_estimate_tmp(1, 0);
        new_estimate(1, 1) = new_estimate_tmp(1, 1);
        new_estimate(1, 2) = new_estimate_tmp(1, 2);
        new_estimate(2, 0) = new_estimate_tmp(2, 0);
        new_estimate(2, 1) = new_estimate_tmp(2, 1);
        new_estimate(2, 2) = new_estimate_tmp(2, 2);
        new_estimate(0, 3) = new_estimate_tmp(0, 3);
        new_estimate(1, 3) = new_estimate_tmp(1, 3);
        new_estimate(2, 3) = new_estimate_tmp(2, 3);

        optimized_estimates_[v_id] = new_estimate;

        MLOG_DEBUG(EventLogger::M_SLAM, "@SingleMarkerSlam::updateState: Updating node %i\n", v_id);
        MLOG_DEBUG(
            EventLogger::M_SLAM,
            "@SingleMarkerSlam::updateState: ### from %f %f %f to %f %f %f\n",
            positions_[v_id].x(),
            positions_[v_id].y(),
            positions_[v_id].z(),
            new_estimate(0, 3),
            new_estimate(1, 3),
            new_estimate(2, 3));

        positions_[v_id](0, 0) = new_estimate(0, 3);
        positions_[v_id](1, 0) = new_estimate(1, 3);
        positions_[v_id](2, 0) = new_estimate(2, 3);
    }

    MLOG_DEBUG(EventLogger::M_SLAM, "@SingleMarkerSlam::updateState: Loop closing edges: %lu\n", num_loop_edges_);
}

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

SingleMarkerSlam::SingleMarkerSlam()
{
    num_vertices_ = 0;
    last_added_id_ = -1;
    num_loop_edges_ = 0;
    loop_closure_edges_name_ = {};

    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =
        g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

    optimizer_.setAlgorithm(solver);
    optimizer_.setVerbose(true);
}

void SingleMarkerSlam::addVertexAndEdge(const Eigen::Affine3f& pose, const int& id)
{

    MLOG_DEBUG(EventLogger::M_SLAM, "@SingleMarkerSlam::addVertexAndEdge: Adding node %i\n", id);

    // Add the first node and fix it.
    if (num_vertices_ == 0)
    {
        Eigen::Isometry3d est(pose.matrix().cast<double>());

        VertexSE3* v0 = new VertexSE3;
        v0->setId(id);
        v0->setEstimate(est);
        v0->setFixed(true);
        optimizer_.addVertex(v0);
        vertices_.insert(v0);

        Eigen::Vector3d pos = est.translation();
        positions_.push_back(pos);

        MLOG_DEBUG(
            EventLogger::M_SLAM,
            "@SingleMarkerSlam::addVertexAndEdge: position (%f, %f, %f)\n",
            pos[0],
            pos[1],
            pos[2]);
    }
    // When adding any nodes other than the first, add an edge connecting
    // to the previous one
    else
    {
        VertexSE3* v0 = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(last_added_id_));

        Eigen::Isometry3d est(pose.matrix().cast<double>());

        VertexSE3* v1 = new VertexSE3;
        v1->setId(id);
        v1->setEstimate(est);
        optimizer_.addVertex(v1);
        vertices_.insert(v1);

        Eigen::Vector3d pos = est.translation();
        positions_.push_back(pos);

        EdgeSE3* e = new EdgeSE3;
        e->vertices()[0] = v0;
        e->vertices()[1] = v1;
        e->setMeasurementFromState();
        optimizer_.addEdge(e);
        edges_.insert(e);

        MLOG_DEBUG(
            EventLogger::M_SLAM,
            "@SingleMarkerSlam::addVertexAndEdge: position (%f, %f, %f)\n",
            pos[0],
            pos[1],
            pos[2]);
        MLOG_DEBUG(
            EventLogger::M_SLAM, "@SingleMarkerSlam::addVertexAndEdge: Adding edge(%lu -> %lu)\n", v0->id(), v1->id());
    }

    last_added_id_ = id;
    num_vertices_++;
}

void SingleMarkerSlam::addLoopClosingEdge(const Eigen::Affine3f& vertex_to_origin_transf, const int& id)
{
    // assumes the first vertex has id = 0
    VertexSE3* origin = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(0));
    VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(id));

    Eigen::Isometry3d measurement(vertex_to_origin_transf.matrix().cast<double>());

    EdgeSE3* e = new EdgeSE3;
    e->vertices()[0] = v;
    e->vertices()[1] = origin;
    e->setMeasurement(measurement);
    optimizer_.addEdge(e);
    edges_.insert(e);

    num_loop_edges_++;

    MLOG_DEBUG(
        EventLogger::M_SLAM, "@SingleMarkerSlam::addLoopClosingEdge: adding edge(%lu -> %lu)\n", v->id(), origin->id());
}

void SingleMarkerSlam::getOptimizedEdge(
    const int& from_id,
    const int& to_id,
    Eigen::Vector3d& from,
    Eigen::Vector3d& to,
    string& name)
{
    for (auto it = optimizer_.activeEdges().begin(); it != optimizer_.activeEdges().end(); ++it)
    {
        EdgeSE3* e = dynamic_cast<EdgeSE3*>(*it);
        if (e->vertex(0)->id() == from_id and e->vertex(1)->id() == to_id)
        {
            VertexSE3* v0 = static_cast<VertexSE3*>(optimizer_.vertex(e->vertex(0)->id()));
            VertexSE3* v1 = static_cast<VertexSE3*>(optimizer_.vertex(e->vertex(1)->id()));

            Eigen::Isometry3f vertix_from_tmp = v0->estimate().cast<float>();
            Eigen::Affine3f vertix_from = Eigen::Affine3f::Identity();
            vertix_from.matrix() = vertix_from_tmp.matrix();

            Eigen::Isometry3f vertix_to_tmp = v1->estimate().cast<float>();
            Eigen::Affine3f vertix_to = Eigen::Affine3f::Identity();
            vertix_to.matrix() = vertix_to_tmp.matrix();

            from = Eigen::Vector3d(vertix_from(0, 3), vertix_from(1, 3), vertix_from(2, 3));
            to = Eigen::Vector3d(vertix_to(0, 3), vertix_to(1, 3), vertix_to(2, 3));
            name = "optimized_edge_" + to_string(e->vertex(0)->id()) + "_" + to_string(e->vertex(1)->id());
        }
    }
}

void SingleMarkerSlam::getEdge(
    const int& from_id,
    const int& to_id,
    Eigen::Vector3d& from,
    Eigen::Vector3d& to,
    string& name)
{

    for (auto it = edges_.begin(); it != edges_.end(); ++it)
    {
        EdgeSE3* e = dynamic_cast<EdgeSE3*>(*it);
        if (e->vertex(0)->id() == from_id and e->vertex(1)->id() == to_id)
        {
            VertexSE3* v0 = static_cast<VertexSE3*>(optimizer_.vertex(e->vertex(0)->id()));
            VertexSE3* v1 = static_cast<VertexSE3*>(optimizer_.vertex(e->vertex(1)->id()));

            Eigen::Isometry3f vertix_from_tmp = v0->estimate().cast<float>();
            Eigen::Affine3f vertix_from = Eigen::Affine3f::Identity();
            vertix_from.matrix() = vertix_from_tmp.matrix();

            Eigen::Isometry3f vertix_to_tmp = v1->estimate().cast<float>();
            Eigen::Affine3f vertix_to = Eigen::Affine3f::Identity();
            vertix_to.matrix() = vertix_to_tmp.matrix();

            from = Eigen::Vector3d(vertix_from(0, 3), vertix_from(1, 3), vertix_from(2, 3));
            to = Eigen::Vector3d(vertix_to(0, 3), vertix_to(1, 3), vertix_to(2, 3));
            name = "edge_" + to_string(e->vertex(0)->id()) + "_" + to_string(e->vertex(1)->id());
        }
    }
}

Eigen::Affine3f SingleMarkerSlam::getVertix(const int& id)
{
    for (auto it = optimizer_.activeVertices().begin(); it != optimizer_.activeVertices().end(); ++it)
    {
        VertexSE3* v = dynamic_cast<VertexSE3*>(*it);

        if (v->id() == id)
        {
            Eigen::Isometry3f pose_tmp = v->estimate().cast<float>();
            Eigen::Affine3f pose = Eigen::Affine3f::Identity();
            pose.matrix() = pose_tmp.matrix();
            return pose;
        }
    }
}
/*

void SingleMarkerSlam::getLastEdge(Eigen::Vector3d& from, Eigen::Vector3d& to, string& name)
{
    for (auto it = --edges_.end(); it != edges_.end(); ++it)
    {
        EdgeSE3* e = dynamic_cast<EdgeSE3*>(*it);

        cout << "Valor do last edge " << optimizer_.vertex(e->vertex(0)->id()) << " Valor do id 2 last edge "
             << optimizer_.vertex(e->vertex(1)->id()) << endl;

        VertexSE3* v0 = static_cast<VertexSE3*>(optimizer_.vertex(e->vertex(0)->id()));
        VertexSE3* v1 = static_cast<VertexSE3*>(optimizer_.vertex(e->vertex(1)->id()));

        Eigen::Isometry3f vertix_from_tmp = v0->estimate().cast<float>();
        Eigen::Affine3f vertix_from = Eigen::Affine3f::Identity();
        vertix_from.matrix() = vertix_from_tmp.matrix();

        Eigen::Isometry3f vertix_to_tmp = v1->estimate().cast<float>();
        Eigen::Affine3f vertix_to = Eigen::Affine3f::Identity();
        vertix_to.matrix() = vertix_to_tmp.matrix();

        from = Eigen::Vector3d(vertix_from(0, 3), vertix_from(1, 3), vertix_from(2, 3));
        to = Eigen::Vector3d(vertix_to(0, 3), vertix_to(1, 3), vertix_to(2, 3));
        name = "edge_" + to_string(e->vertex(0)->id()) + "_" + to_string(e->vertex(1)->id());
    }
}
*/
void SingleMarkerSlam::optimizeGraph(const int& k)
{
    // optimizer_.save("graph.g2o"); // Save file

    optimized_estimates_.size() == 0 ? optimizer_.initializeOptimization()
                                     : optimizer_.updateInitialization(vertices_, edges_);

    // When I pass only the subset the optimization may go wrong
    // vertices_.clear();
    // edges_.clear();
    optimizer_.optimize(k);

    updateState();
}

void SingleMarkerSlam::resetGraph()
{
    num_vertices_ = 0;
    last_added_id_ = -1;
    positions_.clear();
    optimized_estimates_.clear();
    optimizer_.clear();
    num_loop_edges_ = 0;
    loop_closure_edges_name_ = {};
}
