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

#include <iostream>
#include <sstream>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include <event_logger.h>
#include <geometry.h>
#include <pose_graph_slam.h>

using namespace std;
using namespace g2o;

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

PoseGraphSLAM::PoseGraphSLAM()
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

void PoseGraphSLAM::addVertexAndEdge(const Eigen::Affine3f& pose, const int& id)
{

    MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::addVertexAndEdge: adding node %i\n", id);

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

        EdgeSE3* e = new EdgeSE3;
        e->vertices()[0] = v0;
        e->vertices()[1] = v1;
        //e->setMeasurementFromState(); //THIS DOES NOT WORK PROPERLY AND I DON'T KNOW WHY
        e->setMeasurement(relativeTransform(v0->estimate(), v1->estimate()));
        // e->information() = Eigen::Matrix<double, 6, 6>::Identity();
        optimizer_.addEdge(e);
        edges_.insert(e);

        MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::addVertexAndEdge: "
                   "adding edge(%lu -> %lu)\n", v0->id(), v1->id());

        //DEBUG ERASE ME LATER
        Eigen::Isometry3f vertex_from_tmp = v0->estimate().cast<float>();
        Eigen::Affine3f vertex_from = Eigen::Affine3f::Identity();
        vertex_from.matrix() = vertex_from_tmp.matrix();
        Eigen::Isometry3f vertex_to_tmp = v1->estimate().cast<float>();
        Eigen::Affine3f vertex_to = Eigen::Affine3f::Identity();
        vertex_to.matrix() = vertex_to_tmp.matrix();
        Eigen::Affine3f transf = relativeTransform(vertex_from, vertex_to);

        /*
        MLOG_DEBUG(EventLogger::M_SLAM, "@SingleMarkerSlam::addVertexAndEdge:"
               " adding edge(%lu -> %lu) with translation from state (%f %f %f)\n",
               v0->id(), v1->id(), transf(0,3), transf(1,3), transf(2,3));
        MLOG_DEBUG(EventLogger::M_SLAM, "same data stored in the edge:"
               " (%f %f %f)\n",
               e->measurement()(0,3), e->measurement()(1,3), e->measurement()(2,3));
        */
        MLOG_INFO(EventLogger::M_SLAM, "@PoseGraphSLAM::addVertexAndEdge: "
                  "computed odometry transform:\n");
        printTransform(transf);
        MLOG_INFO(EventLogger::M_SLAM, "@PoseGraphSLAM::addVertexAndEdge: "
                  "stored odometry transform:\n");
        printf("%f %f %f %f\n", e->measurement()(0,0), e->measurement()(0,1), e->measurement()(0,2), e->measurement()(0,3));
        printf("%f %f %f %f\n", e->measurement()(1,0), e->measurement()(1,1), e->measurement()(1,2), e->measurement()(1,3));
        printf("%f %f %f %f\n", e->measurement()(2,0), e->measurement()(2,1), e->measurement()(2,2), e->measurement()(2,3));
        printf("%f %f %f %f\n", e->measurement()(3,0), e->measurement()(3,1), e->measurement()(3,2), e->measurement()(3,3));
    }

    last_added_id_ = id;
    num_vertices_++;
}

void PoseGraphSLAM::addLoopClosingEdge(const Eigen::Affine3f& vertex_to_origin_transf, const int& id)
{
    // assumes the first vertex has id = 0
    VertexSE3* origin = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(0));
    VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(id));

    //DEBUG ERASE ME LATER
    Eigen::Isometry3f vertex_from_tmp = v->estimate().cast<float>();
    Eigen::Affine3f vertex_from = Eigen::Affine3f::Identity();
    vertex_from.matrix() = vertex_from_tmp.matrix();
    Eigen::Isometry3f vertex_to_tmp = origin->estimate().cast<float>();
    Eigen::Affine3f vertex_to = Eigen::Affine3f::Identity();
    vertex_to.matrix() = vertex_to_tmp.matrix();
    Eigen::Affine3f transf = relativeTransform(vertex_from, vertex_to);

    Eigen::Isometry3d measurement(vertex_to_origin_transf.matrix().cast<double>());

    EdgeSE3* e = new EdgeSE3;
    e->vertices()[0] = v;
    e->vertices()[1] = origin;
    e->setMeasurement(measurement);
    // e->information() = Eigen::Matrix<double, 6, 6>::Identity();
    optimizer_.addEdge(e);
    edges_.insert(e);

    num_loop_edges_++;

    MLOG_INFO(EventLogger::M_SLAM, "@PoseGraphSLAM::addLoopClosingEdge:"
              " computed lc transform:\n");
    printTransform(transf);
    MLOG_INFO(EventLogger::M_SLAM, "@PoseGraphSLAM::addLoopClosingEdge:"
              " stored lc transform:\n");
    printf("%f %f %f %f\n", e->measurement()(0,0), e->measurement()(0,1), e->measurement()(0,2), e->measurement()(0,3));
    printf("%f %f %f %f\n", e->measurement()(1,0), e->measurement()(1,1), e->measurement()(1,2), e->measurement()(1,3));
    printf("%f %f %f %f\n", e->measurement()(2,0), e->measurement()(2,1), e->measurement()(2,2), e->measurement()(2,3));
    printf("%f %f %f %f\n", e->measurement()(3,0), e->measurement()(3,1), e->measurement()(3,2), e->measurement()(3,3));
}

void PoseGraphSLAM::getOptimizedEdge(
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

void PoseGraphSLAM::getEdge(
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

            MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::getEdge(%i,%i): "
                       "from(%f %f %f) to(%f %f %f)\n", from_id, to_id,
                       from(0), from(1), from(2), to(0), to(1), to(2));
        }
    }
}

Eigen::Affine3f PoseGraphSLAM::getVertex(const int& id)
{

    for (auto it = vertices_.begin(); it != vertices_.end(); ++it)
    {
        VertexSE3* v = dynamic_cast<VertexSE3*>(*it);

        if (v->id() == id)
        {
            Eigen::Isometry3f pose_tmp = v->estimate().cast<float>();
            Eigen::Affine3f pose = Eigen::Affine3f::Identity();
            pose(0, 0) = pose_tmp(0, 0);
            pose(0, 1) = pose_tmp(0, 1);
            pose(0, 2) = pose_tmp(0, 2);
            pose(0, 3) = pose_tmp(0, 3);
            pose(1, 0) = pose_tmp(1, 0);
            pose(1, 1) = pose_tmp(1, 1);
            pose(1, 2) = pose_tmp(1, 2);
            pose(1, 3) = pose_tmp(1, 3);
            pose(2, 0) = pose_tmp(2, 0);
            pose(2, 1) = pose_tmp(2, 1);
            pose(2, 2) = pose_tmp(2, 2);
            pose(2, 3) = pose_tmp(2, 3);

            printf(">>> RETURNING VALID VERTEX POSE\n");
            return pose;
        }
    }

    printf(">>> RETURNING GARBAGE VERTEX POSE\n");
}

Eigen::Affine3f PoseGraphSLAM::getOptimizedVertex(const int& id)
{

    for (auto it = optimizer_.activeVertices().begin(); it != optimizer_.activeVertices().end(); ++it)
    {
        VertexSE3* v = dynamic_cast<VertexSE3*>(*it);

        if (v->id() == id)
        {

            Eigen::Isometry3f pose_tmp = v->estimate().cast<float>();
            Eigen::Affine3f pose = Eigen::Affine3f::Identity();

            pose(0, 0) = pose_tmp(0, 0);
            pose(0, 1) = pose_tmp(0, 1);
            pose(0, 2) = pose_tmp(0, 2);
            pose(0, 3) = pose_tmp(0, 3);
            pose(1, 0) = pose_tmp(1, 0);
            pose(1, 1) = pose_tmp(1, 1);
            pose(1, 2) = pose_tmp(1, 2);
            pose(1, 3) = pose_tmp(1, 3);
            pose(2, 0) = pose_tmp(2, 0);
            pose(2, 1) = pose_tmp(2, 1);
            pose(2, 2) = pose_tmp(2, 2);
            pose(2, 3) = pose_tmp(2, 3);
            return pose;
        }
    }
}
/*

void PoseGraphSLAM::getLastEdge(Eigen::Vector3d& from, Eigen::Vector3d& to, string& name)
{
    auto a = optimizer_.edges().end();

    EdgeSE3* e = dynamic_cast<EdgeSE3*>(*--optimizer_.edges().end());

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
    */

void PoseGraphSLAM::optimizeGraph(const int& k)
{
    MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::optimizeGraph: "
               "%lu vertices and %lu edges\n",
               optimizer_.vertices().size(), optimizer_.edges().size());
    MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::optimizeGraph: "
               "active %lu vertices and %lu edges\n",
               optimizer_.activeVertices().size(), optimizer_.activeEdges().size());

    // optimizer_.save("graph.g2o"); // Save file
    // optimizer_.initializeOptimization();
    cout << "optimized estimates" << optimized_estimates_.size() << endl;
    optimized_estimates_.size() == 0 ? optimizer_.initializeOptimization()
                                     : optimizer_.updateInitialization(vertices_, edges_);

    // When I pass only the subset the optimization may go wrong
    // vertices_.clear();
    // edges_.clear();
    optimizer_.optimize(k);
}

void PoseGraphSLAM::resetGraph()
{
    num_vertices_ = 0;
    last_added_id_ = -1;
    optimizer_.clear();
    optimized_estimates_ = {};
    num_loop_edges_ = 0;
    loop_closure_edges_name_ = {};
}

void PoseGraphSLAM::printGraph()
{
    MLOG_INFO(EventLogger::M_SLAM, "@PoseGraphSLAM::printGraph (all edges):\n");

    //for(auto it = optimizer_.activeEdges().begin(); it != optimizer_.activeEdges().end(); ++it)
    for (HyperGraph::EdgeSet::const_iterator it = optimizer_.edges().begin();
         it != optimizer_.edges().end();
         it++)
    {
        EdgeSE3* e = dynamic_cast<EdgeSE3*>(*it);
        e->computeError();

        const int v0id = e->vertex(0)->id();
        const int v1id = e->vertex(1)->id();
        
        VertexSE3* v0 = static_cast<VertexSE3*>(optimizer_.vertex(v0id));
        VertexSE3* v1 = static_cast<VertexSE3*>(optimizer_.vertex(v1id));

        string edge_type = v1id != 0 ? "odom" : "loop closure";

        MLOG_INFO(EventLogger::M_SLAM, "(%i -> %i): %s\n", v0id, v1id, edge_type.c_str());
        //MLOG_INFO(EventLogger::M_SLAM, "from pose:\n");
        //printTransform(v0->estimate());
        //MLOG_INFO(EventLogger::M_SLAM, "to pose:\n");
        //printTransform(v1->estimate());
        //MLOG_INFO(EventLogger::M_SLAM, "rel. transform:\n");
        //printTransform(e->measurement());
        MLOG_INFO(EventLogger::M_SLAM, "chi2 error: %f\n", e->chi2());
    }

    /*
    MLOG_INFO(EventLogger::M_SLAM, "@PoseGraphSLAM::printGraph (active edges):\n");

    //for(auto it = optimizer_.activeEdges().begin(); it != optimizer_.activeEdges().end(); ++it)
    for (OptimizableGraph::EdgeContainer::const_iterator it = optimizer_.activeEdges().begin();
         it != optimizer_.activeEdges().end();
         it++)
    {
        EdgeSE3* e = dynamic_cast<EdgeSE3*>(*it);
        e->computeError();

        const int v0id = e->vertex(0)->id();
        const int v1id = e->vertex(1)->id();
        
        VertexSE3* v0 = static_cast<VertexSE3*>(optimizer_.vertex(v0id));
        VertexSE3* v1 = static_cast<VertexSE3*>(optimizer_.vertex(v1id));

        string edge_type = v1id != 0 ? "odom" : "loop closure";

        MLOG_INFO(EventLogger::M_SLAM, "(%i -> %i): %s\n", v0id, v1id, edge_type.c_str());
        //MLOG_INFO(EventLogger::M_SLAM, "from pose:\n");
        //printTransform(v0->estimate());
        //MLOG_INFO(EventLogger::M_SLAM, "to pose:\n");
        //printTransform(v1->estimate());
        //MLOG_INFO(EventLogger::M_SLAM, "rel. transform:\n");
        //printTransform(e->measurement());
        MLOG_INFO(EventLogger::M_SLAM, "chi2 error: %f\n", e->chi2());
    }
    */
}