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
    started_ = false;

    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver =
        g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

    optimizer_.setAlgorithm(solver);
    optimizer_.setVerbose(true);
}

void PoseGraphSLAM::addVertex(const Eigen::Affine3f &pose, const int &id,
                              const bool &fixed_vertex)
{
    Eigen::Isometry3d est(pose.matrix().cast<double>());

    VertexSE3* v = new VertexSE3;
    v->setId(id);
    v->setEstimate(est);
    v->setFixed(fixed_vertex);
    optimizer_.addVertex(v);
    added_vertices_.insert(v);

    MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::addVertex: "
                                    "id %lu (fixed: %i) "
                                    "pos. %f %f %f\n",
                                     id, fixed_vertex, 
                                     v->estimate()(0,3),
                                     v->estimate()(1,3),
                                     v->estimate()(2,3));
}

void PoseGraphSLAM::addEdge(const int &from_id, const int &to_id,
                            const Eigen::Isometry3d &transform)
{
    VertexSE3* v0 = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(from_id));
    VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(to_id));

    EdgeSE3* e = new EdgeSE3;
    e->vertices()[0] = v0;
    e->vertices()[1] = v1;
    e->setMeasurement(transform);
    e->information() = Eigen::Matrix<double, 6, 6>::Identity();
    optimizer_.addEdge(e);
    added_edges_.insert(e);

    MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::addEdge: "
                                    "from %lu to %lu "
                                    "with rel. translation [%f %f %f]\n",
                                    from_id, to_id,
                                    e->measurement()(0,3),
                                    e->measurement()(1,3),
                                    e->measurement()(2,3));
}

void PoseGraphSLAM::addOdometryEdge(const int &id)
{
    VertexSE3* v0 = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(id-1));
    VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(id));

    addEdge(id-1, id, relativeTransform(v0->estimate(), v1->estimate()));
}

Eigen::Affine3f PoseGraphSLAM::getVertexPose(const int &id)
{
    for(HyperGraph::VertexIDMap::const_iterator it = optimizer_.vertices().begin();
         it != optimizer_.vertices().end(); it++)
    {
        VertexSE3* v = dynamic_cast<VertexSE3*>(it->second);

        if(v->id() == id)
        {
            return isometryToAffine(v->estimate());
        }
    }

    MLOG_WARN(EventLogger::M_SLAM, "@PoseGraphSLAM::getVertexPose: "
                                   "vertex %lu not found. "
                                   "Returning identity.\n");
    return Eigen::Affine3f::Identity();
}

void PoseGraphSLAM::getEdgeEndpoints(const int &from_id, const int &to_id,
                                     Eigen::Vector3d& from, Eigen::Vector3d& to)
{
    for(HyperGraph::EdgeSet::const_iterator it = optimizer_.edges().begin();
        it != optimizer_.edges().end(); it++)
    {
        EdgeSE3* e = dynamic_cast<EdgeSE3*>(*it);
        if (e->vertex(0)->id() == from_id && e->vertex(1)->id() == to_id)
        {
            VertexSE3* v0 = static_cast<VertexSE3*>(optimizer_.vertex(e->vertex(0)->id()));
            VertexSE3* v1 = static_cast<VertexSE3*>(optimizer_.vertex(e->vertex(1)->id()));

            from = Eigen::Vector3d(v0->estimate()(0, 3),
                                   v0->estimate()(1, 3),
                                   v0->estimate()(2, 3));
            to = Eigen::Vector3d(v1->estimate()(0, 3),
                                 v1->estimate()(1, 3),
                                 v1->estimate()(2, 3));

            MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::getEdge(%lu -> %lu): "
                       "from(%f %f %f) to(%f %f %f)\n", from_id, to_id,
                       from(0), from(1), from(2), to(0), to(1), to(2));
        }
    }
}


void PoseGraphSLAM::optimizeGraph(const int& k)
{
    MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::optimizeGraph: "
               "%lu vertices and %lu edges\n",
               optimizer_.vertices().size(), optimizer_.edges().size());
    MLOG_DEBUG(EventLogger::M_SLAM, "@PoseGraphSLAM::optimizeGraph: "
               "active %lu vertices and %lu edges\n",
               optimizer_.activeVertices().size(), optimizer_.activeEdges().size());

    bool psd_matrices = optimizer_.verifyInformationMatrices(true);
    if(!psd_matrices)
    {
        MLOG_WARN(EventLogger::M_SLAM, "@PoseGraphSLAM::optimizeGraph: "
                  "Some of the information matrices are not Positive Semidefinite. "
                  "The optimization will not converge!");
    }

    if(!started_)
    {
        bool init_ok = optimizer_.initializeOptimization();
        if(!init_ok)
        {
            MLOG_ERROR(EventLogger::M_SLAM, "@PoseGraphSLAM::optimizeGraph: "
                       "failed to initialize optimization.\n");
            exit(0);
        }
        started_ = true;
    }
    else
    {
        bool update_ok = optimizer_.updateInitialization(added_vertices_, added_edges_);
        if(!update_ok)
        {
            MLOG_ERROR(EventLogger::M_SLAM, "@PoseGraphSLAM::optimizeGraph: "
                       "failed to update initialization.\n");
            exit(0);
        }
        added_vertices_.clear();
        added_vertices_.clear();
    }

    optimizer_.optimize(k); //TODO: maybe experiment with the 2nd. param = true?
}

void PoseGraphSLAM::printGraph()
{
    MLOG_INFO(EventLogger::M_SLAM, "@PoseGraphSLAM::printGraph (all edges):\n");

    for(HyperGraph::EdgeSet::const_iterator it = optimizer_.edges().begin();
        it != optimizer_.edges().end(); it++)
    {
        EdgeSE3* e = dynamic_cast<EdgeSE3*>(*it);
        e->computeError();

        const size_t from_id = e->vertex(0)->id();
        const size_t to_id = e->vertex(1)->id();

        MLOG_INFO(EventLogger::M_SLAM, "edge(%lu -> %lu)\n", from_id, to_id);

        //VertexSE3* v0 = static_cast<VertexSE3*>(optimizer_.vertex(from_id));
        //VertexSE3* v1 = static_cast<VertexSE3*>(optimizer_.vertex(to_id));
        //MLOG_DEBUG(EventLogger::M_SLAM, "from pose:\n");
        //printTransform(v0->estimate());
        //MLOG_DEBUG(EventLogger::M_SLAM, "to pose:\n");
        //printTransform(v1->estimate());
        //MLOG_DEBUG(EventLogger::M_SLAM, "rel. transform:\n");
        //printTransform(e->measurement());
        MLOG_INFO(EventLogger::M_SLAM, "chi2 error: %f\n", e->chi2());
    }
}