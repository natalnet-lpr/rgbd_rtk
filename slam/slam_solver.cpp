/*
 * Implementation of klt_slam.h
 *
 * Author: Bruno Marques F. da Silva
 * brunomfs@gmail.com
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
#include <slam_solver.h>

using namespace std;
using namespace g2o;

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

void SLAM_Solver::updateState()
{
    optimized_estimates_.clear();
    optimized_estimates_.resize(positions_.size());

    logger.print(EventLogger::L_INFO, "[SLAM_Solver::updateState] DEBUG: Updating %lu vertices\n", positions_.size());

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

        logger.print(EventLogger::L_INFO, "[SLAM_Solver::updateState] DEBUG: Updating node %i \n", v_id);
        logger.print(
            EventLogger::L_INFO,
            "[SLAM_Solver::updateState] DEBUG: ### from %f %f %f to %f %f %f\n",
            positions_[v_id].x(),
            positions_[v_id].y(),
            positions_[v_id].z(),
            new_estimate(0, 3),
            new_estimate(1, 3),
            new_estimate(2, 3),
            v_id);

        positions_[v_id](0, 0) = new_estimate(0, 3);
        positions_[v_id](1, 0) = new_estimate(1, 3);
        positions_[v_id](2, 0) = new_estimate(2, 3);
    }
}

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

SLAM_Solver::SLAM_Solver()
{
    num_vertices_ = 0;
    last_added_id_ = -1;
    num_loop_edges_ = 0;

    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =
        g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

    optimizer_.setAlgorithm(solver);
    optimizer_.setVerbose(true);
}

void SLAM_Solver::addVertexAndEdge(const Eigen::Affine3f& pose, const int& id)
{
    logger.print(EventLogger::L_INFO, "[SLAM_Solver::addVertexAndEdge] DEBUG: Adding node %i\n", id);

    // Add the first node and fix it.
    if (num_vertices_ == 0)
    {
        Eigen::Isometry3d est(pose.matrix().cast<double>());

        VertexSE3* v0 = new VertexSE3;
        v0->setId(id);
        v0->setEstimate(est);
        v0->setFixed(true);
        optimizer_.addVertex(v0);

        Eigen::Vector3d pos = est.translation();
        positions_.push_back(pos);

        logger.print(
            EventLogger::L_INFO,
            "[SLAM_Solver::addVertexAndEdge] DEBUG: position (%f, %f, %f)\n",
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

        Eigen::Vector3d pos = est.translation();
        positions_.push_back(pos);

        EdgeSE3* e = new EdgeSE3;
        e->vertices()[0] = v0;
        e->vertices()[1] = v1;
        e->setMeasurementFromState();
        optimizer_.addEdge(e);

        logger.print(
            EventLogger::L_INFO,
            "[SLAM_Solver::addVertexAndEdge] DEBUG: position (%f, %f, %f)\n",
            pos[0],
            pos[1],
            pos[2]);

        logger.print(
            EventLogger::L_INFO,
            "[SLAM_Solver::addVertexAndEdge] DEBUG: Adding edge(%lu -> %lu)\n",
            v0->id(),
            v1->id());
    }

    last_added_id_ = id;
    num_vertices_++;
}

void SLAM_Solver::addLoopClosingEdge(const Eigen::Affine3f& vertex_to_origin_transf, const int& id)
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

    num_loop_edges_++;

    logger.print(
        EventLogger::L_INFO,
        "[SLAM_Solver::addLoopClosingEdge] DEBUG: Add loop closing edge: %lu -> %lu\n",
        v->id(),
        origin->id());
}

Edge SLAM_Solver::getEdge(const int& from_id, const int& to_id)
{
    Edge ep(from_id, to_id, positions_[from_id], positions_[to_id]);
    return ep;
}

Edge SLAM_Solver::getLastEdge(const string& type)
{
    int last_id = positions_.size() - 1;
    if (type == "odometry")
    {
        string name = to_string(positions_.size() - 2) + "_" + to_string(last_id);
        Edge ep(positions_.size() - 2, last_id, positions_[positions_.size() - 2], positions_[last_id], name);
        return ep;
    }
    else
    {
        string name = to_string(0) + "_" + to_string(last_id);
        Edge ep(0, last_id, positions_[0], positions_[last_id], name);
        return ep;
    }
}

void SLAM_Solver::optimizeGraph(const int& k)
{
    // optimizer_.save("graph.g2o"); // Save file

    // Only initializeOptimization if the graph is not already initialized.
    if (optimized_estimates_.size() == 0) optimizer_.initializeOptimization();
    optimizer_.optimize(k);

    updateState();
}

void SLAM_Solver::resetGraph()
{
    num_vertices_ = 0;
    last_added_id_ = -1;
    positions_.clear();
    optimized_estimates_.clear();
    optimizer_.clear();
}
