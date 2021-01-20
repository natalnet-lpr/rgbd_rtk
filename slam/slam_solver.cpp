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

void SLAM_Solver::addEdgeToList(const int& from_id, const int& to_id)
{
    Edge ep(from_id, to_id, positions_[from_id], positions_[to_id]);
    stringstream edge_name;
    edge_name << "edge_" << from_id << "_" << to_id;
    ep.name_ = edge_name.str();
    logger.print(
        EventLogger::L_INFO,
        "[SLAM_Solver::addEdgeToList] DEBUG: built edge %s\n",
        ep.name_.c_str());

    // Check if the edge is "odometry" (from i to i+1) or "loop closing" (from i to j)
    (to_id - from_id == 1) ? odometry_edges_.push_back(ep) : loop_edges_.push_back(ep);
    logger.print(
        EventLogger::L_INFO,
        "[SLAM_Solver::addEdgeToList] DEBUG: odom. edges: %lu\n",
        odometry_edges_.size());
    logger.print(
        EventLogger::L_INFO,
        "[SLAM_Solver::addEdgeToList] DEBUG: loop closing edges: %lu\n",
        loop_edges_.size());
}

void SLAM_Solver::updateState()
{
    optimized_estimates_.clear();
    optimized_estimates_.resize(positions_.size());

    logger.print(
        EventLogger::L_INFO,
        "[SLAM_Solver::updateState] DEBUG: Updating %lu vertices\n",
        positions_.size());

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

        logger.print(
            EventLogger::L_INFO, "[SLAM_Solver::updateState] DEBUG: Updating node %i \n", v_id);
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

    // Update all odometry edges
    for (unsigned int i = 0; i < odometry_edges_.size(); i++)
    {
        const int id_from = odometry_edges_[i].id_from_;
        const int id_to = odometry_edges_[i].id_to_;

        odometry_edges_[i].pose_from_ = positions_[id_from];
        odometry_edges_[i].pose_to_ = positions_[id_to];
    }

    // Update all loop closing edges
    for (unsigned int i = 0; i < loop_edges_.size(); i++)
    {
        const int id_from = loop_edges_[i].id_from_;
        const int id_to = loop_edges_[i].id_to_;

        loop_edges_[i].pose_from_ = positions_[id_from];
        loop_edges_[i].pose_to_ = positions_[id_to];
    }
    logger.print(
        EventLogger::L_INFO,
        "[SLAM_Solver::updateState] DEBUG: Loop closing edges: %lu\n",
        loop_edges_.size());
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

    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =
        g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

    optimizer_.setAlgorithm(solver);
    optimizer_.setVerbose(true);
}

void SLAM_Solver::addVertexAndEdge(const Eigen::Affine3f& pose, const int& id)
{
    logger.print(
        EventLogger::L_INFO, "[SLAM_Solver::addVertexAndEdge] DEBUG: Adding node %i\n", id);

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

        addEdgeToList(v0->id(), v1->id());

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

    addEdgeToList(v->id(), origin->id());

    logger.print(
        EventLogger::L_INFO,
        "[SLAM_Solver::addLoopClosingEdge] DEBUG: Add loop closing edge: %lu -> %lu\n",
        v->id(),
        origin->id());
}

void SLAM_Solver::optimizeGraph(const int& k)
{
    // optimizer_.save("graph.g2o"); // Save file

    optimizer_.initializeOptimization();
    optimizer_.optimize(k);

    updateState();
}
int SLAM_Solver::getNumVertices() { return num_vertices_; }