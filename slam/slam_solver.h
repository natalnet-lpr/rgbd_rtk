/*
 * RGB-D Visual Odometry and SLAM.
 *
 * Graph SLAM solver based on G2O.
 *
 * Author: Bruno Marques F. da Silva
 * brunomfs@gmail.com
 */

#ifndef INCLUDE_SLAM_SOLVER_H_
#define INCLUDE_SLAM_SOLVER_H_

#include <vector>

#include <Eigen/Dense>

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <common_types.h>
#include <event_logger.h>

// convenience typedefs
typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>> Block_Solver;
typedef g2o::LinearSolverCSparse<Block_Solver::PoseMatrixType> Linear_Solver;

class SLAM_Solver
{
private:
    // Number of vertices in the graph
    int num_vertices_;

    // Id of the last added vertex (used when adding edges)
    int last_added_id_;

    g2o::SparseOptimizer optimizer_;

    /**
     * Internal function to create edges
     * @param from_edge_id id edge from
     * @param to_edge_id id edge to
     */
    void addEdgeToList(const int &from_id, const int &to_id);

    /**
     * Update internal state after optimization
     */
    void updateState();

public:
    // 3D position in the global ref. frame of each node (for visualization)
    std::vector<Eigen::Vector3d> positions_;

    // Stores information about all "odometry" edges
    std::vector<Edge> odometry_edges_;

    // Stores information about all "loop closing" edges
    std::vector<Edge> loop_edges_;

    // Estimate of each active vertex after optimization
    std::vector<Eigen::Affine3f> optimized_estimates_;

    // Default constructor
    SLAM_Solver();

    /**
     * Adds a vertex to the graph with the given pose and id
     * and also an "odometry" edge connecting it to the previous
     * vertex in the graph
     * @param pose pose to be added
     * @param id pose id
     */
    void addVertexAndEdge(const Eigen::Affine3f &pose, const int &id);

    /**
     * Adds a "loop closing" edge connecting the vertex with given id to the
     * origin (vertex with id = 0)
     * @param vertex_to_origin_transformation A transformation vertex to origin
     * @param id vertex id
     */
    void addLoopClosingEdge(const Eigen::Affine3f &vertex_to_origin_transf, const int &id);

    /**
     * Optimizes the graph with Levenberg-Marquardt for the given n. of iterations
     * @param k number of iterations type int
     */
    void optimizeGraph(const int &k);

    /**
     * @return the number of vertices
     */
    int getMNumVertices();
};

#endif /* INCLUDE_SLAM_H_ */
