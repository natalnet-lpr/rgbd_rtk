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

#include <Eigen/Geometry>

#include <g2o/core/sparse_optimizer.h>

#include <common_types.h>
#include <event_logger.h>

struct EdgeObject
{
    Eigen::Affine3f from;
    Eigen::Affine3f to;
    bool error = false;
};

class SLAM_Solver
{
private:
    // Number of vertices in the graph
    int num_vertices_;

    // Id of the last added vertex (used when adding edges)
    int last_added_id_;

    // G2O nonlinear optimizer
    g2o::SparseOptimizer optimizer_;

    /**
     * Update internal state after optimization
     */
    void updateState();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Number of loop edges
    int num_loop_edges_;

    // 3D position in the global ref. frame of each node (for visualization)
    std::vector<Eigen::Vector3d> positions_;

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
    void addVertexAndEdge(const Eigen::Affine3f& pose, const int& id);

    /**
     * Adds a "loop closing" edge connecting the vertex with given id to the
     * origin (vertex with id = 0)
     * @param vertex_to_origin_transformation A transformation vertex to origin
     * @param id vertex id
     */
    void addLoopClosingEdge(const Eigen::Affine3f& vertex_to_origin_transf, const int& id);

    /**
     * Returns an Edge
     * @param from_id the id of the "back" of the edge arrow
     * @param to_id the id of the "point" of the edge arrow
     * @return Edge that connects from_id to to_id
     */
    EdgeObject getEdge(const int& from_id, const int& to_id);

    /**
     * Returns the the last edge, if it is an odometry it will return the edge from positions_.size() - 2 to
     * positions.size() - 1 If it is an loop it will return 0 to positions.size() - 1
     * @return Edge
     */
    EdgeObject getLastEdge();

    /**
     * Optimizes the graph with Levenberg-Marquardt for the given n. of iterations
     * @param k number of iterations type int
     */
    void optimizeGraph(const int& k);

    /**
     * Reset graph
     */
    void resetGraph();
};

#endif /* INCLUDE_SLAM_H_ */
