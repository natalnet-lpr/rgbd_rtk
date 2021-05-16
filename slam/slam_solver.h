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

class SLAM_Solver
{
private:
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

    // Number of vertices in the graph
    int num_vertices_;

    // Number of loop edges
    int num_loop_edges_;

    // Vertices to add in update optimization
    g2o::HyperGraph::VertexSet vertices_to_add;

    // Edges to add in update optimization
    g2o::HyperGraph::EdgeSet edges_to_add;

    // Loop closure edges name
    std::vector<std::string> loop_closure_edges_name;

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
     * Get Optimized edge from optimizer
     * @param from_id edge from id
     * @param to_id edge to id
     * @param from position from, the result will be saved in this vector
     * @param to position to, the result will be saved in this vector
     * @param name string where the name of edge will be saved
     */
    void getOptimizedEdge(
        const int& from_id,
        const int& to_id,
        Eigen::Vector3d& from,
        Eigen::Vector3d& to,
        std::string& name);

    /**
     * Returns an Edge
     * @param from_id the id of the "back" of the edge arrow
     * @param to_id the id of the "point" of the edge arrow
     * @param from position from, the result will be saved in this vector
     * @param to position to, the result will be saved in this vector
     * @param name string where the name of edge will be saved
     */
    void getEdge(const int& from_id, const int& to_id, Eigen::Vector3d& from, Eigen::Vector3d& to, std::string& name);

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
};

#endif /* INCLUDE_SLAM_H_ */
