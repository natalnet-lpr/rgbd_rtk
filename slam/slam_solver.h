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

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <common_types.h>

//convenience typedefs
typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>>  Block_Solver;
typedef g2o::LinearSolverCSparse<Block_Solver::PoseMatrixType> Linear_Solver;

class SLAM_Solver
{
	private:
		//Number of vertices in the graph
		int m_num_vertices;

		//Id of the last added vertex (used when adding edges)
		int m_last_added_id;

		g2o::SparseOptimizer m_optimizer;

		//Internal function to create edges
		void add_edge_to_list(const int from_id, const int to_id);

		//Update internal state after optimization
		void update_state();

	public:

		//3D position in the global ref. frame of each node (for visualization)
		std::vector<Eigen::Vector3d> m_positions;

		//Stores information about all "odometry" edges
		std::vector<Graph_Edge> m_odometry_edges;

		//Stores information about all "loop closing" edges
		std::vector<Graph_Edge> m_loop_edges;

		//Estimate of each active vertex after optimization
		std::vector<Eigen::Matrix4f> m_optimized_estimates;

		//Default constructor
		SLAM_Solver();

		//Adds a vertex to the graph with the given pose and id
		//and also an "odometry" edge connecting it to the previous
		//vertex in the graph
		void add_vertex_and_edge(const Eigen::Matrix4f pose, const int id);

		//Adds a "loop closing" edge connecting the vertex with given id to the
		//origin (vertex with id = 0)
		void add_loop_closing_edge(const Eigen::Matrix4f vertex_to_origin_transf, const int id);

		//Optimizes the graph with Levenberg-Marquardt for the given n. of iterations
		void optimize_graph(const int k);

		//set nodes fixed

		//void print_debug_info();
};


#endif /* INCLUDE_SLAM_SOLVER_H_ */