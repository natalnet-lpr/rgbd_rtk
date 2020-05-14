/*
 * Implementation of klt_slam.h
 *
 * Author: Bruno Marques F. da Silva
 * brunomfs@gmail.com
 */

#include <iostream>
#include <sstream>

#include <Eigen/Dense>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <slam_solver.h>

using namespace std;
using namespace g2o;

/* #####################################################
 * #####                                           #####
 * #####               Private Impl.               #####
 * #####                                           #####
 * #####################################################
 */

void SLAM_Solver::add_edge_to_list(const int from_id, const int to_id)
{
	Graph_Edge ep(from_id, to_id, m_positions[from_id], m_positions[to_id]);
	stringstream edge_name;
	edge_name << "edge_" << from_id << "_" << to_id;
	ep.m_name = edge_name.str();

	//Check if the edge is "odometry" (from i to i+1) or "loop closing" (from i to j)
	if(to_id - from_id == 1)
	{
		m_odometry_edges.push_back(ep);
	}
	else
	{
		m_loop_edges.push_back(ep);
	}
}

void SLAM_Solver::update_state()
{
	m_optimized_estimates.clear();
	m_optimized_estimates.resize(m_positions.size());
	printf("Updating %lu vertices\n", m_positions.size());

	//Update estimates of all vertices and positions
	for(OptimizableGraph::VertexIDMap::const_iterator it = m_optimizer.vertices().begin();
													  it != m_optimizer.vertices().end();
													  it++)
	{
		int v_id = static_cast<int>(it->first);
		VertexSE3* v = static_cast<VertexSE3*>(it->second);

		Eigen::Isometry3f new_estimate_tmp = v->estimate().cast<float>();
		Eigen::Matrix4f new_estimate = Eigen::Matrix4f::Identity();

		new_estimate(0,0) = new_estimate_tmp(0,0); new_estimate(0,1) = new_estimate_tmp(0,1); new_estimate(0,2) = new_estimate_tmp(0,2);
		new_estimate(1,0) = new_estimate_tmp(1,0); new_estimate(1,1) = new_estimate_tmp(1,1); new_estimate(1,2) = new_estimate_tmp(1,2);
		new_estimate(2,0) = new_estimate_tmp(2,0); new_estimate(2,1) = new_estimate_tmp(2,1); new_estimate(2,2) = new_estimate_tmp(2,2);
		new_estimate(0,3) = new_estimate_tmp(0,3); new_estimate(1,3) = new_estimate_tmp(1,3); new_estimate(2,3) = new_estimate_tmp(2,3);

		m_optimized_estimates[v_id] = new_estimate;

		//printf("### Updating node %i\n", v_id);
		//printf("### from %f %f %f to %f %f %f\n", m_vertices[v_id].x(), m_vertices[v_id].y(), m_vertices[v_id].z(),
		//										  new_estimate(0,3), new_estimate(1,3), new_estimate(2,3));

		m_positions[v_id](0,0) = new_estimate(0,3);
		m_positions[v_id](1,0) = new_estimate(1,3);
		m_positions[v_id](2,0) = new_estimate(2,3);
	}

	//Update all odometry edges
	for(unsigned int i = 0; i < m_odometry_edges.size(); i++)
	{
		const int idx0 = m_odometry_edges[i].m_id0;
		const int idx1 = m_odometry_edges[i].m_id1;

		m_odometry_edges[i].m_pos0 = m_positions[idx0];
		m_odometry_edges[i].m_pos1 = m_positions[idx1];
	}

	//Update all loop closing edges
	for(unsigned int i = 0; i < m_loop_edges.size(); i++)
	{
		const int idx0 = m_loop_edges[i].m_id0;
		const int idx1 = m_loop_edges[i].m_id1;

		m_loop_edges[i].m_pos0 = m_positions[idx0];
		m_loop_edges[i].m_pos1 = m_positions[idx1];
	}
	printf("loop closing edges: %lu\n", m_loop_edges.size());
}


/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */


SLAM_Solver::SLAM_Solver()
{
	m_num_vertices = 0;
	m_last_added_id = -1;

	std::unique_ptr<g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> linear_solver (new g2o:Linear_Solver)
	//std::unique_ptr<g2o::BlockSolverTraits::LinearSolverType> linearSolver (new g2o::LinearSolverCSparse<g2o::BlockSolver::PoseMatrixType());
	//std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr (new g2o::BlockSolver_6_3(std::move(linearSolver)));
	//g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

	//Linear_Solver* linear_solver = new Linear_Solver();
	//Block_Solver* block_solver = new Block_Solver(linear_solver);
//	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(block_solver);
//	typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>>  Block_Solver;
//	typedef g2o::LinearSolverCSparse<Block_Solver::PoseMatrixType> Linear_Solver;

	//m_optimizer.setAlgorithm(solver);
	//m_optimizer.setVerbose(true);
}

void SLAM_Solver::add_vertex_and_edge(const Eigen::Matrix4f pose, const int id)
{
	//We didn't add any node yet. Add the first and fix it.
	if(m_num_vertices == 0)
	{
		Eigen::Isometry3d est(pose.cast<double>());

		VertexSE3* v0 = new VertexSE3;
		v0->setId(id);
		v0->setEstimate(est);
		v0->setFixed(true);
		m_optimizer.addVertex(v0);

		Eigen::Vector3d pos = est.translation();
		m_positions.push_back(pos);
	}
	//When adding any nodes other than the first, add an edge connecting
	//to the previous one
	else
	{
		VertexSE3* v0 = dynamic_cast<g2o::VertexSE3*>(m_optimizer.vertex(m_last_added_id));

		Eigen::Isometry3d est(pose.cast<double>());

		VertexSE3* v1 = new VertexSE3;
		v1->setId(id);
		v1->setEstimate(est);
		m_optimizer.addVertex(v1);

		Eigen::Vector3d pos = est.translation();
		m_positions.push_back(pos);

		EdgeSE3* e = new EdgeSE3;
		e->vertices()[0] = v0;
		e->vertices()[1] = v1;
		e->setMeasurementFromState();
		m_optimizer.addEdge(e);

		add_edge_to_list(v0->id(), v1->id());
	}

	m_last_added_id = id;
	m_num_vertices++;
}

void SLAM_Solver::add_loop_closing_edge(const Eigen::Matrix4f vertex_to_origin_transf, const int id)
{
	//assumes the first vertex has id = 0
	VertexSE3* origin = dynamic_cast<g2o::VertexSE3*>(m_optimizer.vertex(0));
	VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(m_optimizer.vertex(id));

	Eigen::Isometry3d measurement(vertex_to_origin_transf.cast<double>());

	EdgeSE3* e = new EdgeSE3;
	e->vertices()[0] = v;
	e->vertices()[1] = origin;
	e->setMeasurement(measurement);
	m_optimizer.addEdge(e);

	add_edge_to_list(v->id(), origin->id());
}

void SLAM_Solver::optimize_graph(const int k)
{
	//m_optimizer.save("graph.g2o");

	m_optimizer.initializeOptimization();
	m_optimizer.optimize(k);

	update_state();
}
