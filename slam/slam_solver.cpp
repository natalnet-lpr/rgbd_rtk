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

void SLAM_Solver::addEdgeToList(const int from_id, const int to_id)
{
	Graph_Edge ep(from_id, to_id, m_positions_[from_id], m_positions_[to_id]);
	stringstream edge_name;
	edge_name << "edge_" << from_id << "_" << to_id;
	ep.m_name = edge_name.str();

	//Check if the edge is "odometry" (from i to i+1) or "loop closing" (from i to j)
	if(to_id - from_id == 1)
	{
		m_odometry_edges_.push_back(ep);
	}
	else
	{
		m_loop_edges_.push_back(ep);
	}
}

void SLAM_Solver::updateState()
{
	EventLogger& logger = EventLogger::getInstance();
	logger.setVerbosityLevel(EventLogger::L_DEBUG);
	logger.setLogFileName("log_event_slam_solver.txt");

	m_optimized_estimates_.clear();
	m_optimized_estimates_.resize(m_positions_.size());

	logger.print(EventLogger::L_INFO, "[slam_solver.cpp] INFO: Updating %lu vertices\n", m_positions_.size());

	//Update estimates of all vertices and positions
	for(OptimizableGraph::VertexIDMap::const_iterator it = m_optimizer.vertices().begin();
													  it != m_optimizer.vertices().end();
													  it++)
	{
		int v_id = static_cast<int>(it->first);
		VertexSE3* v = static_cast<VertexSE3*>(it->second);

		Eigen::Isometry3f new_estimate_tmp = v->estimate().cast<float>();
		Eigen::Affine3f new_estimate = Eigen::Affine3f::Identity();

		new_estimate(0,0) = new_estimate_tmp(0,0); new_estimate(0,1) = new_estimate_tmp(0,1); new_estimate(0,2) = new_estimate_tmp(0,2);
		new_estimate(1,0) = new_estimate_tmp(1,0); new_estimate(1,1) = new_estimate_tmp(1,1); new_estimate(1,2) = new_estimate_tmp(1,2);
		new_estimate(2,0) = new_estimate_tmp(2,0); new_estimate(2,1) = new_estimate_tmp(2,1); new_estimate(2,2) = new_estimate_tmp(2,2);
		new_estimate(0,3) = new_estimate_tmp(0,3); new_estimate(1,3) = new_estimate_tmp(1,3); new_estimate(2,3) = new_estimate_tmp(2,3);

		m_optimized_estimates_[v_id] = new_estimate;


		logger.print(EventLogger::L_INFO, "[slam_solver.cpp] INFO: Updating node %i \n", v_id);
		//verificar se é m_positions_ mesmo o desejado
		logger.print(EventLogger::L_INFO, "[slam_solver.cpp] INFO: ### from %f %f %f to %f %f %f\n",
						m_positions_[v_id].x(), m_positions_[v_id].y(), m_positions_[v_id].z(),
						new_estimate(0,3), new_estimate(1,3), new_estimate(2,3), v_id);

		
		//logger.print(EventLogger::L_INFO, "[slam_solver.cpp] INFO: ### from %f %f %f to %f %f %f\n", m_vertices[v_id].x(), m_vertices[v_id].y(), m_vertices[v_id].z(), new_estimate(0,3), new_estimate(1,3), new_estimate(2,3), v_id));

		m_positions_[v_id](0,0) = new_estimate(0,3);
		m_positions_[v_id](1,0) = new_estimate(1,3);
		m_positions_[v_id](2,0) = new_estimate(2,3);
	}

	//Update all odometry edges
	for(unsigned int i = 0; i < m_odometry_edges_.size(); i++)
	{
		const int idx0 = m_odometry_edges_[i].m_id0;
		const int idx1 = m_odometry_edges_[i].m_id1;

		m_odometry_edges_[i].m_pos0 = m_positions_[idx0];
		m_odometry_edges_[i].m_pos1 = m_positions_[idx1];
	}

	//Update all loop closing edges
	for(unsigned int i = 0; i < m_loop_edges_.size(); i++)
	{
		const int idx0 = m_loop_edges_[i].m_id0;
		const int idx1 = m_loop_edges_[i].m_id1;

		m_loop_edges_[i].m_pos0 = m_positions_[idx0];
		m_loop_edges_[i].m_pos1 = m_positions_[idx1];
	}
	logger.print(EventLogger::L_INFO, "[slam_solver.cpp] INFO: Loop closing edges: %lu\n", m_loop_edges_.size());
}

/* #####################################################
 * #####                                           #####
 * #####               Public Impl.                #####
 * #####                                           #####
 * #####################################################
 */

SLAM_Solver::SLAM_Solver()
{
	m_num_vertices_ = 0;
	m_last_added_id_ = -1;

	Linear_Solver* linear_solver = new Linear_Solver();
	Block_Solver* block_solver = new Block_Solver(linear_solver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(block_solver);

	m_optimizer.setAlgorithm(solver);
	m_optimizer.setVerbose(true);
}

void SLAM_Solver::addVertexAndEdge(const Eigen::Affine3f& pose, const int id)
{
	//Add the first node and fix it.
	if(m_num_vertices_ == 0)
	{
		Eigen::Isometry3d est(pose.matrix().cast<double>());

		VertexSE3* v0 = new VertexSE3;
		v0->setId(id);
		v0->setEstimate(est);
		v0->setFixed(true);
		m_optimizer.addVertex(v0);

		Eigen::Vector3d pos = est.translation();
		m_positions_.push_back(pos);
	}
	//When adding any nodes other than the first, add an edge connecting
	//to the previous one
	else
	{
		VertexSE3* v0 = dynamic_cast<g2o::VertexSE3*>(m_optimizer.vertex(m_last_added_id_));

		Eigen::Isometry3d est(pose.matrix().cast<double>());

		VertexSE3* v1 = new VertexSE3;
		v1->setId(id);
		v1->setEstimate(est);
		m_optimizer.addVertex(v1);

		Eigen::Vector3d pos = est.translation();
		m_positions_.push_back(pos);

		EdgeSE3* e = new EdgeSE3;
		e->vertices()[0] = v0;
		e->vertices()[1] = v1;
		e->setMeasurementFromState();
		m_optimizer.addEdge(e);

		addEdgeToList(v0->id(), v1->id());
	}

	m_last_added_id_ = id;
	m_num_vertices_++;
}

void SLAM_Solver::addLoopClosingEdge(const Eigen::Affine3f& vertex_to_origin_transf, const int id)
{
	//assumes the first vertex has id = 0
	VertexSE3* origin = dynamic_cast<g2o::VertexSE3*>(m_optimizer.vertex(0));
	VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(m_optimizer.vertex(id));

	Eigen::Isometry3d measurement(vertex_to_origin_transf.matrix().cast<double>());

	EdgeSE3* e = new EdgeSE3;
	e->vertices()[0] = v;
	e->vertices()[1] = origin;
	e->setMeasurement(measurement);
	m_optimizer.addEdge(e);

	addEdgeToList(v->id(), origin->id());
}

void SLAM_Solver::optimizeGraph(const int k)
{
	//m_optimizer.save("graph.g2o");

	m_optimizer.initializeOptimization();
	m_optimizer.optimize(k);

	updateState();
}
