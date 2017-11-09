#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/registration/icp.h>
#include "motion_estimator_icp.h"
 #include <pcl/filters/uniform_sampling.h>
 #include <pcl/filters/impl/uniform_sampling.hpp>
using namespace std;
using namespace pcl;

void  MotionEstimatorICP::DownSamplimg(pcl::PointCloud<PointT>::Ptr cloud,float radius){
	
	sampling.setInputCloud(cloud);
	sampling.setRadiusSearch(radius);
	sampling.filter(*cloud);
	cloud->is_dense=false;


}

MotionEstimatorICP::MotionEstimatorICP (){

	pose = Eigen::Matrix4f::Identity();
	
	tgt_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	src_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	


}
MotionEstimatorICP::MotionEstimatorICP( const pcl::PointCloud<PointT>::Ptr src, const pcl::PointCloud<PointT>::Ptr tgt){
	
	pose = Eigen::Matrix4f::Identity();
	
	tgt_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	tgt_cloud = tgt;

	src_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	src_cloud = src;

	if(!src->is_dense)
		DownSamplimg(src_cloud,0.1);
	// Set the input source and target
	ICP.setInputCloud(src);
	
	ICP.setInputTarget(tgt);
	
	

		
}



MotionEstimatorICP::MotionEstimatorICP( const pcl::PointCloud<PointT>::Ptr src, const pcl::PointCloud<PointT>::Ptr tgt, Eigen::Matrix4f  &guess){
	
	pose = Eigen::Matrix4f::Identity();
	
	tgt_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	tgt_cloud= tgt;

	src_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	src_cloud= src;

	if(!src_cloud->is_dense){
		
		DownSamplimg(src_cloud,0.1);
	}
	if(!tgt_cloud->is_dense){
		
		DownSamplimg(tgt_cloud,0.1);
	}
	
	// Set the input source and target
	ICP.setInputCloud(src);
	
	ICP.setInputTarget(tgt);


	this->guess = guess;


		
}

Eigen::Matrix4f MotionEstimatorICP::pairAlign (pcl::PointCloud<PointT>& output){
	if(tgt_cloud->empty() || src_cloud->empty()){
		
		cout<<"one or more of your inputs are empty\n";
		exit(0);
	}

	if(!src_cloud->is_dense){
		
		DownSamplimg(src_cloud,0.1);
	}
	if(!tgt_cloud->is_dense){
		
		DownSamplimg(tgt_cloud,0.1);
	}
	
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	
	ICP.setMaxCorrespondenceDistance (0.008);
	

	// Set the maximum number of iterations (criterion 1)

	 ICP.setMaximumIterations (50);

	// Set the transformation epsilon (criterion 2)

	ICP.setTransformationEpsilon (1e-9);

	 // Set the euclidean distance difference epsilon (criterion 3)
	ICP.setEuclideanFitnessEpsilon (0.001); 
	
	// Perform the alignment
	ICP.align (output);
	
	return ICP.getFinalTransformation ();
		
}

Eigen::Matrix4f MotionEstimatorICP::pairAlign ( const pcl::PointCloud<PointT>::Ptr src, const pcl::PointCloud<PointT>::Ptr tgt, pcl::PointCloud<PointT>& output){
	
	pcl::PointCloud<PointT> test;


	tgt_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	tgt_cloud = tgt;
	
	src_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	src_cloud = src;


	if(tgt_cloud->empty() || src_cloud->empty()){
		
		cout<<"one or more of your inputs are empty\n";
		exit(0);
	}

	
	if(!src_cloud->is_dense){

		DownSamplimg(src_cloud,0.1);
	}
	if(!tgt_cloud->is_dense){
		
		DownSamplimg(tgt_cloud,0.1);
	}
	
	
		
	cout<<"src size " <<src->size()<<endl;
	cout<<"tgt size " <<tgt->size()<<endl;
	// Set the input source and target
	ICP.setInputCloud(src);
	
	ICP.setInputTarget(tgt);
	
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	
	ICP.setMaxCorrespondenceDistance (0.05);


	// Set the maximum number of iterations (criterion 1)

	 ICP.setMaximumIterations (50);

	// Set the transformation epsilon (criterion 2)

	ICP.setTransformationEpsilon (1e-2);

	 // Set the euclidean distance difference epsilon (criterion 3)
	ICP.setEuclideanFitnessEpsilon (0.01); 
	
	// Perform the alignment
	ICP.align (test);
					cout<<"passou2\n";			
	return ICP.getFinalTransformation ();
	
}

Eigen::Matrix4f  MotionEstimatorICP::pairAlign ( const pcl::PointCloud<PointT>::Ptr src, const pcl::PointCloud<PointT>::Ptr tgt, pcl::PointCloud<PointT>& output,  Eigen::Matrix4f  &guess){

	tgt_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	tgt_cloud = tgt;
	
	src_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	src_cloud = src;

	if(tgt_cloud->empty() || src_cloud->empty()){
		
		cout<<"one or more of your inputs are empty\n";
		exit(0);
	}
	
	
	
	if(!src_cloud->is_dense){
		
		DownSamplimg(src_cloud,0.1);
	}
	if(!tgt_cloud->is_dense){
		
		DownSamplimg(tgt_cloud,0.1);
	}
	
	
	
	// Set the input source and target
	ICP.setInputCloud(src);
	
	ICP.setInputTarget(tgt);
	
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	
	ICP.setMaxCorrespondenceDistance (0.008);
	

	// Set the maximum number of iterations (criterion 1)

	 ICP.setMaximumIterations (50);

	// Set the transformation epsilon (criterion 2)

	ICP.setTransformationEpsilon (1e-9);

	 // Set the euclidean distance difference epsilon (criterion 3)
	ICP.setEuclideanFitnessEpsilon (0.001); 
	
	// Perform the alignment
	ICP.align (output,guess);
	
	return ICP.getFinalTransformation ();
}



