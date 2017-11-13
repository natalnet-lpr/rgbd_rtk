#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/registration/icp.h>
#include "motion_estimator_icp.h"
 #include <pcl/filters/uniform_sampling.h>


#include <geometry.h>

#include <cstdio>

using namespace std;
using namespace pcl;



Eigen::Matrix4f rigid_transformation ( const pcl::PointCloud<PointT>::Ptr src_cloud_, const pcl::PointCloud<PointT>::Ptr tgt_cloud_){
	std::vector<unsigned char> is_inlier_;
		long unsigned num_inliers_=0;
	long unsigned N = src_cloud_->points.size();
//Build a RANSAC registration model to estimate the rigid transformation
	pcl::SampleConsensusModelRegistration<PointT>::Ptr sac_model(new pcl::SampleConsensusModelRegistration<PointT>(src_cloud_));
	sac_model->setInputTarget(tgt_cloud_);
	pcl::RandomSampleConsensus<PointT> ransac(sac_model);
	ransac.setDistanceThreshold(0.1); //8mm

	ransac.computeModel();

	//Get the model estimated by RANSAC
	Eigen::VectorXf coeffs, opt_coeffs;
	ransac.getModelCoefficients(coeffs);

	//Get info from RANSAC data and set 0 for outlier and 1 for inlier in each position of the vector
	vector<int> inl;
	ransac.getInliers(inl);
	is_inlier_.resize(N, 0);
	for(size_t i = 0; i < inl.size(); i++)
	{
		int idx = inl[i];
		is_inlier_[idx] = 1;
	}
	num_inliers_ = inl.size();
	float inl_ratio = float(inl.size())/N;
		
	#ifdef DEBUG
	printf("\tinlier ratio: %f\n", inl_ratio);
	#endif

	//Optimize registration transformation using all inlier correspondences
	sac_model->optimizeModelCoefficients(inl, coeffs, opt_coeffs);

	//Set the transf. matrix data from the coeff. param. and return it
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
	trans(0,0) = opt_coeffs[0]; trans(0,1) = opt_coeffs[1]; trans(0,2) = opt_coeffs[2]; trans(0,3) = opt_coeffs[3];
	trans(1,0) = opt_coeffs[4]; trans(1,1) = opt_coeffs[5]; trans(1,2) = opt_coeffs[6]; trans(1,3) = opt_coeffs[7];
	trans(2,0) = opt_coeffs[8]; trans(2,1) = opt_coeffs[9]; trans(2,2) = opt_coeffs[10]; trans(2,3) = opt_coeffs[11];
	trans(3,0) = opt_coeffs[12]; trans(3,1) = opt_coeffs[13]; trans(3,2) = opt_coeffs[14]; trans(3,3) = opt_coeffs[15];
	return trans;
}



void  MotionEstimatorICP::set_lenghts(){

	pcl::PointCloud<PointT>  aux_tgt ;	
	pcl::PointCloud<PointT> aux_src ;
	int menor ;
	


	aux_tgt = *tgt_cloud;

	aux_src = *src_cloud;

	if(src_cloud->size()<tgt_cloud->size()){
		menor = src_cloud->size();
		tgt_cloud->resize(menor);
	}
	else{

		 menor = tgt_cloud->size();
		 src_cloud->resize(menor);
	}	

	
	
}

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





Eigen::Matrix4f MotionEstimatorICP::pairAlign ( const pcl::PointCloud<PointT> src, const pcl::PointCloud<PointT> tgt, pcl::PointCloud<PointT> output){
	


	*tgt_cloud = tgt;
	

	*src_cloud = src;
	
	if(tgt_cloud->empty() || src_cloud->empty()){
		
		cout<<"one or more of your inputs are empty\n";
		exit(0);
	}
	
	
	
	if(!src_cloud->is_dense){
		
		DownSamplimg(src_cloud,0.05);
	}
	if(!tgt_cloud->is_dense){
		
		DownSamplimg(tgt_cloud,0.05);
	}
		cout<<src_cloud->size()<<endl;

	ICP.setInputSource(src_cloud);
	
	ICP.setInputTarget(tgt_cloud);

	ICP.setMaxCorrespondenceDistance (0.1);
	
	// Set the maximum number of iterations (criterion 1)

	ICP.setMaximumIterations (50);

	// Set the transformation epsilon (criterion 2)

	ICP.setTransformationEpsilon (1e-9);

	 // Set the euclidean distance difference epsilon (criterion 3)
	ICP.setEuclideanFitnessEpsilon (0.001); 
	
	// Perform the alignment

	
	ICP.align (output);

	
	tgt_cloud->clear(); src_cloud->clear();


	return ICP.getFinalTransformation ();
}


void MotionEstimatorICP::pairAlign ( const pcl::PointCloud<PointT> src, const pcl::PointCloud<PointT> tgt, pcl::PointCloud<PointT> output, Eigen::Matrix4f &guess){
	


	*tgt_cloud = tgt;
	

	*src_cloud = src;
	
	if(tgt_cloud->empty() || src_cloud->empty()){
		
		cout<<"one or more of your inputs are empty\n";
		exit(0);
	}
	
	
	
	if(!src_cloud->is_dense){
		
		DownSamplimg(src_cloud,0.05);
	}
	if(!tgt_cloud->is_dense){
		
		DownSamplimg(tgt_cloud,0.05);
	}
	set_lenghts();

	guess = rigid_transformation(src_cloud,tgt_cloud);
	

	ICP.setInputSource(src_cloud);
	
	ICP.setInputTarget(tgt_cloud);

	ICP.setMaxCorrespondenceDistance (0.1);
	
	// Set the maximum number of iterations (criterion 1)

	ICP.setMaximumIterations (50);

	// Set the transformation epsilon (criterion 2)

	ICP.setTransformationEpsilon (1e-9);

	 // Set the euclidean distance difference epsilon (criterion 3)
	ICP.setEuclideanFitnessEpsilon (0.001); 
	
	// Perform the alignment

	
	ICP.align (output,guess);

	
	tgt_cloud->clear(); src_cloud->clear();


	guess = ICP.getFinalTransformation ();
}


