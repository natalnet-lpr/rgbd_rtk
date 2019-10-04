#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include<iostream>
using std::vector;

using namespace Eigen;

double RPE(vector<Eigen::Matrix4f> groundPoses,vector<Eigen::Matrix4f> estimatedPoses){

	vector <Eigen::Matrix4f> RPE_vector;
	for(int i=0;i<(int)estimatedPoses.size()-1;i++){
		Eigen::Matrix4f  a = ((groundPoses[i].inverse())*groundPoses[i+1]).inverse();
		Eigen::Matrix4f  b = (estimatedPoses[i].inverse())*estimatedPoses[i+1];
		RPE_vector.push_back(a*b);
		//std::cout<<a*b<<"\n\n";


	}




	double RMSE=0;
	int m = RPE_vector.size();
	for(int i=0;i<m;i++){

		Vector4f trans_h = RPE_vector[i].rightCols<1>();
		Vector3f trans;
		trans(0,0) = trans_h(0,0);		
		trans(1,0) = trans_h(1,0);
		trans(2,0) = trans_h(2,0);

		RMSE += trans.norm()*trans.norm();
	}
	RMSE /=m;
	return sqrt(RMSE);
}
