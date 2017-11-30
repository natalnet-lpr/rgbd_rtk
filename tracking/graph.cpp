#include <graph.h>


using namespace std;
using namespace cv;

Vertice::Vertice(){
	
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();;
	KF_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	KF_pts_.clear();
}


Vertice* Graph::add_new_KF(std::vector<cv::Point2f> pts, cv::Mat frame){
	Vertice* tmp;
	tmp = new Vertice;
	tmp->KF_pts_ = pts;
	tmp->features_in_KF = pts.size();
	tmp->KeyFrame = frame;
	
	return tmp;


}
Vertice* Graph::add_new_KF(std::vector<cv::Point2f> pts, cv::Mat frame,	pcl::PointCloud<PointT>::Ptr KF_cloud){
	Vertice* tmp;

	//pcl::PointCloud<PointT> cloud = *KF_cloud;


	tmp = new Vertice;
	tmp->KF_pts_ = pts;
	tmp->features_in_KF = pts.size();
	tmp->KeyFrame = frame;
	*(tmp->KF_cloud) = *KF_cloud;
	

	return tmp;


}
void Graph::Update_Iterators(){
	last = graph.end();
	--last;	
	penult = last;
	--penult;

}
Graph::Graph(){
	Update_Iterators();
}




