#include <vector>
#include <opencv2/core.hpp>
#include <list>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <common_types.h>

#include <pcl/correspondence.h>

#include <common_types.h>

class Vertice
{
	public:
	
	Vertice();
		
	//number of features in the lastKey Frame
	int features_in_KF;
	

	//vector than contein the number of features of a KF
	std::vector<cv::Point2f> KF_pts_;
		
	cv::Mat KeyFrame;
	cv::Mat depth_KF;
	
	Eigen::Matrix4f trans;
	
	pcl::PointCloud<PointT>::Ptr KF_cloud;


	/*//determinate if a frame is KeyFrame
	bool is_KeyFrame(int features_in_currF, float fkf);

	*/
};


class Graph{

	public:
		void Update_Iterators();
		Graph();
	 	std::list<Vertice*> graph;

		// this iterator points to the last element in the graph
		std::list<Vertice*>::iterator last;
		// this iterator points to the penult element in the graph
		std::list<Vertice*>::iterator penult;
		
		//if this frame is a keyframe add it
		Vertice* add_new_KF(std::vector<cv::Point2f> pts, cv::Mat frame);
		Vertice* add_new_KF(std::vector<cv::Point2f> pts, cv::Mat frame,pcl::PointCloud<PointT>::Ptr KF_cloud);
		
		

		
		


};


