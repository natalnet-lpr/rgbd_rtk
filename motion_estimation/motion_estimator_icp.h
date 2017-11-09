#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/uniform_sampling.h>

typedef pcl::PointXYZRGB PointT;
 
class  MotionEstimatorICP
{



	
	public:
		
		pcl::UniformSampling<PointT> sampling;
		pcl::IterativeClosestPoint<PointT, PointT> ICP; /* to use icp algorithms present in PCL is necessary to have a ICP object */ 
		pcl::PointCloud<PointT>::Ptr tgt_cloud;	/*!target cloud*/
		pcl::PointCloud<PointT>::Ptr src_cloud;	/*!source cloud*/
		
		
		
		
		Eigen::Matrix4f guess;			/*!guess matrix than will be used to estimate a new pose*/

		Eigen::Matrix4f pose;
		
		MotionEstimatorICP ();
		MotionEstimatorICP( const pcl::PointCloud<PointT>::Ptr src, const pcl::PointCloud<PointT>::Ptr tgt);



		MotionEstimatorICP( const pcl::PointCloud<PointT>::Ptr src, const pcl::PointCloud<PointT>::Ptr tgt, Eigen::Matrix4f  &guess);
		
		
		
		Eigen::Matrix4f pairAlign ( const pcl::PointCloud<PointT>::Ptr src_cloud, const pcl::PointCloud<PointT>::Ptr tgt_cloud, pcl::PointCloud<PointT>& output,  Eigen::Matrix4f  &guess);
		
		Eigen::Matrix4f pairAlign ( const pcl::PointCloud<PointT>::Ptr src_cloud, const pcl::PointCloud<PointT>::Ptr tgt_cloud, pcl::PointCloud<PointT>& output);

		Eigen::Matrix4f pairAlign (pcl::PointCloud<PointT>& output);
		

		void DownSamplimg(pcl::PointCloud<PointT>::Ptr cloud,float radius);
		
		
		
		
		
};
