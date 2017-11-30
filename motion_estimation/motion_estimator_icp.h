#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/uniform_sampling.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;
 typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
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
	




		
		
		
		void pairAlign ( const pcl::PointCloud<PointT> src, const pcl::PointCloud<PointT> tgt, pcl::PointCloud<PointT> output, Eigen::Matrix4f& guess);
		
		Eigen::Matrix4f pairAlign ( const pcl::PointCloud<PointT> src_cloud, const pcl::PointCloud<PointT> tgt_cloud, pcl::PointCloud<PointT> output);

		

		void DownSamplimg(pcl::PointCloud<PointT>::Ptr cloud,float radius);
		
		void set_lenghts();
		
		
		
};
