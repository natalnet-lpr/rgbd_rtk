#include <cstdio>
#include <cstdlib>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry.h>
#include <rgbd_loader.h>
#include <motion_estimator_icp.h>
#include <reconstruction_visualizer.h>


#include <fstream>
#include <ctime>

using namespace std;
using namespace cv;
int main(int argc, char **argv)
{	
	string index_file_name;

	RGBDLoader loader;
	Mat frame, depth;
	pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr curr_cloud(new pcl::PointCloud<PointT>);
	
	Eigen::Affine3f pose = Eigen::Affine3f::Identity();
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();

	Eigen::Matrix4f guess;

	ReconstructionVisualizer visualizer;

	ofstream cam_path;
	cam_path.open("pos_relativa2.txt");
	Intrinsics intr(0);
	
	if(argc != 2)
	{
		fprintf(stderr, "Usage: %s <index file>\n", argv[0]);
		exit(0);
	}

	index_file_name = argv[1];
	loader.processFile(index_file_name);
	
	MotionEstimatorICP estimator;

	
	for(int i = 0; i < loader.num_images_; i++)
	{
		//Load RGB-D image and point cloud 
		loader.getNextImage(frame, depth);
		*curr_cloud = getPointCloud(frame, depth, intr);
		/*
		if(i > 0 && i<200)
		{	
			
			guess = estimator.pairAlign(*curr_cloud,*prev_cloud,*curr_cloud);

			
		}
		else if(i>=200)
		{	
			
			estimator.pairAlign(*curr_cloud,*prev_cloud,*curr_cloud,guess);
			

		}
		if(i){	
			trans = guess;			
			pose = trans*pose;
		}*/
		if(i > 0)
		{	

			guess = estimator.pairAlign(*curr_cloud,*prev_cloud,*curr_cloud);

			trans = guess;			
			pose = trans*pose;
			
		}
		if(i == 0) visualizer.addReferenceFrame(pose, "origin");
		//visualizer.addQuantizedPointCloud(curr_cloud, 0.3, pose);
		 visualizer.viewReferenceFrame(pose);
		//visualizer.viewPointCloud(curr_cloud, pose);
		 visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, pose);

		 visualizer.spinOnce();


		//Show RGB-D image
		imshow("Image view", frame);
		//imshow("Depth view", depth);
		char key = waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}
	
		//Let the prev. cloud in the next frame be the current cloud
		*prev_cloud = *curr_cloud;

		
			 Eigen::Matrix3f R;
	      		    R(0,0) = pose(0,0); R(0,1) = pose(0,1); R(0,2) = pose(0,2);
      			  R(1,0) = pose(1,0); R(1,1) = pose(1,1); R(1,2) = pose(1,2);
      			  R(2,0) = pose(2,0); R(2,1) = pose(2,1); R(2,2) = pose(2,2);
      			  Eigen::Quaternionf q(R);
      			  cam_path<< loader.tstamps[i] << " " << pose(0,3) << " "
                                      << pose(1,3) << " "
                                      << pose(2,3) << " "
                                      << q.x() << " "
                                      << q.y() << " "
                                      << q.z() << " "
                                      << q.w() << "\n";						
		
		
	}

	return 0;
}
