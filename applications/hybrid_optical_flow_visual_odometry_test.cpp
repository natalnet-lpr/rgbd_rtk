#include <cstdio>
#include <cstdlib>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <geometry.h>
#include <rgbd_loader.h>
#include <optical_flow_visual_odometry.h>
#include <reconstruction_visualizer.h>
#include <kitti_stereo_loader.h>
#include <velodyne_loader.h>
#include <kitti_cloud_generator.h>
#include <stereo_optical_flow_visual_odometry.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	KITTIStereoLoader loader; 
	KITTICloudGenerator generator;
	VelodyneLoader vl;
	string path_stereo =  argv[1];
	string path_velo =  argv[2];
	int sequenceNum =atoi(argv[3]);
	//int sequenceNum = 0;
	Intrinsics intr(718.856,718.856,607.1928,185.2157);
	StereoOpticalFlowVisualOdometry vo(intr);
	//ReconstructionVisualizer visualizer;
	Mat frame, depth;
	Mat Q = (cv::Mat_<float>(4,4) <<1, 0, 0,       -607.1928, 
                                      0, 1, 0,       -185.2157,
                                      0, 0, 0,       718.856,
                                      0, 0, 1/0.54, 0); 
	float T[3]={-0.004069766,-0.00631618,-0.2717806};   //Translation Matrix
  	float R[3][3]={0.007533745,     -0.999714,    -0.0006166020, //Rotation Matrix
           0.01489249,      0.0007280733, -0.998902,
           0.9998621,       0.007523790,  0.01480755
  };

	if(argc != 4)
	{
		fprintf(stderr, "Usage: %s <index file stereo> <index file lidar> <index file sequence>\n", argv[0]);
		exit(0);
	}

	loader.loaderImageSequence(path_stereo,sequenceNum,true);	
	vl.loaderPathSequence(path_velo,sequenceNum);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cv = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cs = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cv_fov = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cs_fov = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    Eigen::Matrix4f pose; //Reference pose of PCL coordinate system where X is forward, Y is up, and Z is right.
		pose << 0, 0, 1, 0,
		             0,-1, 0, 0,
		             1, 0, 0, 0,
		             0, 0, 0, 1;
		

	//Compute visual odometry on each image
    double vect_poses[loader.num_images][12];
    char s_num[3];
    sprintf(s_num, "%02d", sequenceNum);
    cout<<"The poses file will be write into hybrid_pose_"<<s_num<<".txt"<<endl;
    string pose_s= "hybrid_pose_"+string(s_num)+".txt";
    ofstream outputFile(pose_s);
    outputFile<<std::scientific;
	for(int i = 0; i < loader.num_images; i++)
	{
		//Load Stereo image 
		frame=loader.getNextLeftSequence(true);

		//Generate Stereo point cloud
		generator.cloudGenerator(frame,loader.getNextRightSequence(true),Q);

		//Load Lidar Monochromatic RGB point cloud 
		*cv=*vl.getPointCloudRGB();

		float x,y,z;
  		for(int j=0;j<cv->points.size();j++){
     		 x=cv->points[j].x;
		      y=cv->points[j].y;
		      z=cv->points[j].z;
		      cv->points[j].x= R[0][0]*(x-T[0]) + R[0][1]*(y-T[1]) + R[0][2]*(z-T[2]);  
		      cv->points[j].y= R[1][0]*(x-T[0])+ R[1][1]*(y-T[1]) + R[1][2]*(z-T[2]);  
		      cv->points[j].z= R[2][0]*(x-T[0])+ R[2][1]*(y-T[1]) + R[2][2]*(z-T[2]);  
		  }

		//Put the lidar point cloud into a frustum.
		pcl::FrustumCulling<pcl::PointXYZRGB> fc;
		fc.setInputCloud (cv);
		fc.setCameraPose(pose);
		fc.setVerticalFOV (85);
		fc.setHorizontalFOV (85);
		fc.setNearPlaneDistance (5);
		fc.setFarPlaneDistance (120);
		fc.filter (*cv_fov);
  		
  		*cs=*generator.clearPointCloud();
		
	  	//Filter clouds 
	  	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

	  	out_cloud->clear();
  		kdtree.setInputCloud (cv_fov);
		for(int i=0;i<cs->points.size();i++){
		  	if(!isnan(cs->points[i].z)){
		        vector<int> pointIdxNKNSearch(1);
		        vector<float> pointNKNSquaredDistance(1);
		        kdtree.nearestKSearch(cs->points[i],1,pointIdxNKNSearch,pointNKNSquaredDistance);
		        if(pointNKNSquaredDistance[0]<1.0){    	
		          out_cloud->points.push_back(cv_fov->points[pointIdxNKNSearch[0]]);
		   	}
		        else{
		          out_cloud->points.push_back(cs->points[i]);
		          //out_cloud->points[i].z=  std::numeric_limits<double>::quiet_NaN();
		        }
		    }
		    else{
		    	out_cloud->points.push_back(cs->points[i]);
		    }  
	  }
		out_cloud->width=cs->width;
		out_cloud->height=cs->height;
		

		//Estimate current camera pose
		vo.computeCameraPose(frame, out_cloud);
		//Store poses
		vect_poses[i][0]=vo.pose_.matrix()(0,0);
		vect_poses[i][1]=vo.pose_.matrix()(0,1);
		vect_poses[i][2]=vo.pose_.matrix()(0,2);
		vect_poses[i][3]=vo.pose_.matrix()(0,3);
		vect_poses[i][4]=vo.pose_.matrix()(1,0);
		vect_poses[i][5]=vo.pose_.matrix()(1,1);
		vect_poses[i][6]=vo.pose_.matrix()(1,2);
		vect_poses[i][7]=vo.pose_.matrix()(1,3);
		vect_poses[i][8]=vo.pose_.matrix()(2,0);
		vect_poses[i][9]=vo.pose_.matrix()(2,1);
		vect_poses[i][10]=vo.pose_.matrix()(2,2);
		vect_poses[i][11]=vo.pose_.matrix()(2,3);
		
		/*
		//View tracked points
		for(size_t k = 0; k < vo.tracker_.curr_pts_.size(); k++)
		{
			Point2i pt1 = vo.tracker_.prev_pts_[k];
			Point2i pt2 = vo.tracker_.curr_pts_[k];
			circle(frame, pt1, 1, CV_RGB(255,0,0), -1);
			circle(frame, pt2, 3, CV_RGB(0,0,255), -1);
			line(frame, pt1, pt2, CV_RGB(0,0,255));
		}

		if(i == 0) visualizer.addReferenceFrame(vo.pose_, "origin");
		visualizer.addQuantizedPointCloud(vo.curr_dense_cloud_, 5.0, vo.pose_);
		visualizer.viewReferenceFrame(vo.pose_);
		visualizer.viewPointCloud(vo.curr_dense_cloud_, vo.pose_);
		//visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.02, vo.pose_);

		visualizer.spinOnce();

		//Show RGB-D image
		imshow("Image view", frame);
		//imshow("Depth view", depth);
		char key = waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}*/
	}
	for(int j=0;j<loader.num_images;j++){
		if(j!=loader.num_images-1){
			outputFile << vect_poses[j][0] <<" "<<vect_poses[j][1]<<" "<<vect_poses[j][2]<<" "<<vect_poses[j][3]<<" "<<vect_poses[j][4]<<" "<<vect_poses[j][5]<<" "<<
			vect_poses[j][6]<<" "<<vect_poses[j][7]<<" "<<vect_poses[j][8]<<" "<<vect_poses[j][9]<<" "<<vect_poses[j][10]<<" "<<vect_poses[j][11]<<"\n";}
		else{
			outputFile << vect_poses[j][0] <<" "<<vect_poses[j][1]<<" "<<vect_poses[j][2]<<" "<<vect_poses[j][3]<<" "<<vect_poses[j][4]<<" "<<vect_poses[j][5]<<" "<<
			vect_poses[j][6]<<" "<<vect_poses[j][7]<<" "<<vect_poses[j][8]<<" "<<vect_poses[j][9]<<" "<<vect_poses[j][10]<<" "<<vect_poses[j][11];}
		
	}	
	outputFile.close();

	return 0;
}