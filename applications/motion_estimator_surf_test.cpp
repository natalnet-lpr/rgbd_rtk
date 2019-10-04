/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <cstdio>
#include <cstdlib>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../experiment/RPE.hpp"

#include <geometry.h>
#include <rgbd_loader.h>
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>
#include <surf_detector.h>
#include <visual_memory.h>
#include <string>
#include <fstream>
using namespace std;
std::fstream& GotoLine(std::fstream& file,  int num){
    file.seekg(std::ios::beg);
    for(int i=0; i < num - 1; ++i){
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}


bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}
Eigen::Matrix4f getGroundPose (string filename, int line_numb ){
	fstream file(filename.c_str());
	line_numb+=4;
	GotoLine(file,line_numb);
	string line;
	std::getline(file, line);

	std::istringstream iss(line);
	std::vector<std::string> splited_line((std::istream_iterator<std::string>(iss)),std::istream_iterator<std::string>());

	double 	T[3]; // tx,ty,tz
	double qx,qy,qz,qw;	//qx,qy,qz,qw
	//qx,qy,qz,qw
	T[0] = std::stod( splited_line[1]);
	T[1] = std::stod( splited_line[2]);
	T[2] = std::stod( splited_line[3]);
	qx = std::stod( splited_line[4]);
	qy = std::stod( splited_line[5]);
	qz = std::stod( splited_line[6]);
	qw = std::stod( splited_line[7]);
	Eigen::Matrix3f rotation =Eigen::Quaternionf(qx,qy,qz,qw).toRotationMatrix(); //qx,qy,qz,qw
	Eigen::Vector3f translation(T[0],T[1],T[2]);

	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	pose.block(0,0,3,3)  = rotation;
	pose.col(3).head<3>() = translation;

/*
	cout<<"pose matrix \n\n";

	cout<<pose<<endl<<endl;
*/
	return pose;




}
bool is_KF(vector<Point2f> curr_KPs, vector<Point2f> KPs_from_last_KF){
	int dif = curr_KPs.size()- KPs_from_last_KF.size();
	if(dif<0)
		dif = -dif;
	return  (
		dif
		>= (int)KPs_from_last_KF.size()*60/100
	);

}

int main(int argc, char **argv)
{

	string index_file_name;
	RGBDLoader loader;
	Intrinsics intr(0);
	MotionEstimatorRANSAC motion_estimator(intr);
	ReconstructionVisualizer visualizer;
    cv::Mat frame, depth,prev_frame;
	Eigen::Affine3f pose = Eigen::Affine3f::Identity();
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr curr_cloud(new pcl::PointCloud<PointT>);
	SurfDetector detector_surf(400);
	VisualMemory memory;
	if(argc != 3)
	{
		fprintf(stderr, "Usage: %s <index file> <frame_idx>\n", argv[0]);
		exit(0);
	}

	index_file_name = argv[1];
	std::string ground_file_name = argv[1];
	replace(ground_file_name,"index", "groundtruth");




	loader.processFile(index_file_name);

	vector<Point2f> last_KF_points;


	cv::Mat saved_descriptor;

	Mat img;
	int img_searched_idx = atoi(argv[2]);

	Eigen::Matrix4f searched_pose;
	Eigen::Matrix4f searched_ground_pose = 	getGroundPose(ground_file_name,atoi(argv[2]));

	vector<Eigen::Matrix4f> pose_set;
	for(int i = 0; i < loader.num_images_; i++)
	{

		//Load RGB-D image and point cloud
		loader.getNextImage(frame, depth);

		*curr_cloud = getPointCloud(frame, depth, intr);

	

		if(i > 0)
		{
			detector_surf.detectAndMatch(frame,prev_frame);


			detector_surf.drawSurfMatches(prev_frame,frame);
			trans = motion_estimator.estimate(detector_surf.prev_good_Pts_, prev_cloud,
				                            detector_surf.curr_good_Pts_, curr_cloud);
			pose = pose*trans;

			pose_set.push_back( pose.matrix());

		}

		if(i==img_searched_idx){
			img = frame;

			searched_pose = pose.matrix();
		}
		 if(is_KF(detector_surf.curr_good_Pts_,last_KF_points)){
			cout<<"keyframe adicionado, seu índice é: "<< i<<endl;

            last_KF_points = detector_surf.curr_good_Pts_;

			memory.add(detector_surf.getLastTrainDescriptor(),i, pose.matrix());


		}


		if(i == 0) visualizer.addReferenceFrame(pose, "origin");
		visualizer.addQuantizedPointCloud(curr_cloud, 0.3, pose);
		visualizer.viewReferenceFrame(pose);
	//	visualizer.viewPointCloud(curr_cloud, pose);
		visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, pose);

		visualizer.spinOnce();



        cv::imshow("Depth view", depth);

		char key = waitKey(1);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}


		//Let the prev. cloud in the next frame be the current cloud
		*prev_cloud = *curr_cloud;


		prev_frame = frame;

	}


	vector<int> img_idx_set = memory.searchImage(img);
	vector<Eigen::Matrix4f> ground_poses;
	vector<Eigen::Matrix4f> relative_ground_poses;
	vector<Eigen::Matrix4f> relative_estimated_poses;

	for(int i =0;i<img_idx_set.size();i++){
		int idx = img_idx_set[i];
		ground_poses.push_back(	getGroundPose(ground_file_name,img_idx_set[i]));

		relative_ground_poses.push_back( searched_pose.inverse()*ground_poses[i]);
		relative_estimated_poses.push_back(searched_pose.inverse()*pose_set[idx]);

	}
	cout<<"passou\n\n\n";

	cout<<"\n\nvalor do erro RPE: "<<RPE(relative_estimated_poses,relative_ground_poses)<<endl;



	return 0;
}
