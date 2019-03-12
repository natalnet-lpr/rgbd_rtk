#include "velodyne_loader.h"


VelodyneLoader::VelodyneLoader(){

	cloud_velodyne = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	cloud_velodyne_rgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void VelodyneLoader::loaderPathSequence(string sequence_path,int sequence_num){
	char sequence_number[3];
	s_path=sequence_path;
	sprintf(sequence_number, "%02d", sequence_num);
	s_num =string(sequence_number);
	string systemCommand;
	string indexPath;
	
	systemCommand = "ls "+sequence_path+"dataset/sequences/"+s_num+"/velodyne >"+sequence_path+"dataset/sequences/"+s_num+"/velodyne/index.txt";
	indexPath = sequence_path+"dataset/sequences/"+s_num+"/velodyne/index.txt";

	system (systemCommand.c_str());

	ifstream myfile(indexPath.c_str()); 
	copy(istream_iterator<string>(myfile),
         istream_iterator<string>(),
         back_inserter(lidarBinNameList));

	myfile.close();

	lidarBinNameList.erase(lidarBinNameList.begin()+lidarBinNameList.size()-1);


 	num_bins = lidarBinNameList.size();
}
pcl::PointCloud<pcl::PointXYZI>::Ptr VelodyneLoader::getPointCloud(){


	cloud_velodyne->clear();

	string infile=s_path+"dataset/sequences/"+s_num+"/velodyne/"+lidarBinNameList[next];
	next++;

	fstream input(infile.c_str(), ios::in | ios::binary);


	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *) &point.x, sizeof(float));
		input.read((char *) &point.y, sizeof(float));
		input.read((char *) &point.z, sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		cloud_velodyne->push_back(point);
	}
	input.close();

    //pcl::PCDWriter writer;

    // Save DoN features
	//writer.write<pcl::PointXYZI> ("outfile_lidar.pcd", *cloud_velodyne, false);
	
	return cloud_velodyne;

}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr VelodyneLoader::getPointCloudRGB(){
	cloud_velodyne_rgb->clear();

	string infile=s_path+"dataset/sequences/"+s_num+"/velodyne/"+lidarBinNameList[next];
	next++;

	fstream input(infile.c_str(), ios::in | ios::binary);

	int i;
	for (i=0; input.good() && !input.eof(); i++) {
      pcl::PointXYZRGB point;
       char trash_i;
		input.read((char *) &point.x, sizeof(float));
		input.read((char *) &point.y, sizeof(float));
		input.read((char *) &point.z, sizeof(float));
		input.read((char *) &trash_i, sizeof(float));
		point.r=0;
		
		point.g=0;
		
		point.b=255;
		

		cloud_velodyne_rgb->push_back(point);

	}
	input.close();

	return cloud_velodyne_rgb;

}

VelodyneLoader::~VelodyneLoader(){

}






