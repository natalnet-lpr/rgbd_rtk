#include "kitti_stereo_loader.h"


using namespace cv;
using namespace std;

KITTIStereoLoader::KITTIStereoLoader(){

}
void KITTIStereoLoader::loaderImage(string left_path, string right_path){
	leftImage=imread(left_path);
	rightImage=imread(right_path);
 	


}
void KITTIStereoLoader::loaderImageSequence(string sequence_path,int sequence_num,bool colored){
	 char sequence_number[3];
	 s_path=sequence_path;
	 sprintf(sequence_number, "%02d", sequence_num);
	 s_num =string(sequence_number);
	 string systemCommand;
	 string indexPath;
	if(colored){
		systemCommand = "ls "+sequence_path+"dataset/sequences/"+s_num+"/image_2 >"+sequence_path+"dataset/sequences/"+s_num+"/image_2/index.txt";
		indexPath = sequence_path+"dataset/sequences/"+s_num+"/image_2/index.txt";
	}
	else{
		systemCommand = "ls "+sequence_path+"dataset/sequences/"+s_num+"/image_0 >"+sequence_path+"dataset/sequences/"+s_num+"/image_0/index.txt";
		indexPath = sequence_path+"dataset/sequences/"+s_num+"/image_0/index.txt";
	}

	system (systemCommand.c_str());

	ifstream myfile(indexPath.c_str()); 
	copy(istream_iterator<string>(myfile),
         istream_iterator<string>(),
         back_inserter(leftImageNameList));

	myfile.close();

	leftImageNameList.erase(leftImageNameList.begin()+leftImageNameList.size()-1);
	rightImageNameList=leftImageNameList;

 	num_images = leftImageNameList.size();
	
}

/*
void KITTISeteroLoader::loaderLidarCloud(string lidar_path){

}
void KITTISeteroLoader::loaderLidarCloudSequence(string lidar_path, sequence_num){

}
*/
 Mat KITTIStereoLoader::getLeftImage(){
 	return leftImage;
 }
 
 Mat KITTIStereoLoader::getRightImage(){
 	return rightImage;
 }
 
 Mat KITTIStereoLoader::getNextLeftSequence(bool colored){
 	string path;
 	Mat leftImageS;
 	if(colored){
 		path = s_path+"dataset/sequences/"+s_num+"/image_2/"+leftImageNameList[nextL];
 		leftImageS = imread(path,IMREAD_COLOR);
 	}
 	else{
 		path = s_path+"dataset/sequences/"+s_num+"/image_0/"+leftImageNameList[nextL];
 		leftImageS = imread(path,CV_LOAD_IMAGE_UNCHANGED);
 	}
 	nextL++;
 	return leftImageS;
 }
 

 Mat KITTIStereoLoader::getNextRightSequence(bool colored){
 	string path;
 	Mat rightImageS;
 	if(colored){
 		string path = s_path+"dataset/sequences/"+s_num+"/image_3/"+rightImageNameList[nextR];
 		rightImageS = imread(path,IMREAD_COLOR);
 	}
 	else{
 		string path = s_path+"dataset/sequences/"+s_num+"/image_1/"+rightImageNameList[nextR];
 		rightImageS = imread(path,CV_LOAD_IMAGE_UNCHANGED);
 	}
 	nextR++;
 	return rightImageS;
 }
KITTIStereoLoader::~KITTIStereoLoader(){
	leftImage.release();
	rightImage.release();
	s_path.clear();
	s_num.clear();
	leftImageNameList.clear();
	rightImageNameList.clear();


}






