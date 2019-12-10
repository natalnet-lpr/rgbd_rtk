#include <cstdio>
#include <cstdlib>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "config_loader.h"


using namespace std;
using namespace cv;
/**
 * Load configfile params
 * @Params string , path where the configfile is
 */
void ConfigLoader::loadParams(string filename){
  FileStorage fs(filename,FileStorage::READ);  //Reading config file
  if(fs.isOpened() == false){
    cout<<"ConfigFile couldn't be opened, check if your path is right\n";
    exit(0);
  }

  try{//Loading Params
    fs["camera_calibration_file"] >> camera_calibration_file_;
    fs["index_file"] >> index_file_;
    fs["aruco_dic"] >> aruco_dic_;
    fs["aruco_distance"] >> aruco_distance_;
    fs["aruco_marker_size"] >> aruco_marker_size_;
    //looking if any of the params was not loaded
    if(camera_calibration_file_.empty() || index_file_.empty() ||aruco_dic_.empty() || aruco_distance_ == 0 || aruco_marker_size_ == 0) throw 1;
   
    cout<<"Params load successfully\n"<<"camera_calibration_file: "<<camera_calibration_file_<<endl;
    cout<<"inex_file: "<<index_file_<<endl<<"aruco_dic: "<<aruco_dic_<<endl;
    cout<<"aruco_distance: "<<aruco_distance_<<endl<<"aruco_marker_size: "<<aruco_marker_size_<<endl;
  }
  catch(int e){//If a param was not loaded, use the default
    cout<<"Coudn't load the params, at least one of the param names is wrong\n\n";
    cout<<"Using default values\ncamera_calibration_file: \"kinect_default.yaml\"\n";
    cout<<"index_file: \"index\"\n";
    cout<<"aruco_dic: \"ARUCO\"\naruco_pose_file: \"aruco_poses\"\naruco_marker_size: 0.1778\n";
    camera_calibration_file_ = "../kinect_default.yaml";
    index_file_ = "index";
    aruco_dic_ = "ARUCO";
    aruco_distance_ = 4;
    aruco_marker_size_ = 0.1778;
  }
}