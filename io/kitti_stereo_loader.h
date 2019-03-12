#ifndef KITTISTEREOLOADER_H
#define KITTISTEREOLOADER_H

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>

using namespace cv;
using namespace std;


class KITTIStereoLoader
{
private:
	Mat leftImage;
	Mat rightImage;
	string s_path;
	string s_num;
	int nextL=0;
 	int nextR=0;
	vector<string> leftImageNameList;
	vector<string> rightImageNameList; 

    
public:
	int num_images=0;
	KITTIStereoLoader();
	void loaderImage(string left_path,string right_path);
	void loaderImageSequence(string sequence_path,int sequence_num,bool colored);
    Mat getLeftImage();
    Mat getRightImage();
    Mat getNextLeftSequence(bool colored); 
    Mat getNextRightSequence(bool colored);

    ~KITTIStereoLoader();


};

#endif // KITTISTEREOLOADER_H 

