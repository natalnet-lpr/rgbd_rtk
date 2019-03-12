//export LD_LIBRARY_PATH=/home/vanessadantas/Downloads/opencv-3.4.1/Build/lib
#include <iostream>
#include <opencv2/opencv.hpp>
#include "kitti_stereo_loader.h"

using namespace std;
using namespace cv;


int main()
{
  KITTIStereoLoader l;
  string path = "/home/vanessadantas/Music/KITTIStereoLib/KSL/";
  Mat im;

  l.loaderImageSequence(path,0,false);
  im=l.getNextLeftSequence(false);
  imshow("Teste",im);
  waitKey();
  
    return 0;
}

