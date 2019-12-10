#ifndef INCLUDE_CONFIG_LOADER_H_
#define INCLUDE_CONFIG_LOADER_H_

#include <cstdio>
#include <cstdlib>
#include <fstream>

using namespace std;

class ConfigLoader{
    public:
        string camera_calibration_file_;
        string index_file_;
        string aruco_dic_;
        double aruco_distance_;
        float aruco_marker_size_;
        
    void loadParams(string filename);
};

#endif
