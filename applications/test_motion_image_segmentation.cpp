#include <iostream>
#include <factory_motion_segmenter.h>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <config_loader.h>
using namespace std;
typedef std::unordered_map<int,std::string> classMap;

int main(int argc, char ** argv)
{       

    if (argc != 2)
    {
        cout << "usage: ./test_motion_image_segmentation <yml_config_filepath>" << endl;
        return 1;
    }

    // load parameters
    ConfigLoader param_loader(argv[1]);
    std::string model_path;  
    param_loader.checkAndGetString("dnn_model_path",model_path);
    std::string config_file; 
    param_loader.checkAndGetString("dnn_config_file_path",config_file);
    std::string img_path;
    param_loader.checkAndGetString("image_path",img_path);

    // create image segmentor
    auto segmenter = FactoryMotionSegmenter::create(MotionSegmenterType::DNN,model_path,
                            config_file,classMap({{0,"person"},{2,"car"}}));
   
    cv::Mat in_img = cv::imread(img_path);
    cv::Mat out_mask;
    
    // segment the image
    segmenter->segment(in_img, out_mask);

    cv::imshow("original img", in_img);
    cv::imshow("mask", out_mask);

    in_img.convertTo(in_img,CV_8UC1);
    cv::cvtColor(in_img,in_img,cv::COLOR_BGR2GRAY);
    
    bitwise_and(in_img,out_mask,in_img);
    cv::imshow("masked img", in_img);

    cv::waitKey(10000);

    return 0;
}