#include <iostream>
#include <factory_motion_segmenter.h>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <config_loader.h>
#include <rgbd_loader.h>

using namespace std;
typedef std::unordered_map<int,std::string> classMap;
void mergeMask(cv::Mat & img, cv::Mat mask)
{
    img.convertTo(img,CV_8UC1);
    cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);
    bitwise_and(img,mask,img);
}
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
    bool is_video;
    param_loader.checkAndGetBool("is_video",is_video);
    bool resize_src;
    param_loader.checkAndGetBool("resize_src",resize_src);
    string src_path;
    param_loader.checkAndGetString("src_path",src_path);
    

    // create image segmentor
    auto segmenter = FactoryMotionSegmenter::create<DNNMotionSegmenter>(model_path,
                            config_file,classMap({{0,"person"},{2,"car"}}));
    segmenter->setThreshold(0.3);

    if(is_video)
    {           
        cv::VideoCapture cap(src_path);

        if(!cap.isOpened()){
            cerr << "invalid video file "<< src_path << endl;
            exit(-1);
        }
        cv::Mat frame;
        while(cap.isOpened())
        {
            cv::Mat mask;
            cap >> frame;
            if(resize_src)
            {
                cv::resize(frame,frame,cv::Size(320,240));
            }
            // segment the image
            segmenter->segment(frame, mask);

            cv::imshow("original img", frame);
            cv::imshow("mask", mask);

            mergeMask(frame,mask);

            cv::imshow("masked img", frame);

            char k = cv::waitKey(1);
            if ( k == 27)
            {
                break;
            }
        }
        return 0;
        
    }
    else{

        param_loader.checkAndGetString("src_path",src_path);
    
        cv::Mat in_img = cv::imread(src_path);
        if(resize_src)
        {
            cv::resize(in_img,in_img,cv::Size(300,300));
        }
        if(in_img.empty()){
            cerr << "invalid img file "<< src_path << endl;
            exit(-1);
        }
        cv::Mat mask;
        // segment the image
        segmenter->segment(in_img, mask);

        cv::imshow("original img", in_img);
        cv::imshow("mask", mask);

        mergeMask(in_img,mask);

        cv::imshow("masked img", in_img);

        cv::waitKey(1);
    
    }
   

    return 0;
}