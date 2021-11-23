#include "dnn_motion_segmenter.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
using namespace cv;
DNNMotionSegmenter::DNNMotionSegmenter(const std::string & model_path,const std::string & config_file, classMap && class_map): 
    model_path_(model_path), config_file_(config_file)
{
    net_ = cv::dnn::readNetFromTensorflow(model_path_,config_file_);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    setClassesMap(std::move(class_map));
    
}

void DNNMotionSegmenter::segment( const cv::Mat & in_img, cv::Mat & out_img)
{
    const float scale_factor = 1.0;
    bool crop = false;
    bool swapRB = true;

    cv::Mat blob_img = cv::dnn::blobFromImage(in_img,scale_factor,cv::Size(),cv::Scalar(),swapRB,crop);

    net_.setInput(blob_img);

    std::vector<cv::Mat> outputs;
    std::vector<std::string> outputs_name = {"detection_out_final","detection_masks"};
    
    auto dnn_start = std::chrono::steady_clock::now();
    net_.forward(outputs,outputs_name);
    auto dnn_end = std::chrono::steady_clock::now();
    
    std::chrono::duration<double> diff = dnn_end - dnn_start;

    this->postProcess(in_img,out_img,outputs,threshold_);

}


void DNNMotionSegmenter::postProcess(const Mat & in_img,Mat & out_img, const std::vector<Mat> outputs,float threshold) const
{
    Mat detections = outputs[0];
    Mat masks = outputs[1];
    // this will be copyed to out_img
    Mat output_mask(in_img.rows,in_img.cols,CV_8UC1);

    // Output size of masks is NxCxHxW where
	// N - number of detected boxes
	// C - number of classes (excluding background)
	// HxW - segmentation shape
    const int num_detections = detections.size[2];
    const int num_classes = masks.size[1];
        
    // each detection contains 7 elements,
    // - no information (0)
    // - class id
    // - score
    // - left coordinate
    // - top coordinate
    // - right coordinate
    // - bottom coordinate
    
    detections = detections.reshape(1, detections.total() / 7); 


    for (int i = 0; i < num_detections; i++)
    {
        float score = detections.at<float>(i,2);
        
        if (score > threshold)
        {
            int class_id = int(detections.at<float>(i,1));

            if(classes_map_.find(class_id) != classes_map_.end())
            {
                int left = int(in_img.cols * detections.at<float>(i,3));
                int top = int(in_img.rows * detections.at<float>(i,4));
                int right = int(in_img.cols * detections.at<float>(i,5));
                int bottom = int(in_img.rows * detections.at<float>(i,6));

                left = max(0, min(left, in_img.cols - 1));
                top = max(0, min(top, in_img.rows - 1));
                right = max(0, min(right, in_img.cols - 1));
                bottom = max(0, min(bottom, in_img.rows - 1));
                
                Rect box = Rect(left, top, right - left + 1, bottom - top + 1);
                
                // Extract the mask for the object
                Mat mask(masks.size[2], masks.size[3],CV_32F, masks.ptr<float>(i,class_id));
                
                //rectangle(in_img, Point(box.x, box.y), Point(box.x+box.width, box.y+box.height), Scalar(255, 178, 50), 2);        

                cv::Rect roi(Point(box.x, box.y),Point(box.x+box.width, box.y+box.height));            

                resize(mask,mask,box.size());

                // convert the mask to a binary image
                mask = mask > threshold;

                Mat mask_roi = output_mask(roi);

                mask.copyTo(mask_roi);

            }

            /*Mat colored_roi = (0.7*Scalar(255,178,50) + 0.6*in_img(box));
            colored_roi.convertTo(colored_roi, CV_8UC3);
            
            
            colored_roi.copyTo(frame(box),mask);*/
            
           
        }

        
    }

    output_mask.copyTo(out_img);
}
