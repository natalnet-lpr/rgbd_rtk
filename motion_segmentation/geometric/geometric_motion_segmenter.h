#pragma once
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <utility>
#include <pcl/point_cloud.h>
#include "../../common/common_types.h"
#include "../motion_segmenter.h"

using std::shared_ptr;
using std::vector;
using pcl::PointCloud;

class GeometricMotionSegmenter : public MotionSegmenter{
    public:
        GeometricMotionSegmenter();
        GeometricMotionSegmenter(const PointCloud<PointT>::Ptr cloud_from,
                                vector<cv::Point2f> * points_from,
                                 const PointCloud<PointT>::Ptr cloud_to,
                                vector<cv::Point2f> * points_to,
                                 const shared_ptr<Eigen::Affine3f> & clouds_relative_pose, float threshold);
                        
        virtual void segment(const cv::Mat & in_img, cv::Mat & out_img);
        // assuming that these points are already projected in it respectives poses
        static bool isDynamicPoint(const PointT & pt_from, const PointT & pt_to, const float threshold);

        bool isDynamicPoint(const PointT & pt_from, const PointT & pt_to) const;

        void calculateDynamicPoints(const pcl::PointCloud<PointT> & sparse_cloud_from, 
            const pcl::PointCloud<PointT> & sparse_cloud_to, const Eigen::Affine3f & clouds_relative_pose,
            std::vector<cv::Point2f> & dyna_pts, const float  threshold,std::vector<int>  & idxs_to);
                                            
        void calculateDynamicPoints();

        inline shared_ptr<vector<cv::Point2f>> getDynaPoints(){
            return dyna_pts_;
        };
    protected:

        // This variable determinate how many poses will be
        // used to determinate if an object is dynamic or not
        uint16_t pose_range_;

        // store a reference to the tracklets vector
        std::shared_ptr<std::vector<std::vector<PointT>>> tracklets_;

        //  current dynamic points
        shared_ptr<vector<cv::Point2f>> dyna_pts_;
        pcl::PointCloud<PointT>::Ptr dense_cloud_from_;
        vector<cv::Point2f> * points_from_;
        pcl::PointCloud<PointT>::Ptr dense_cloud_to_;
        vector<cv::Point2f> * points_to_;
        std::shared_ptr<Eigen::Affine3f> clouds_relative_pose_;


};