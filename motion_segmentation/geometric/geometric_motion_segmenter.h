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
        GeometricMotionSegmenter(vector<Tracklet> * tracklets,
                                 const PointCloud<PointT>::Ptr curr_cloud,
                                 const PointCloud<PointT>::Ptr prev_cloud,
                                 const shared_ptr<Eigen::Affine3f> & curr_cloud_relative_pose, 
                                 float threshold, 
                                 uint8_t dynamic_range=5,
                                 float mask_point_radius=4.0f);
                        
        virtual void segment(const cv::Mat & in_img, cv::Mat & out_img);
        // assuming that these points are already projected in it respectives poses
        static bool isDynamicPoint(const PointT & pt_from, const PointT & pt_to, const float threshold);

        bool isDynamicPoint(const PointT & pt_from, const PointT & pt_to) const;

        void calculateDynamicPoints(const pcl::PointCloud<PointT> & sparse_cloud_from, 
            const pcl::PointCloud<PointT> & sparse_cloud_to,  const std::vector<cv::Point2f> & curr_pts,
            const Eigen::Affine3f & clouds_relative_pose,std::vector<cv::Point2f> & dyna_pts, 
            const float  threshold, const std::vector<int> & idxs_to);
                                            
        void calculateDynamicPoints();

        inline shared_ptr<vector<cv::Point2f>> getDynaPoints(){
            return dyna_pts_;
        };
    protected:

        //  current dynamic points
        shared_ptr<vector<cv::Point2f>> dyna_pts_;

        // This variable determinate how many poses will be
        // used to determinate if an object is dynamic or not
        uint8_t dynamic_range_;

        // store a reference to the tracklets vector
        vector<Tracklet> * tracklets_;
        // store a reference to the current dense cloud
        PointCloud<PointT>::Ptr curr_dense_cloud_;
        // store a reference to the previous dense cloud
        PointCloud<PointT>::Ptr prev_dense_cloud_;
        // store a reference to the current relative pose 
        std::shared_ptr<Eigen::Affine3f> curr_cloud_relative_pose_;
        
        // store the sparses point clouds
        vector<PointCloud<PointT>> sparse_clouds_;
        // store the relative poses
        vector<Eigen::Affine3f> relative_poses_;

        bool initialized_;
        
        float mask_point_radius_;


};