#pragma once
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <utility>
#include <pcl/point_cloud.h>
#include <unordered_map>
#include "../../common/common_types.h"
#include "../../common/common_types.h"
#include "../motion_segmenter.h"

using std::vector;
using pcl::PointCloud;

class GeometricMotionSegmenter : public MotionSegmenter{
    public:
        GeometricMotionSegmenter();
        GeometricMotionSegmenter(vector<Tracklet> * tracklets,
                                 const PointCloud<PointT>::Ptr curr_sparse_cloud,
                                 const PointCloud<PointT>::Ptr prev_saparse_cloud,
                                 const boost::shared_ptr<Eigen::Affine3f> & curr_cloud_relative_pose, 
                                 std::vector<int> * mapper_2d_3d,
                                 float threshold, 
                                 uint8_t dynamic_range=5,
                                 float mask_point_radius=4.0f);
                        
        virtual void segment(const cv::Mat & in_img, cv::Mat & out_img);
        // assuming that these points are already projected in it respectives poses
        bool isDynamicPoint(const PointT & pt_from, const PointT & pt_to, const float threshold);

        void calculateDynamicPoints(const pcl::PointCloud<PointT> & sparse_cloud_from, 
            const pcl::PointCloud<PointT> & sparse_cloud_to,
            const Eigen::Affine3f & clouds_relative_pose,
            const std::vector<int>& mappers_2d_3d,
            const float  threshold);
                                            
        void calculateDynamicPoints();

        inline boost::shared_ptr<vector<cv::Point2f>> getDynaPoints(){
            return dyna_pts_;
        };
        inline boost::shared_ptr<vector<cv::Point2f>> getCurrStaticPoints()
        {
            return curr_static_pts_;
        }
        inline boost::shared_ptr<vector<cv::Point2f>> getPrevStaticPoints()
        {
            return prev_static_pts_;
        }
        std::vector<int> getStaticPointsIndex();
        std::vector<int> getDynamicPointsIndex();

        inline void setIntrinsics(Intrinsics & cameraIntrinsic)
        {
            cameraIntrinsic_ = cameraIntrinsic;
        }

    protected:
        float max_kinect_depth_ = 3.5f;
        int static_threshold_ = 3;
        int dynamic_threshold_ = -6;
        Intrinsics cameraIntrinsic_;

        std::unordered_map<int,int> idx_is_dyna_pts_map_;
        std::vector<int> * mapper_2d_3d_;
        std::vector<std::vector<int>>  mappers_2d_3d_;

        // current static points
        boost::shared_ptr<vector<cv::Point2f>> curr_static_pts_;
        // previous static points
        boost::shared_ptr<vector<cv::Point2f>> prev_static_pts_;

        //  current dynamic points
        boost::shared_ptr<vector<cv::Point2f>> dyna_pts_;

        // This variable determinate how many poses will be
        // used to determinate if an object is dynamic or not
        uint8_t dynamic_range_;

        // store a reference to the current tracklets vector
        vector<Tracklet> * tracklets_;

        // store a reference to the current sparse cloud
        PointCloud<PointT>::Ptr curr_sparse_cloud_;
        // store a reference to the previous sparse cloud
        PointCloud<PointT>::Ptr prev_sparse_cloud_;
        // store a reference to the current relative pose 
        boost::shared_ptr<Eigen::Affine3f> curr_cloud_relative_pose_;
        
        // store the sparses point clouds
        vector<PointCloud<PointT>> sparse_clouds_;
        // store the relative poses
        vector<Eigen::Affine3f> relative_poses_;

        // store the clouds in pairs
        typedef std::pair<PointCloud<PointT>,PointCloud<PointT>> sparseCloudsPair;
        std::vector<sparseCloudsPair> sparse_clouds_pairs_;

        bool initialized_;
        
        float mask_point_radius_;


};