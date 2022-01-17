#include "geometric_motion_segmenter.h"
#include "../common/geometry.h"
#include <cmath>
#include <pcl/common/transforms.h>
#include <algorithm>
#include <math.h>
#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>

GeometricMotionSegmenter::GeometricMotionSegmenter()
{
    prev_static_pts_ = boost::make_shared<std::vector<cv::Point2f>>();
    curr_static_pts_ = boost::make_shared<std::vector<cv::Point2f>>();
    dyna_pts_ = boost::make_shared<vector<cv::Point2f>>();
    initialized_ = false;
}
GeometricMotionSegmenter::GeometricMotionSegmenter(vector<Tracklet> * tracklets,
                                 const PointCloud<PointT>::Ptr curr_sparse_cloud,
                                 const PointCloud<PointT>::Ptr prev_sparse_cloud,
                                 const boost::shared_ptr<Eigen::Affine3f> & curr_cloud_relative_pose,
                                 std::vector<std::pair<int,int>> * mapper_2d_3d,
                                 float threshold,uint8_t dynamic_range,
                                 float mask_point_radius)
{
    prev_static_pts_ = boost::make_shared<std::vector<cv::Point2f>>();
    curr_static_pts_ = boost::make_shared<std::vector<cv::Point2f>>();
    dyna_pts_ = boost::make_shared<std::vector<cv::Point2f>>();
    tracklets_ = tracklets;
    curr_sparse_cloud_ = curr_sparse_cloud;
    prev_sparse_cloud_ = prev_sparse_cloud;
    curr_cloud_relative_pose_ = curr_cloud_relative_pose;
    mapper_2d_3d_ = mapper_2d_3d;
    threshold_ = threshold;
    initialized_ = false;
    dynamic_range_ = dynamic_range;
    mask_point_radius_ = mask_point_radius;

}
void printPoint2d(const cv::Point2d&  pt)
{
    std::cout << pt.x <<", "<< pt.y << "\n";
}
void printPoint3d(const PointT& pt)
{
    std::cout << pt.x <<", "<< pt.y << ", " << pt.z <<"\n";
}
bool print = false;
int nearest_idx = -1;
bool GeometricMotionSegmenter::isDynamicPoint(const PointT & pt_from, const PointT & pt_to, const float  threshold)
{
    auto convert3dTo2d = [](const PointT & pt3d, cv::Point2d & pt2d, const Intrinsics & cameraIntrsParameters)
                        {
                            double fx = cameraIntrsParameters.fx_;
                            double fy = cameraIntrsParameters.fy_;
                            double cx = cameraIntrsParameters.cx_;
                            double cy = cameraIntrsParameters.cy_;

                            double z = pt3d.z;
                            double x = pt3d.x;
                            double y = pt3d.y;
                            pt2d.x = (fx*x)/z + cx;
                            pt2d.y = (fy*y)/z + cy;
                        };

    cv::Point2d cv_pt_from, cv_pt_to;
    convert3dTo2d(pt_from,cv_pt_from,cameraIntrinsic_);
    convert3dTo2d(pt_to,cv_pt_to,cameraIntrinsic_);
 
    double dist = cv::norm(cv::Mat(cv_pt_from),cv::Mat(cv_pt_to));
    if(isnan(dist) || isinf(dist)  || pt_to.z > max_kinect_depth_)
    {
        return true;
    }
    bool is_dynamic = dist > threshold;
    if(print)
    {
        std::cout << "from " << cv_pt_from.x <<", "<< cv_pt_from.y <<"\n";
        std::cout <<  "to " << cv_pt_to.x <<", "<< cv_pt_to.y <<"\n";
        //exit(0);
        //printPoint3d(pt_from);
        //printPoint3d(pt_to);
        //std::cout << "dist " << dist <<"\n";

        //printPoint2d(cv_pt_from);
        //printPoint3d(pt_from);
    }
   
    return is_dynamic;
}
void GeometricMotionSegmenter::calculateDynamicPoints(const pcl::PointCloud<PointT> & sparse_cloud_from, 
            const pcl::PointCloud<PointT> & sparse_cloud_to,
            const Eigen::Affine3f & clouds_relative_pose,
            const float  threshold)
{
    // apply the relative transformation in the cloud
    pcl::PointCloud<PointT> transformed_cloud_from;
    pcl::transformPointCloud(sparse_cloud_from, transformed_cloud_from, clouds_relative_pose);
    
    double z = std::numeric_limits<double>::max();
    int saved_idx = -1;
    // loop over all cloud points and check if it is a dynamic point
    for(int i = 0 ; i < sparse_cloud_to.size(); i++)
    {
        const auto pt_from = (transformed_cloud_from)[i];
        const auto pt_to = (sparse_cloud_to)[i];

        // get the index based on tracker
        int idx = (*mapper_2d_3d_)[i].second;
        /*print = false;
        if( z > pt_to.z)
        {
            z = pt_to.z;
            saved_idx = idx;
        }
        if( idx == nearest_idx)
        {
            print = true;
        }*/
        if( idx_is_dyna_pts_map_.find(idx) == idx_is_dyna_pts_map_.end())
        {
            idx_is_dyna_pts_map_[idx] = 0;
        }

        if(isDynamicPoint(pt_from,pt_to,threshold))
        {   
            idx_is_dyna_pts_map_[idx]-=1;
        }
        else
        {
            idx_is_dyna_pts_map_[idx]+=1;
        }
        
    }
    if(nearest_idx == -1)
    {
        //std::cout << "indice mais proximo " << saved_idx <<"\n";
        nearest_idx = saved_idx;
    }
}
std::vector<cv::Point2f> createPointVectorFromTracklet
    (const std::vector<Tracklet> & tracklets_, uint reverse_idx = 0)
{
    vector<cv::Point2f> points;

    // store the last track value to build the
    // current points vector
    // prev -> 0  <- curr 
    for(int i = 0; i < tracklets_.size(); i++ )
    {
        auto point_it =  tracklets_[i].pts2D_.rbegin() + reverse_idx;
        points.push_back(*point_it);
    }

    return points;

}
void GeometricMotionSegmenter::calculateDynamicPoints()
{   
    // clear the data
    dyna_pts_->clear();
    *prev_static_pts_ = *curr_static_pts_;
    curr_static_pts_->clear();
    idx_is_dyna_pts_map_.clear();
    
    if(prev_sparse_cloud_->size() == 0 || curr_sparse_cloud_->size() == 0)  
    {
        return;
    }

    // push a pair of clouds and it relative pose
    sparse_clouds_pairs_.push_back(sparseCloudsPair(*prev_sparse_cloud_,*curr_sparse_cloud_));
    relative_poses_.push_back(*curr_cloud_relative_pose_);
    
    // if we reach the maximum size, we remove the first pair of clouds
    // and the first relative pose.
    if(sparse_clouds_pairs_.size() > dynamic_range_)
    {
        sparse_clouds_pairs_.erase(sparse_clouds_pairs_.begin());
        relative_poses_.erase(relative_poses_.begin());
    }
    // for each pair of cloud and relative pose,
    // determinate if the points is dynamic or not
    for(int i = sparse_clouds_pairs_.size() - 1; i >= 0; i--)
    {
        auto tgt_cloud = sparse_clouds_pairs_[i].first;
        auto src_cloud = sparse_clouds_pairs_[i].second;

        if(src_cloud.size() > curr_sparse_cloud_->size() || tgt_cloud.size() > curr_sparse_cloud_->size())
        {
            src_cloud.resize(curr_sparse_cloud_->size());
            tgt_cloud.resize(curr_sparse_cloud_->size());
        }

        auto relative_pose = relative_poses_[i];
        calculateDynamicPoints(tgt_cloud,src_cloud,relative_pose,threshold_);
    }

}
void GeometricMotionSegmenter::segment(const cv::Mat & in_img, cv::Mat & out_img)
{
    calculateDynamicPoints();

    cv::Mat output_mask(cv::Size(in_img.cols,in_img.rows),CV_8UC1,cv::Scalar(255));

    for(const auto & pt:*dyna_pts_)
    {
        cv::circle(output_mask,pt,mask_point_radius_,cv::Scalar(0),-1);
    }

    output_mask.copyTo(out_img);
}
std::vector<int> GeometricMotionSegmenter::getStaticPointsIndex()
{
    std::vector<int> idxs = {}; 
    for(const auto & idx_counter:idx_is_dyna_pts_map_)
    {
        int idx = idx_counter.first;
        int counter = idx_counter.second;

        if(counter >= static_threshold_ || counter > dynamic_threshold_)
        {
            idxs.push_back(idx);
        }
        
    }

    std::sort(idxs.begin(),idxs.end());

    return idxs;
}
std::vector<int> GeometricMotionSegmenter::getDynamicPointsIndex()
{
    std::vector<int> idxs = {};
    for(const auto & idx_counter:idx_is_dyna_pts_map_)
    {
        int idx = idx_counter.first;
        int counter = idx_counter.second;

        if(counter <= dynamic_threshold_)
        {
            idxs.push_back(idx);
        }
    }

    std::sort(idxs.begin(),idxs.end());

    return idxs;
}