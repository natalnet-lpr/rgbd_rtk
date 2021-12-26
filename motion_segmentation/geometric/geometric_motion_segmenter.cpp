#include "geometric_motion_segmenter.h"
#include "../common/geometry.h"
#include <cmath>
#include <pcl/common/transforms.h>
#include <algorithm>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>

GeometricMotionSegmenter::GeometricMotionSegmenter()
{
    dyna_pts_ = boost::make_shared<vector<cv::Point2f>>();
    initialized_ = false;
}
GeometricMotionSegmenter::GeometricMotionSegmenter(vector<Tracklet> * tracklets,
                                 const PointCloud<PointT>::Ptr curr_cloud,
                                 const PointCloud<PointT>::Ptr prev_cloud,
                                 const boost::shared_ptr<Eigen::Affine3f> & curr_cloud_relative_pose, 
                                 float threshold,uint8_t dynamic_range,
                                 float mask_point_radius)
{
    dyna_pts_ = boost::make_shared<std::vector<cv::Point2f>>() ;
    tracklets_ = tracklets;
    curr_dense_cloud_ = curr_cloud;
    prev_dense_cloud_ = prev_cloud;
    curr_cloud_relative_pose_ = curr_cloud_relative_pose;
    threshold_ = threshold;
    initialized_ = false;
    dynamic_range_ = dynamic_range;
    mask_point_radius_ = mask_point_radius;

}

void featureVector2SparseCloud(const pcl::PointCloud<PointT>::Ptr dense_cloud,  vector<cv::Point2f> & vec, 
pcl::PointCloud<PointT> & sparse_cloud,vector<int> & idxs){
    
    idxs.clear();
    vec.erase(
        std::remove_if(vec.begin(),vec.end(),[&](const cv::Point2f & pt2d){
            return (is_valid(get3Dfrom2D(pt2d,dense_cloud)) == false);
    }),vec.end());

    int idx = 0;

    for(auto it = vec.begin(); it != vec.end() ; ++it)
    {
        auto pt2d = *it;
        auto pt3d = get3Dfrom2D(pt2d,dense_cloud);
        if(is_valid(pt3d))
        {   
            sparse_cloud.push_back(pt3d);
            idxs.push_back(idx);
            idx ++;
        }     
    }
}

bool GeometricMotionSegmenter::isDynamicPoint(const PointT & pt_from, const PointT & pt_to, const float  threshold)
{
    // serialize the points inside Eigen::Vector3fs
    Eigen::Vector3f eg_pt_from({pt_from.x,pt_from.y,pt_from.z});
    Eigen::Vector3f eg_pt_to({pt_to.x,pt_to.y,pt_to.z});
        
    // calculate the distance between the projected point and the to point
    // and return true if this is greater than a threshold
    float dist = abs(eg_pt_to.norm() - eg_pt_from.norm());
    constexpr float max_distance = 2.5f;
    if(isnan(dist) || isinf(dist) || dist > max_distance)
    {
        return false;
    }
  
    return  dist > threshold;
}
bool GeometricMotionSegmenter::isDynamicPoint(const PointT & pt_from, const PointT & pt_to) const
{
    return GeometricMotionSegmenter::isDynamicPoint(pt_from, pt_to, threshold_);
}
void GeometricMotionSegmenter::calculateDynamicPoints(const pcl::PointCloud<PointT> & sparse_cloud_from, 
            const pcl::PointCloud<PointT> & sparse_cloud_to,  const std::vector<cv::Point2f> & curr_pts,
            const Eigen::Affine3f & clouds_relative_pose,
            const float  threshold, const std::vector<int> & idxs_to)
{
    // apply the relative transformation in the cloud
    pcl::PointCloud<PointT> transformed_cloud_from;
    pcl::transformPointCloud(sparse_cloud_from, transformed_cloud_from, clouds_relative_pose);
    
    // loop over all cloud points and check if it is a dynamic point
    auto it_from = transformed_cloud_from.begin();
    auto it_to = sparse_cloud_to.begin(); 

    int k = 0;

    for(int i=0; i < sparse_cloud_from.size(); i++)
    {
        const auto pt_from = (transformed_cloud_from)[i];
        const auto pt_to = (sparse_cloud_to)[i];
        // this is a test to discard nan points like this:
        // PointXYZ: x-> nan, y-> nan, z->nan;
        if(!is_valid(pt_from) && !is_valid(pt_from))
        {
            continue;
        }
        if(isDynamicPoint(pt_from,pt_to,threshold))
        {   
            if(k >= idxs_to.size() )
                break;
            int idx = idxs_to[k];
            if( idx_is_dyna_pts_map_.find(idx) == idx_is_dyna_pts_map_.end())
            {
                idx_is_dyna_pts_map_[idx] = 0;
            }
            idx_is_dyna_pts_map_[idx] += 1;
            //dyna_pts.push_back((curr_pts)[idx]);
            k++;
        }
    }
}
std::vector<cv::Point2f> createPointVectorFromTracklet
    (const std::vector<Tracklet> & tracklets_, uint reverse_idx = 0)
{
    vector<cv::Point2f> points;
    reverse_idx++;
    // store the last track value to build the
    // current points vector
    for(const auto & tracklet:tracklets_)
    {
        auto it =  tracklet.pts2D_.end() - reverse_idx;
        points.push_back(*it);
    }

    return points;

}
void GeometricMotionSegmenter::calculateDynamicPoints()
{
    dyna_pts_->clear();
    auto curr_pts = createPointVectorFromTracklet(*tracklets_);
    PointCloud<PointT> curr_sparse_cloud;
    
    vector<int> point_idxs;
    
    //PointCloud<PointT>::Ptr prev_sparse_cloud(new  PointCloud<PointT>());
    PointCloud<PointT> prev_sparse_cloud;

    if(!initialized_)
    {
        //prev_sparse_cloud = pcl::makeShared();
        auto prev_pts = createPointVectorFromTracklet(*tracklets_,1);
        featureVector2SparseCloud(prev_dense_cloud_,prev_pts,prev_sparse_cloud,point_idxs);
        sparse_clouds_.push_back(prev_sparse_cloud);
        initialized_ = true;
        return;
    }
    else{
        constexpr uint8_t prev_cloud_idx = 1;
        auto prev_sparse_cloud_it = sparse_clouds_.end() - prev_cloud_idx;
        prev_sparse_cloud = *prev_sparse_cloud_it;
    }
    
    
    featureVector2SparseCloud(curr_dense_cloud_,curr_pts,curr_sparse_cloud,point_idxs);
    
    sparse_clouds_.push_back(curr_sparse_cloud);
    relative_poses_.push_back(*curr_cloud_relative_pose_);

    if(sparse_clouds_.size() > dynamic_range_)
    {
        sparse_clouds_.erase(sparse_clouds_.begin());
    }
    if(relative_poses_.size() > dynamic_range_)
    {
        relative_poses_.erase(relative_poses_.begin());
    }
    idx_is_dyna_pts_map_.clear();

    for(uint8_t i = 1; i < relative_poses_.size(); i++)
    {   
        uint8_t prev_cloud_idx = i - 1;
        auto prev_sparse_cloud = *(sparse_clouds_.begin() + prev_cloud_idx);
        auto curr_sparse_cloud = *(sparse_clouds_.begin() + i);

        calculateDynamicPoints(prev_sparse_cloud,curr_sparse_cloud,
                           curr_pts, (relative_poses_[i-1]),
                           threshold_,point_idxs);
        
    }
    for(const auto & idx_counter:idx_is_dyna_pts_map_)
    {
        if(idx_counter.second)
        {
            dyna_pts_->push_back(curr_pts[idx_counter.first]);
        }
    }
    
}
void GeometricMotionSegmenter::segment(const cv::Mat & in_img, cv::Mat & out_img)
{
    calculateDynamicPoints();

    cv::Mat output_mask(cv::Size(in_img.cols,in_img.rows),CV_8UC3,cv::Scalar(255,255,255));

    for(const auto & pt:*dyna_pts_)
    {
        cv::circle(output_mask,pt,mask_point_radius_,cv::Scalar(0,0,0),-1);
    }

    output_mask.copyTo(out_img);
}