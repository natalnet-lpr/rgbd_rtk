#include "geometric_motion_segmenter.h"
#include "../common/geometry.h"
#include <cmath>
#include <pcl/common/transforms.h>
#include <algorithm>
#include <math.h>

GeometricMotionSegmenter::GeometricMotionSegmenter()
{
    dyna_pts_ = std::make_shared<vector<cv::Point2f>>();
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
GeometricMotionSegmenter::GeometricMotionSegmenter(const PointCloud<PointT>::Ptr cloud_from,
                                vector<cv::Point2f> * points_from,
                                const PointCloud<PointT>::Ptr cloud_to,
                                vector<cv::Point2f> * points_to,
                                const shared_ptr<Eigen::Affine3f> & clouds_relative_pose, float threshold)
{
    points_to_ = points_to;
    points_from_ = points_from;
    dense_cloud_from_ = cloud_from;
    dense_cloud_to_ = cloud_to;
    clouds_relative_pose_ = clouds_relative_pose;
    threshold_ = threshold;
}
void GeometricMotionSegmenter::segment(const cv::Mat & in_img, cv::Mat & out_img)
{
    assert(("not implemented yet ", false));
}

bool GeometricMotionSegmenter::isDynamicPoint(const PointT & pt_from, const PointT & pt_to, const float  threshold)
{
    // serialize the points inside Eigen::Vector3fs
    Eigen::Vector3f eg_pt_from({pt_from.x,pt_from.y,pt_from.z});
    Eigen::Vector3f eg_pt_to({pt_to.x,pt_to.y,pt_to.z});
        
    // calculate the distance between the projected point and the to point
    // and return true if this is greater than a threshold
    float dist = abs(eg_pt_to.norm() - eg_pt_from.norm());
    if(isnan(dist) || isinf(dist))
    {
        return false;
    }

    return  dist > threshold;
}
bool GeometricMotionSegmenter::isDynamicPoint(const PointT & pt_from, const PointT & pt_to) const
{
    return GeometricMotionSegmenter::isDynamicPoint(pt_from, pt_to, threshold_);
}
void getPointsFromClound(const pcl::PointCloud<PointT> & cloud, std::vector<PointT> & pts)
{
    for(const auto & pt:cloud)
    {
        if(is_valid(pt))
        {
            pts.push_back(pt);
        }
    }
}   

void GeometricMotionSegmenter::calculateDynamicPoints(const pcl::PointCloud<PointT> & sparse_cloud_from, 
            const pcl::PointCloud<PointT> & sparse_cloud_to, const Eigen::Affine3f & clouds_relative_pose,
            std::vector<cv::Point2f> & dyna_pts, const float  threshold, std::vector<int> & idxs_to)
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
            dyna_pts.push_back((*points_to_)[idxs_to[k]]);
            k++;
        }
    }
}

void GeometricMotionSegmenter::calculateDynamicPoints()
{
    dyna_pts_ = std::make_shared<std::vector<cv::Point2f>>() ;
    
    PointCloud<PointT> sparse_cloud_from;
    vector<int> idxs_from;

    featureVector2SparseCloud(dense_cloud_from_,*points_from_,sparse_cloud_from,idxs_from);
    PointCloud<PointT> sparse_cloud_to;
    vector<int> idxs_to;
    featureVector2SparseCloud(dense_cloud_to_,*points_to_,sparse_cloud_to, idxs_to);

    calculateDynamicPoints(sparse_cloud_from,sparse_cloud_to,*clouds_relative_pose_,
                                                        (*dyna_pts_),threshold_,idxs_to);
}