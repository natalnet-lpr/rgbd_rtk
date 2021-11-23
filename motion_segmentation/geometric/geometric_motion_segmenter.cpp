#include "geometric_motion_segmenter.h"
#include "../common/geometry.h"
#include <cmath>

GeometricMotionSegmenter::GeometricMotionSegmenter()
{
    curr_dyna_pts_ = std::make_shared<std::vector<cv::Point2f>>();
}
void GeometricMotionSegmenter::segment(const cv::Mat & in_img, cv::Mat & out_img)
{

}
void GeometricMotionSegmenter::set3DTracklets(std::shared_ptr<std::vector<std::vector<PointT>>> tracklets)
{
    tracklets_ = tracklets;
}

bool GeometricMotionSegmenter::isDynamicPoint(const PointT & pt_from, const PointT & pt_to) const
{
    // serialize the points inside Eigen::Vector3fs
    Eigen::Vector3f eg_pt_from({pt_from.x,pt_from.y,pt_from.z});
    Eigen::Vector3f eg_pt_to({pt_to.x,pt_to.y,pt_to.z});

    // calculate the distance between the projected point and the to point
    // and return true if this is greater than a threshold
    return abs(eg_pt_to.norm() - eg_pt_from.norm()) > threshold_;

}

void GeometricMotionSegmenter::calculateDynamicPoints(const std::vector<PointT> & pts_from)
{
    curr_dyna_pts_->clear();
    // loop over the the tracklets points to
    // figure out which of them is dynamic or not
    for( uint16_t i = 0; i < pts_from.size(); i++)
    {
        if( i >= tracklets_->size() )
            break;
        auto point_track = (*tracklets_)[i];
        for( uint16_t j = 0; j < pose_range_; j++)
        {
            if(j >= point_track.size())
                break;
            if(isDynamicPoint(pts_from[i], (point_track)[j]));
            {
                cv::Point2f pt(pts_from[i].x, pts_from[i].y);
                curr_dyna_pts_->push_back(pt);
            }
        }
    }
    if(tracklets_->size()>=pose_range_)
        tracklets_->resize(pose_range_);

}
