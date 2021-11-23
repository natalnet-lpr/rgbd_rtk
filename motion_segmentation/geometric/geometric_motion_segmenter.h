#pragma once
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <utility>

#include "../../common/common_types.h"
#include "../motion_segmenter.h"

class GeometricMotionSegmenter : public MotionSegmenter{
    public:
        GeometricMotionSegmenter();
        virtual void segment(const cv::Mat & in_img, cv::Mat & out_img);
        void set3DTracklets(std::shared_ptr<std::vector<std::vector<PointT>>> tracklets);
        // assuming that these points are already projected in it respectives poses
        bool isDynamicPoint(const PointT & pt_from, const PointT & pt_to) const;
        void calculateDynamicPoints(const std::vector<PointT> & pts_from);
        inline std::shared_ptr<std::vector<cv::Point2f>> getDynaPoints(){return curr_dyna_pts_;};
    protected:

        // This variable determinate how many poses will be
        // used to determinate if an object is dynamic or not
        uint16_t pose_range_;

        // store a reference to the tracklets vector
        std::shared_ptr<std::vector<std::vector<PointT>>> tracklets_;

        //  current dynamic points
        std::shared_ptr<std::vector<cv::Point2f>> curr_dyna_pts_;
};