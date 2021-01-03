/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or
 *     promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Authors:
 *
 *  Bruno Silva
 *  Rodrigo Sarmento Xavier
 */

#ifndef INCLUDE_COMMON_TYPES_H_
#define INCLUDE_COMMON_TYPES_H_

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

typedef pcl::PointXYZRGB PointT;

/*
 * Tracklet class is responsible to hold all
 * relevant information related to a tracked keypoint
 * (tracklet)
 */
struct Tracklet
{
    /* Index of the frame in which the keypoint first appeared */
    int start_;

    /* History of image projections of the keypoints along consecutive
     * frames.
     * m_pts2D[i] holds the position of the point at index start_ + i,
     * that is,
     * m_pts2D[0] holds the position of the point at index start_,
     * m_pts2D[1] holds the position of the point at index start_ + 1
     * and so on.
     */
    std::vector<cv::Point2f> pts2D_;

    /*
     * At position i, the vector stores the index of the 3D point
     * at the 3D point cloud start_ + i.
     */
    std::vector<int> cloud_indices_;

    /*
     * At position i, the vector stores the index of the keypoint
     * corresponding to the 2D point stored in pts2D_.
     */
    std::vector<int> keypoint_indices_;

    /**
     * Initializes a tracklet with start_ = 0
     */
    Tracklet() : start_(0) {}

    /**
     * Initializes a tracklet with start_ = idx
     */
    Tracklet(const int idx) { start_ = idx; }
};

/*
 * Set of intrinsic parameters for perspective cameras.
 * Values taken from: http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
 */
struct Intrinsics
{
    /* Focal distance in the horizontal direction */
    double fx_;

    /* Focal distance in the vertical direction */
    double fy_;

    /* Projection center coordinate in the horizontal direction */
    double cx_;

    /* Projection center coordinate in the vertical direction */
    double cy_;

    /* Scale factor for the Kinect depth */
    double scale_;

    /* Distance between cameras (used in stereo) */
    double baseline_;

    /* First radial distortion coefficient (d0) */
    double k1_;

    /* Second radial distortion coefficient (d1) */
    double k2_;

    /* First tangential distortion coefficient (d2) */
    double p1_;

    /* Second tangential distortion coefficient (d3) */
    double p2_;

    /* Third radial distortion coefficient (d4) */
    double k3_;

    /**
     * Default constructor: zero for all parameters
     */
    Intrinsics()
        : fx_(0.0), fy_(0.0), cx_(0.0), cy_(0.0), scale_(0.0), baseline_(0.0), k1_(0.0), k2_(0.0), p1_(0.0), p2_(0.0),
          k3_(0.0)
    {
    }

    /**
     * Constructor for the Freiburg1, Freiburg2, Freiburg3 TUM datasets
     * or for the Kinect factor defaults.
     * @param set with parameter will be used, 1 2 and 3
     * for Freiburg1 Freiburg2 and Freiburg3
     */
    Intrinsics(const int& set)
    {
        switch (set)
        {
            case 1:
                fx_ = 517.3;
                fy_ = 516.5;
                cx_ = 318.6;
                cy_ = 255.3;
                scale_ = 5000;
                baseline_ = 0.0; // not used by Kinect
                // k1_ = 0.2624;
                // k2_ = -0.9531;
                // p1_ = -0.0054;
                // p2_ = 0.0026;
                // k3_ = 1.1633;
                k1_ = 0.0;
                k2_ = 0.0;
                p1_ = 0.0;
                p2_ = 0.0;
                k3_ = 0.0;
                break;
            case 2:
                fx_ = 520.9;
                fy_ = 521.0;
                cx_ = 325.1;
                cy_ = 249.7;
                scale_ = 5000;
                baseline_ = 0.0; // not used by Kinect
                // k1_ = 0.2312;
                // k2_ = -0.7849;
                // p1_ = -0.0033;
                // p2_ = -0.0001;
                // k3_ = 0.9172;
                k1_ = 0.0;
                k2_ = 0.0;
                p1_ = 0.0;
                p2_ = 0.0;
                k3_ = 0.0;
                break;
            case 3:
                fx_ = 535.4;
                fy_ = 539.2;
                cx_ = 320.1;
                cy_ = 247.6;
                scale_ = 5000;
                baseline_ = 0.0; // not used by Kinect
                k1_ = 0.0;
                k2_ = 0.0;
                p1_ = 0.0;
                p2_ = 0.0;
                k3_ = 0.0;
                break;
            default:
                fx_ = 525.0;
                fy_ = 525.0;
                cx_ = 319.5;
                cy_ = 239.5;
                scale_ = 5000;
                baseline_ = 0.0; // not used by Kinect
                k1_ = 0.0;
                k2_ = 0.0;
                p1_ = 0.0;
                p2_ = 0.0;
                k3_ = 0.0;
                break;
        }
    }
    /**
     * Constructor for a perspective camera with given focal lengths
     * and center of projection (scale is 1.0).
     * @param fx @param fy @param cx @param cy @param baseline
     */
    Intrinsics(const double& fx, const double& fy, const double& cx, const double& cy, const double& baseline)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), scale_(1.0), baseline_(baseline), k1_(0.0), k2_(0.0), p1_(0.0), p2_(0.0),
          k3_(0.0)
    {
    }
};

/**
 * Keyframe contains all the relevant information for SLAM with pose graph optimization:
 * keypoints, RGB image, camera pose, point cloud, etc. captured in a given time step.
 */
struct Keyframe
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Index of the keyframe
    int idx_;

    // Pose of the keyframe (in the global reference frame)
    Eigen::Affine3f pose_;

    // RGB image of the keyframe
    cv::Mat img_;

    // Keyframe name
    std::string name_;

    // RGB-D point cloud of the keyframe (in local reference frame)
    pcl::PointCloud<PointT>::Ptr local_cloud_;

    // Keypoints detected in the keyframe
    std::vector<cv::Point2f> keypoints_;

    Keyframe() : idx_(0), pose_(Eigen::Affine3f::Identity()), local_cloud_(new pcl::PointCloud<PointT>) {}
};

/**
 * Edge saves two vertex poses that makes an edge
 * it also saves the id of those poses and the name of the edge
 */
struct Edge
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Id of vertex "from"
    int id_from_;

    // Id of vertex "to"
    int id_to_;

    // Position of vertex "from"
    Eigen::Vector3d pose_from_;

    // Position of vertex "to
    Eigen::Vector3d pose_to_;

    // Edge name
    std::string name_;

    Edge() : id_from_(-1), id_to_(-1) {}

    /**
     * Edge Constructor
     * @param from_id int id of vertex from
     * @param to_id int id of vertex to
     * @param from pose of vertex from
     * @param to pose of vertex to
     * @param name Edge name(optional)
     */
    Edge(
        const int& from_id,
        const int& to_id,
        const Eigen::Vector3d& from,
        const Eigen::Vector3d& to,
        const std::string& name = "")
    {
        id_from_ = from_id;
        id_to_ = to_id;
        pose_from_ = from;
        pose_to_ = to;
        name_ = name;
    }
};
#endif /* COMMON_TYPES_H_ */
