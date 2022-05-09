/*
*  Software License Agreement (BSD License)
*
*  Copyright (c) 2016-2022, Natalnet Laboratory for Perceptual Robotics
*  All rights reserved.
*  Redistribution and use in source and binary forms, with or without modification, are permitted provided
*  that the following conditions are met:
*
*  1. Redistributions of source code must retain the above copyright notice, this list of conditions and
*     the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
*     the following disclaimer in the documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
*  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*  Authors:
*
*  Luiz Correia
*/

#include "scored_fbmt.h"

#include <pcl/common/transforms.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;

ScoredFBMT::ScoredFBMT(MotionEstimatorRANSAC *motion_estimator, const Intrinsics &intrisics, float dist_threshold, int score_threshold)
    : FeatureBasedMT(motion_estimator),
      intrisics_(intrisics),
      dist_threshold_(dist_threshold),
      score_threshold_(score_threshold)
{
  max_score_ = score_threshold_ * 2;
  min_score_ = (score_threshold_ * 2) * -1;
}

template <class T, class V>
bool mapContains(const map<T, V> &m, const T &key)
{
  auto it = m.find(key);
  bool containsKey = (it != m.end());
  return containsKey;
}

double euclidianDistance(const Point2f &pt0, const Point2f &pt1)
{
  // pt0
  float x0 = pt0.x;
  float y0 = pt0.y;
  // pt1
  float x1 = pt1.x;
  float y1 = pt1.y;

  return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

Point2f convert3dTo2d(const PointT &pt3d, const Intrinsics &intrisics)
{
  Point2f pt2d;

  double fx = intrisics.fx_;
  double fy = intrisics.fy_;
  double cx = intrisics.cx_;
  double cy = intrisics.cy_;

  double z = pt3d.z;
  double x = pt3d.x;
  double y = pt3d.y;

  pt2d.x = (fx * x) / z + cx;
  pt2d.y = (fy * y) / z + cy;

  return pt2d;
}

vector<int> ScoredFBMT::estimateStaticPointsIndexes(const vector<Point2f> &curr_pts)
{
  vector<Point3DCloudPairs> *point_cloud_pairs = &motion_estimator_->sparse_point_cloud_pairs_;
  vector<map<int, int>> *mapper_2d_3d = &motion_estimator_->mappers_2d_3d_;
  vector<Matrix4f> *relative_poses = &motion_estimator_->relative_poses_;

  // Should have at least 2 poses inside of the motion estimator
  if (relative_poses->size() < 2)
  {

    MLOG_DEBUG(EventLogger::M_SEGMENTATION, "@ScoredFBMT::estimateStaticPointsIndexes:"
                                            "MotionEstimation should "
                                            "have at least 2 poses stored,"
                                            "current amout of poses: %ld",
               relative_poses->size());

    return {};
  }

  Point3DCloudPairs *curr_cloud_pair = &(point_cloud_pairs->back());
  map<int, int> curr_map = *(mapper_2d_3d->rbegin());
  map<int, int> prev_map = *(mapper_2d_3d->rbegin() + 1);

  Matrix4f &curr_relative_pose = relative_poses->back();

  point_scores_.clear();

  // For each point inside of the point set
  //     get the correspondences in the cloud
  //    and calculate the distance between then;
  for (int pt_idx = 0; pt_idx < curr_pts.size(); pt_idx++)
  {

    PointCloud<PointT> *prev_cloud = &(curr_cloud_pair->first);
    PointCloud<PointT> *curr_cloud = &(curr_cloud_pair->second);

    PointCloud<PointT> curr_trans_cloud;

    transformPointCloud(*curr_cloud, curr_trans_cloud, curr_relative_pose);

    if (mapContains(curr_map, pt_idx) && mapContains(prev_map, pt_idx))
    {
      int curr_cloud_idx = (curr_map)[pt_idx];
      int prev_cloud_idx = (prev_map)[pt_idx];

      const Point2f pt_from = convert3dTo2d((*prev_cloud)[prev_cloud_idx], intrisics_);
      const Point2f pt_to = convert3dTo2d(curr_trans_cloud[curr_cloud_idx], intrisics_);

      float distance = euclidianDistance(pt_from, pt_to);

      if (distance < dist_threshold_ && point_scores_[pt_idx] < max_score_)
      {
        point_scores_[pt_idx] += 1;
      }
      else if (point_scores_[pt_idx] > min_score_)
      {
        point_scores_[pt_idx] -= 1;
      }
    }
  }

  return getStaticPointIndexes(curr_pts);
}

vector<int> ScoredFBMT::getStaticPointIndexes(const vector<Point2f> &curr_pts)
{
  vector<int> static_points;

  for (int pt_idx = 0; pt_idx < curr_pts.size(); pt_idx++)
  {
    if (mapContains(point_scores_, pt_idx))
    {
      if (point_scores_[pt_idx] > 0)
      {
        static_points.push_back(pt_idx);
      }
    }
  }

  MLOG_DEBUG(EventLogger::M_SEGMENTATION, "@ScoredFBMT::getStaticPointIndexes:"
                                          "total of static points %dl/%dl",
             static_points.size(),
             curr_pts.size());

  return static_points;
}