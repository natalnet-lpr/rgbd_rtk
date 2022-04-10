#include "scored_fbmt.h"
#include <pcl/common/transforms.h>

using namespace std;
using namespace cv;
using namespace pcl;

ScoredFBMT::ScoredFBMT(MotionEstimatorRANSAC *motion_estimator, float dist_threshold, int score_threshold)
    : FeatureBasedMT(motion_estimator),
      dist_threshold_(dist_threshold),
      score_threshold_(score_threshold)
{
  max_score_ = score_threshold_ * 2;
  min_score_ = (score_threshold_ * 2) * -1;
}

template <class T, class V>
bool mapContains(const map<T, V> &m, const T &val)
{
  auto it = m.find(val);
  return it != m.end();
}

double euclidianDistance(const PointT &pt0, const PointT &pt1)
{
  // pt0
  float x0 = pt0.x;
  float y0 = pt0.y;
  float z0 = pt0.z;
  // pt1
  float x1 = pt1.x;
  float y1 = pt1.y;
  float z1 = pt1.z;

  return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
}

vector<int> ScoredFBMT::getStaticPointsIndexes(const vector<Point2f> &curr_pts)
{
  vector<Point3DCloudPairs> *point_cloud_pairs = &motion_estimator_->sparse_point_cloud_pairs_;
  vector<map<int, int>> *mapper_2d_3d = &motion_estimator_->mappers_2d_3d_;
  vector<Eigen::Matrix4f> *relative_poses = &motion_estimator_->relative_poses_;
  /*
  size_t total_of_pairs = point_cloud_pairs->size();

  // loop over each cloud pair
  for (size_t i = 1; i < total_of_pairs; i++)
  {
    PointCloud<PointT> *prev_cloud = &(*point_cloud_pairs)[i].first;
    PointCloud<PointT> *curr_cloud = &(*point_cloud_pairs)[i].second;
    Eigen::Matrix4f *relative_pose = &(*relative_poses)[i];

    PointCloud<PointT> curr_trans_cloud;

    pcl::transformPointCloud(*curr_cloud, curr_trans_cloud, *relative_pose);

    map<int, int> *prev_map = &(*mapper_2d_3d)[i - 1];
    map<int, int> *curr_map = &(*mapper_2d_3d)[i];

    // for each point inside of the point set
    //     get the correspondences in the cloud
    //    and calculate the distance between then;
    for (int pt_idx = 0; pt_idx < curr_pts.size(); pt_idx++)
    {
      if (mapContains(*curr_map, pt_idx) && mapContains(*prev_map, pt_idx))
      {
        int curr_cloud_idx = (*curr_map)[pt_idx];
        int prev_cloud_idx = (*prev_map)[pt_idx];

        PointT pt_from = (*prev_cloud)[prev_cloud_idx];
        PointT pt_to = curr_trans_cloud[curr_cloud_idx];

        float distance = euclidianDistance(pt_from, pt_to);

        if (distance < dist_threshold_)
        {
          point_scores_[pt_idx] += 1;
        }
        else
        {
          point_scores_[pt_idx] -= 1;
        }
      }
    }
  }
  
  vector<int> static_points;

  for (int pt_idx = 0; pt_idx < curr_pts.size(); pt_idx++)
  {
    if (mapContains(point_scores_, pt_idx))
    {
      if (point_scores_[pt_idx] > score_threshold_)
      {
        static_points.push_back(pt_idx);
      }
    }
  }*/

  vector<int> static_points;

  if (mapper_2d_3d->size() < 2)
  {
    return static_points;
  }

  Point3DCloudPairs *curr_cloud_pair = &(point_cloud_pairs->back());
  map<int, int> curr_map = *(mapper_2d_3d->rbegin());
  map<int, int> prev_map = *(mapper_2d_3d->rbegin() + 1);

  Eigen::Matrix4f &curr_relative_pose = relative_poses->back();

  point_scores_.clear();
  // for each point inside of the point set
  //     get the correspondences in the cloud
  //    and calculate the distance between then;
  for (int pt_idx = 0; pt_idx < curr_pts.size(); pt_idx++)
  {

    PointCloud<PointT> *prev_cloud = &(curr_cloud_pair->first);
    PointCloud<PointT> *curr_cloud = &(curr_cloud_pair->second);

    PointCloud<PointT> curr_trans_cloud;

    pcl::transformPointCloud(*curr_cloud, curr_trans_cloud, curr_relative_pose);

    if (mapContains(curr_map, pt_idx) && mapContains(prev_map, pt_idx))
    {
      int curr_cloud_idx = (curr_map)[pt_idx];
      int prev_cloud_idx = (prev_map)[pt_idx];

      const PointT &pt_from = (*prev_cloud)[prev_cloud_idx];
      const PointT &pt_to = curr_trans_cloud[curr_cloud_idx];

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

  return static_points;
}