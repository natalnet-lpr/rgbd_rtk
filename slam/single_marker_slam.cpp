/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2021, Natalnet Laboratory for Perceptual Robotics
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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Authors:
 *
 *  Bruno Silva
 *  Rodrigo Xavier
 */

#include <utility>
#include <map>

#include <event_logger.h>
#include <geometry.h>
#include <single_marker_slam.h>

using namespace std;
using namespace cv;

void SingleMarkerSLAM::_addKeyframe()
{
    const size_t next_kf_id = keyframes_.size();

    Keyframe kf = vo_.createKeyframe(next_kf_id);
    
    MLOG_DEBUG(EventLogger::M_SLAM,
               "@SingleMarkerSLAM::_addKeyFrame: "
               "adding keyframe of id %lu\n",
               next_kf_id);

    keyframes_.insert(pair<size_t, Keyframe>(kf.idx_, kf));
}

void SingleMarkerSLAM::_updateKeyframes()
{
    for(std::map<size_t, Keyframe>::iterator it = keyframes_.begin();
         it != keyframes_.end(); it++)
    {
        MLOG_DEBUG(EventLogger::M_SLAM,
                   "@SingleMarkerSLAM::_updateKeyframes: "
                   "updating keyframe of id %lu\n",
                   it->first);
        MLOG_DEBUG(EventLogger::M_SLAM,
                   "@SingleMarkerSLAM::_updateKeyframes: "
                   "old pos.: %f %f %f\n",
                   keyframes_[it->first].pose_(0,3),
                   keyframes_[it->first].pose_(1,3),
                   keyframes_[it->first].pose_(2,3));

        keyframes_[it->first].pose_ = slam_.getVertexPose(it->first);

        MLOG_DEBUG(EventLogger::M_SLAM,
                   "@SingleMarkerSLAM::_updateKeyframes: "
                   "new pos.: %f %f %f\n",
                   keyframes_[it->first].pose_(0,3),
                   keyframes_[it->first].pose_(1,3),
                   keyframes_[it->first].pose_(2,3));
    }
}

Eigen::Affine3f SingleMarkerSLAM::_lastVertexPose()
{
    const int last_vertex_id = keyframes_.size() - 1;
    MLOG_DEBUG(EventLogger::M_SLAM,
               "@SingleMarkerSLAM::_lastVertexPose: "
               "returning pose of id %lu\n", last_vertex_id);
    
    return slam_.getVertexPose(last_vertex_id);
}

void SingleMarkerSLAM::_correctVisualOdometry()
{
    Eigen::Affine3f slam_pose = _lastVertexPose();
    vo_correction_ = relativeTransform(vo_.pose_, slam_pose);
    vo_.pose_ = slam_pose; //same as right multiplying vo_.pose_ by vo_correction_
}

// SingleMarkerSLAM::SingleMarkerSLAM():
//     max_marker_dist_(4.0),
//     min_kf_dist_(0.05),
//     opt_iterations_(10),
//     opt_loop_closures_(10),
//     vo_(Intrinsics(0))
// {
//     initialized_ = false;
//     marker_found_= false;
//     curr_loop_closures_ = 0;

//     last_odom_kf_pose_ = Eigen::Affine3f::Identity();
//     last_lc_kf_pose_  = Eigen::Affine3f::Identity();
//     first_pose_from_AR_ = Eigen::Affine3f::Identity();
//     last_pose_from_AR_ = Eigen::Affine3f::Identity();
//     vo_correction_ = Eigen::Affine3f::Identity();
// }

SingleMarkerSLAM::SingleMarkerSLAM(const FeatureTracker::Parameters &tracker_param,
                                   const MarkerFinder::Parameters &mf_param,
                                   const SingleMarkerSLAM::Parameters &slam_param):
    max_marker_dist_(mf_param.max_marker_dist_),
    min_kf_dist_(slam_param.min_dist_bw_keyframes_),
    opt_iterations_(slam_param.opt_iterations_),
    opt_loop_closures_(slam_param.opt_loop_closures_),
    vo_(Intrinsics(0), tracker_param, 0.008), //TODO Remove this (hardcoded)!!!
    marker_finder_(mf_param.calib_file_, mf_param.marker_size_, mf_param.aruco_dict_)
{
    initialized_ = false;
    marker_found_= false;
    curr_loop_closures_ = 0;

    last_odom_kf_pose_ = Eigen::Affine3f::Identity();
    last_lc_kf_pose_  = Eigen::Affine3f::Identity();
    first_pose_from_AR_ = Eigen::Affine3f::Identity();
    last_pose_from_AR_ = Eigen::Affine3f::Identity();
    vo_correction_ = Eigen::Affine3f::Identity();
}

void SingleMarkerSLAM::addVertexAndEdge(const Eigen::Affine3f &pose)
{
    const size_t last_vertex_id = keyframes_.size() - 1;

    // Add the first node and fix it.
    if(last_vertex_id == 0)
    {
        slam_.addVertex(pose, last_vertex_id, true);
    }
    // When adding any nodes other than the first, add an edge connecting
    // to the previous one
    else
    {
        slam_.addVertex(pose, last_vertex_id);
        slam_.addOdometryEdge(last_vertex_id); //from last_vertex_id-1 to last_vertex_id
    }
}

void SingleMarkerSLAM::addLoopClosingEdge(const Eigen::Affine3f &transform)
{
    const size_t last_vertex_id = keyframes_.size() - 1;

    slam_.addEdge(0, last_vertex_id, affineToIsometry(transform));
}

bool SingleMarkerSLAM::processImage(const Mat &rgb, const Mat &depth,
                                    const int &marker_id)
{
    bool is_kf, kf_created = false;

    // Compute camera pose with visual odometry
    is_kf = vo_.computeCameraPose(rgb, depth);

    // Detect aruco markers in the image
    marker_finder_.detectMarkersPoses(rgb, Eigen::Affine3f::Identity(), max_marker_dist_);

    // Check if the ref. marker was found
    marker_found_ = marker_finder_.isMarkerFound(marker_id);
    if(marker_found_)
    {
        Eigen::Affine3f pose_from_AR = marker_finder_.markerPose(marker_id).inverse();

        // If we found the mark for the first time, we will consider the marker as the origin
        // of the system and set the first camera pose as the marker pose.
        if(!initialized_)
        {
            MLOG_DEBUG(EventLogger::M_SLAM, "@SingleMarkerSLAM: setting coord. system origin\n");

            // Save poses
            last_pose_from_AR_ = pose_from_AR;
            vo_.pose_ = last_pose_from_AR_; // Set the first odometry pose as the first marker pose
            first_pose_from_AR_ = last_pose_from_AR_;
            last_lc_kf_pose_ = last_pose_from_AR_;
            last_odom_kf_pose_ = last_pose_from_AR_;

            // Update state
            initialized_ = true;
            kf_created = true;
            _addKeyframe(); // Create a keyframe (marker found for the first time)
            curr_loop_closures_++;

            // Update SLAM graph
            addVertexAndEdge(vo_.pose_);
        }
        // If the marker has been found before (system is initialized),
        // we check if the current pose is eligible for a new keyframe
        else if(distanceBetween(vo_.pose_, last_lc_kf_pose_) > min_kf_dist_ &&
                zAxisAngleBetween(pose_from_AR.inverse(),
                                  first_pose_from_AR_.inverse()) <= 10.0)
        {
            MLOG_DEBUG(EventLogger::M_SLAM, "@SingleMarkerSLAM: loop closure keyframe\n");

            // Save poses
            last_lc_kf_pose_ = vo_.pose_;

            // Update state
            kf_created = true;
            _addKeyframe(); // Create a keyframe (marker found and distance/angle ok)
            curr_loop_closures_++;

            // Update SLAM graph
            last_pose_from_AR_ = pose_from_AR; // Note: updated only if passed on the tests
            Eigen::Affine3f transf = relativeTransform(first_pose_from_AR_, last_pose_from_AR_);
            addVertexAndEdge(vo_.pose_);
            addLoopClosingEdge(transf);
        }
    }
    // Marker not found: check if there is keyframe from odometry
    else
    {
        MLOG_DEBUG(EventLogger::M_SLAM, "@SingleMarkerSLAM: marker not found\n");

        if(is_kf && distanceBetween(vo_.pose_, last_odom_kf_pose_) > min_kf_dist_)
        {
            MLOG_DEBUG(EventLogger::M_SLAM, "@SingleMarkerSLAM: odometry keyframe\n");

            // Save poses
            last_odom_kf_pose_ = vo_.pose_;

            // Update state
            kf_created = true;
            _addKeyframe(); // Create a keyframe (marker not found but distance bw. last kf. ok)

            // Update SLAM graph
            addVertexAndEdge(vo_.pose_);
        }
    }

    // Run optimization if criteria is met and update poses of the optimized keyframes/positions of the edges
    if(curr_loop_closures_ == opt_loop_closures_)
    {
        MLOG_INFO(EventLogger::M_SLAM,
                  "@SingleMarkerSLAM: pose graph before:\n");
        slam_.printGraph();
        MLOG_INFO(EventLogger::M_SLAM,
                  "@SingleMarkerSLAM: optimizing graph "
                  "for %lu iterations\n", opt_iterations_);
        slam_.optimizeGraph(opt_iterations_);
        MLOG_INFO(EventLogger::M_SLAM,
                  "@SingleMarkerSLAM: pose graph after:\n");
        slam_.printGraph();
        
        _updateKeyframes();
        _correctVisualOdometry();
        
        curr_loop_closures_ = 0;
    }

    return kf_created;
}

bool SingleMarkerSLAM::isMarkerFound()
{
    return marker_found_;   
}

const std::map<size_t, Keyframe>& SingleMarkerSLAM::keyframes()
{
    return keyframes_;
}

const Keyframe& SingleMarkerSLAM::lastKeyframe()
{
    if(keyframes_.size() == 0)
    {
        MLOG_ERROR(EventLogger::M_SLAM,
                  "@SingleMarkerSLAM::lastKeyframe: "
                  "no keyframes stored.\n");
        exit(0);
    }
    else
    {
        const size_t last_kf_id = keyframes_.size() - 1;
        MLOG_DEBUG(EventLogger::M_SLAM,
                   "@SingleMarkerSLAM::lastKeyframe: "
                   "returning keyframe of id %lu\n", last_kf_id);
        return keyframes_[last_kf_id];
    }
}

Eigen::Affine3f SingleMarkerSLAM::visualOdometryPose()
{
    return vo_.pose_;
}

Eigen::Affine3f SingleMarkerSLAM::markerPose()
{
    return last_pose_from_AR_;
}

pcl::PointCloud<PointT>::Ptr SingleMarkerSLAM::visualOdometryPointCloud()
{
    return vo_.curr_dense_cloud_;
}