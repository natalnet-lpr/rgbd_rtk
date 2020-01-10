/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2019, Natalnet Laboratory for Perceptual Robotics
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
 *  Luiz Felipe Maciel Correia (y9luiz@hotmail.com)
 *  Bruno Silva (brunomfs@gmail.com)
 */

#include <iostream>
#include <cmath>
#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>

#include <event_logger.h>
#include <quad_tree.h>

using namespace std;
using namespace cv;

EventLogger& logger = EventLogger::getInstance();

void QuadTree::_generateMask(cv::Mat &mask, const std::vector<cv::Point2f>& upper_levels_points)
{
    if(allocated_)
    {
        //Compute the point density (points/pixel) of the quadtree node
        int x = boundary_.x;
        int y = boundary_.y;
        int h = boundary_.height;
        int w = boundary_.width;
        float node_area = h*w;
        float node_density = 0.0;

        logger.print(pcl::console::L_DEBUG, "[QuadTree::generateMask] DEBUG: node (%i,%i) <-> (%i,%i)\n", x, y, x+h, y+w);

        //To compute the node density, check which points in all parent nodes
        //are within the node boundary and add it to a vector
        logger.print(pcl::console::L_DEBUG, "[QuadTree::generateMask] DEBUG: checking nodes from upper levels...\n");
        vector<Point2f> points_within_node;
        for(size_t i = 0; i < upper_levels_points.size(); i++)
        {
            logger.print(pcl::console::L_DEBUG, "[QuadTree::generateMask] DEBUG: point (%f,%f)\n", upper_levels_points[i].x, upper_levels_points[i].y);
            if(boundary_.contains(upper_levels_points[i]))
            {
                logger.print(pcl::console::L_DEBUG, "[QuadTree::generateMask] DEBUG: inside\n");
                points_within_node.push_back(upper_levels_points[i]);
            }
        }
        //The node points are obviously within the node boundary
        points_within_node.reserve(points_within_node.size() + pts_.size());
        points_within_node.insert(points_within_node.end(),
                                 pts_.begin(),
                                 pts_.end());
        node_density = float(points_within_node.size())/node_area;
        logger.print(pcl::console::L_DEBUG, "[QuadTree::generateMask] DEBUG: points within node: %lu\n", points_within_node.size());
        logger.print(pcl::console::L_DEBUG, "[QuadTree::generateMask] DEBUG: node area: %f\n", node_area);
        logger.print(pcl::console::L_DEBUG, "[QuadTree::generateMask] DEBUG: node_density: %f\n", node_density);

        //If the node density is larger than the max density,
        //mark the region with black
        //(no points should be detected in the region)
        if(node_density >= max_density_)
        {
            logger.print(pcl::console::L_DEBUG, "[QuadTree::generateMask] DEBUG: node region marked as black\n");
            rectangle(mask, boundary_, Scalar(0), -1);
        }
        else
        {
            logger.print(pcl::console::L_DEBUG, "[QuadTree::generateMask] DEBUG: node region marked as white\n");
            rectangle(mask, boundary_, Scalar(255), -1);
        }

        //Recursively pass all points within the node to the child nodes
        topLeft_->_generateMask(mask, points_within_node);
        topRight_->_generateMask(mask, points_within_node);
        botLeft_->_generateMask(mask, points_within_node);
        botRight_->_generateMask(mask, points_within_node);
    }
}

void QuadTree::insert(const Point2f& new_point)
{
    int x = boundary_.x;
    int y = boundary_.y;
    int h = boundary_.height;
    int w = boundary_.width;

    logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: point: (%f,%f)\n", new_point.x, new_point.y);
    logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: node region: (%i,%i) <-> (%i,%i)\n", x, y, x+h, y+w);

    allocated_ = true;

    //The supplied point is out of the quadtree node
    if(!boundary_.contains(new_point))
    {
        logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: point is out of region\n");
        return;
    }

    //The supplied point is within the quadtree node and
    //there is room to store it
    if(pts_.size() < capacity_)
    {
        pts_.push_back(new_point);
        logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: point succesfully inserted (%i/%i)\n", pts_.size(), capacity_);
    }
    //There is no room to store the point: subdivide the tree
    //and insert it in one of the subtrees of the tree
    else
    {
        logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: node capacity reached -> inserting in subtree\n");
        if(!divided_)
        {
            logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: subdividing trees...\n");
            subdivide();
        }

        topLeft_->insert(new_point);
        topRight_->insert(new_point);
        botLeft_->insert(new_point);
        botRight_->insert(new_point);
    }
}

void QuadTree::subdivide()
 {
    int x = boundary_.x;
    int y = boundary_.y;
    int h = boundary_.height;
    int w = boundary_.width;

    //Exchange the child nodes for nodes that will store points 
    Rect TL_boundary(x, y, h/2, w/2);
    logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: creating top left tree (%i,%i) <-> (%i,%i)\n", x, y, x+h/2, y+w/2);
    delete topLeft_;
    topLeft_ = new QuadTree(TL_boundary, capacity_, max_density_);
    
    Rect TR_boundary(x+w/2, y, h/2, w/2);
    logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: creating top right tree (%i,%i) <-> (%i,%i)\n", x+w/2, y, x+w/2+h/2, y+w/2);
    delete topRight_;
    topRight_ = new QuadTree(TR_boundary, capacity_, max_density_);

    Rect BL_boundary(x, y+h/2, h/2, w/2);
    logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: creating bottom left tree (%i,%i) <-> (%i,%i)\n", x, y+h/2, x+h/2, y+h/2+w/2);
    delete botLeft_;
    botLeft_ = new QuadTree(BL_boundary, capacity_, max_density_);

    Rect BR_boundary(x+w/2, y+h/2, h/2, w/2);
    logger.print(pcl::console::L_DEBUG, "[QuadTree::insert] DEBUG: creating bottom right tree (%i,%i) <-> (%i,%i)\n", x+w/2, y+h/2, x+w/2+h/2, y+h/2+w/2);
    delete botRight_;
    botRight_ = new QuadTree(BR_boundary, capacity_, max_density_);

    divided_ = true;
}

void QuadTree::generateMask(cv::Mat &mask)
{
	vector<Point2f> vector_without_pts;

    _generateMask(mask, vector_without_pts);
}

void QuadTree::drawTree(Mat& img)
{    
    if(allocated_)
    {
        rectangle(img, boundary_, Scalar(255,255,0), 1);

        topLeft_->drawTree(img);
        topRight_->drawTree(img);
        botLeft_->drawTree(img);
        botRight_->drawTree(img);
    }
}

void QuadTree::setBoundary(const cv::Rect& boundary)
{
    boundary_ = boundary;
}

