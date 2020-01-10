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

#include <vector>
#include <opencv2/core/core.hpp>

#include <iostream>

/*
 * Quad tree data structure to store image keypoints
 */
class QuadTree
{

private:

    //Recursive function that actually generates the mask:
    //uses the points from all previous upper levels nodes
    void _generateMask(cv::Mat &mask, const std::vector<cv::Point2f>& upper_levels_points);

public:

    //Rectangular region kept in a node of the quadtree
    cv::Rect boundary_;

    //Maximum number of keypoints in a node of the quadtree
    size_t capacity_;

    //Flag used to check if the current node is subdived
    bool divided_;

    //Flag used to check if the current node is empty or not
    bool allocated_;

    //Points present in a node of the quadtree
    std::vector<cv::Point2f> pts_;
  
    //Quadtree subdivisions
    QuadTree * topLeft_;
    QuadTree * topRight_;
    QuadTree * botLeft_;
    QuadTree * botRight_;

    //This variable helps in the task of marking the mask.
    //If some region have at least max_density points,
    //it is automatically marked as a black region,
    //i.e. no point need to be detected here
    float max_density_;
	
    QuadTree():
    capacity_(4), divided_(false), allocated_(false), max_density_(0.0)
    {
        topLeft_ = NULL;
        topRight_ = NULL;
        botLeft_ = NULL;
        botRight_ = NULL;
        printf("[QuadTree::QuadTree] DEBUG: building node with undefined boundary and no subtrees\n");
    }
  
    QuadTree(const cv::Rect& boundary, const int& capacity, const float& max_density=0):
    divided_(false), allocated_(false)
    {
        boundary_ = boundary;
        capacity_ = capacity;
        max_density_ = max_density;

        //Initialize 4 child nodes (they are actually used to traverse the tree)
        topLeft_ = new QuadTree();
        topRight_ = new QuadTree();
        botLeft_ = new QuadTree();
        botRight_ = new QuadTree();

        int x = boundary_.x;
        int y = boundary_.y;
        int h = boundary_.height;
        int w = boundary_.width;

        printf("[QuadTree::QuadTree] DEBUG: building node (%i,%i) <-> (%i,%i)\n", x, y, x+h, y+w);
    }

    //Default destructor
    ~QuadTree()
    {
        if(topLeft_)
        {
            printf("[QuadTree::QuadTree] DEBUG: deleting topLeft_ tree\n");
            delete topLeft_;
        }
        if(topRight_)
        {
            printf("[QuadTree::QuadTree] DEBUG: deleting topRight_ tree\n");
            delete topRight_;
        }
        if(botLeft_)
        {
            printf("[QuadTree::QuadTree] DEBUG: deleting botLeft_ tree\n");
            delete botLeft_;
        }
        if(botRight_)
        {
            printf("[QuadTree::QuadTree] DEBUG: deleting botRight_ tree\n");
            delete botRight_;
        }
    }

    //Inserts a point in region
    void insert(const cv::Point2f& new_point);

    //Subdivides the region if the region is not divided yet
    //(create subtrees by equally dividing the rect. region into 4 equal regions)
    void subdivide();

    //Generate a mask: b/w image in which white pixels denote regions in which keypoints
    //should be extracted
    void generateMask(cv::Mat &mask);

    void drawTree(cv::Mat &img);

    //Sets the node boundary (rectangular region)
    void setBoundary(const cv::Rect& boundary);
};
