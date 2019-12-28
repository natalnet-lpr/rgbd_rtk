/*
 * 
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

/*
 * Quad tree data structure to store image keypoints,
 */
class QuadTree
{

public:

    //Region than will be marked and segmented	
    cv::Rect boundary_;

    //Maximum number of keypoints in a node of quadtree
    int capacity_;

    //flag used to check if the current node is subdived
    bool divided_ = false;

    //flag used to check if the current node is empty or not
    bool allocated_ = false;

    //set of points of the node 
    std::vector<cv::Point2f> pts_;
  
    //these pointers represents four regions present in a 2-D plan
    QuadTree * topLeft_;
    QuadTree * topRight_;
    QuadTree * botLeft_;
    QuadTree * botRight_;

    //this variable helps in the task of marking the mask.
    //if some region have at least max_density points,
    //it is automatically marked as a black region, i.e. no point need to be detected here
    float max_density_;
	
    QuadTree(): capacity_(4), divided_(false), allocated_(false)
    {}
  
    QuadTree(const cv::Rect& boundary, const int& capacity, const float& max_density=0)
    {
        boundary_ = boundary;
        capacity_ = capacity;
        max_density_ = max_density;
        topLeft_ = new QuadTree();
        topRight_ = new QuadTree();
        botLeft_ = new QuadTree();
        botRight_ = new QuadTree();
    }

    //Subdivides the region if the region is not divided and the region is not out of capacity
    void subdivide();

    //Mark the mask, black regions are regions that will be dispensed in point detection
    void markMask(cv::Mat &mask, const std::vector<cv::Point2f>& pts, const bool& initialized);

    void drawTree(cv::Mat &img);

    //Inserts a point in region
    void insert(const cv::Point2f& new_point);
};
