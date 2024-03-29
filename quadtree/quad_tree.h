/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2019, Natalnet Laboratory for Perceptual Robotics
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
 *  Luiz Felipe Maciel Correia (y9luiz@hotmail.com)
 *  Bruno Silva (brunomfs@gmail.com)
 */

#include <opencv2/core/core.hpp>
#include <vector>

#include <memory>

/*
 * Quad tree data structure to store image keypoints
 */
class QuadTree
{

public:
    // Rectangular region kept in a node of the quadtree
    cv::Rect boundary_;

    // Maximum number of keypoints in a node of the quadtree
    std::size_t capacity_;

    // Flag used to check if the current node is subdived
    bool divided_;

    // Flag used to check if the current node is empty or not
    bool allocated_;

    // Points present in a node of the quadtree
    std::vector<cv::Point2f> pts_;

    // Quadtree subdivisions
    std::unique_ptr<QuadTree> topLeft_;
    std::unique_ptr<QuadTree> topRight_;
    std::unique_ptr<QuadTree> botLeft_;
    std::unique_ptr<QuadTree> botRight_;

    // This variable helps in the task of marking the mask.
    // If some region have at least max_density points,
    // it is automatically marked as a black region,
    // i.e. no point need to be detected here
    float max_density_;

    /**
     * Default constructor
     */
    QuadTree();

    /**
     * Constructor
     * @param boundary
     * @param capacity
     * @param max_density default is 0
     */
    QuadTree(const cv::Rect &boundary, const int &capacity, const float &max_density = 0);

    /**
     * Inserts a point in region
     * @param new_point to be insert
     */
    void insert(const cv::Point2f &new_point);

    /**
     * Subdivides the region if the region is not divided yet
     * (create subtrees by equally dividing the rect. region into 4 equal regions)
     */
    void subdivide();

    /**
     * Mark the mask, black regions are regions that will not be used in point detection
     * @param mask @param pts @param initialized
     */
    void markMask(cv::Mat &mask, const std::vector<cv::Point2f> &pts, const bool &initialized);

    /**
     * @param img rgb image
     */
    void drawTree(cv::Mat &img);
};
