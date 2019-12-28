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

#include <iostream>
#include <cmath>

#include <opencv2/imgproc/imgproc.hpp>

#include <quad_tree.h>

using namespace std;
using namespace cv;

void QuadTree::subdivide()
 {
    int x = boundary_.x;
    int y = boundary_.y;
    int h = boundary_.height;
    int w = boundary_.width;

    Rect TL_boundary(x, y, h/2, w/2);
    topLeft_ = new QuadTree(TL_boundary, capacity_, max_density_);
    
    Rect TR_boundary(x+w/2, y, h/2, w/2);
    topRight_ = new QuadTree(TR_boundary, capacity_, max_density_);

    Rect BR_boundary(x+w/2, y+h/2, h/2, w/2);
    botRight_ = new QuadTree(BR_boundary, capacity_, max_density_);

    Rect BL_boundary(x, y+h/2, h/2, w/2);
    botLeft_ = new QuadTree(BL_boundary, capacity_, max_density_);

    divided_ = true;
}

void QuadTree::markMask(Mat &mask, const vector<Point2f>& pts, const bool& initialized)
{

	int x = boundary_.x;
    int y = boundary_.y;
    int h = boundary_.height;
    int w = boundary_.width;	
	float area = mask.cols*mask.rows;
	float density_total = pts.size()/area;

	Rect TR_boundary(x, y, w, h);

	if(allocated_)
    {
		int n_points = 0;
		for(int i=0; i<pts.size(); i++)
        {
			if(boundary_.contains(pts[i]))
            {
				n_points++;
			}
		}
		float sub_density = n_points/float(h*w);

		if(2*sub_density >= density_total || 2*sub_density >= max_density_)
        {
			if(initialized)
				rectangle(mask, TR_boundary, Scalar(0), -1);
		}
		else
        {
			rectangle(mask, TR_boundary, Scalar(255), -1);
		}
		
		topRight_->markMask(mask, pts, true);
		topLeft_->markMask(mask, pts, true);
		botLeft_->markMask(mask, pts, true);
		botRight_->markMask(mask, pts, true);
	}
}

void QuadTree::drawTree(Mat& img){
    
    int x = boundary_.x;
    int y = boundary_.y;
    int h = boundary_.height;
    int w = boundary_.width;    
    
    Rect TR_boundary(x, y, w, h);

    if(allocated_)
    {
        rectangle(img, TR_boundary, Scalar(255,255,0), 1);

        topRight_->drawTree(img);
        topLeft_->drawTree(img);
        botLeft_->drawTree(img);
        botRight_->drawTree(img);
    }
}

void QuadTree::insert(const Point2f& new_point)
{
    allocated_ = true;
        
    if(!boundary_.contains(new_point))
        return;

    if(pts_.size() < capacity_)
    {
        pts_.push_back(new_point);
    }
    else
    {
        if(!divided_)
        {
            subdivide();
        }

        topRight_->insert(new_point);
        topLeft_->insert(new_point);
        botRight_->insert(new_point);
        botLeft_->insert(new_point);
    }
}


