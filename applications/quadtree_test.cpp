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

#include <cstdio>
#include <cstdlib>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <event_logger.h>
#include <quad_tree.h>

using namespace std;
using namespace cv;

/**
 * This program shows the use of the QuadTree
 * class to store 2D points.
 */
int main(int argc, char **argv)
{
	EventLogger& logger = EventLogger::getInstance();
	logger.setVerbosityLevel(pcl::console::L_DEBUG);

	Mat mask = Mat::zeros(400, 400, CV_8U);
	mask.setTo(255);

	//Build quadtree with given boundary and capacity
	Rect tree_boundary(0, 0, 400, 400);
	QuadTree *tree = new QuadTree(tree_boundary, 5, 1.0);

	//Insert a few points into the quadtree:
	vector<Point2f> points;
	points.push_back(cv::Point2f(0, 0));
	points.push_back(cv::Point2f(5, 5));
	points.push_back(cv::Point2f(10, 10));
	points.push_back(cv::Point2f(20, 20));
	points.push_back(cv::Point2f(400, 400));
	points.push_back(cv::Point2f(100, 100));
	points.push_back(cv::Point2f(2, 10));
	points.push_back(cv::Point2f(250, 10));
	points.push_back(cv::Point2f(2, 250));
	points.push_back(cv::Point2f(250, 250));

	//First node
	tree->insert(points[0]); //1st point inserted
	tree->insert(points[1]); //2nd point inserted
	tree->insert(points[2]); //3rd point inserted
	tree->insert(points[3]); //4th point inserted
	tree->insert(points[4]); //point is out of boundary
	tree->insert(points[5]); //5th point inserted
	
	//Top left node
	tree->insert(points[6]); //6th point inserted -> makes the quadtree subdivide

	//Top right node
	tree->insert(points[7]);

	//Bottom left node
	tree->insert(points[8]);

	//Bottom right node
	tree->insert(points[9]);

	tree->markMask(mask, points, false);

	imshow("Quadtree Mask", mask);
	waitKey(0);

	delete tree;

	return 0;
}
