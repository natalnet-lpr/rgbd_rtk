/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Natalnet Laboratory for Perceptual Robotics
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
 */
 
#include <cstdio>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rgbd_loader.h>
#include <klttacw_tracker.h>
#include <common_types.h>
#include <graph.h>

using namespace std;
using namespace cv;


bool is_KeyFrame(int features_in_currF,Graph graph, float fkf){
	
	std::list<Vertice*>::iterator it = graph.graph.end();
	
		
	--it;

	
	float features_in_KF = (*it)->features_in_KF ;


	return sqrt((features_in_KF-features_in_currF)*(features_in_KF-features_in_currF))>=fkf*features_in_KF;

}


void draw_circle( Mat &img, Point2i pt2, float r){
	if(r>0){
		circle(img, pt2, r+5, CV_RGB(0,0,255), 1);

	}
	else {
		circle(img, pt2, 5, CV_RGB(0,0,255), 1);

	}
}

void draw_last_track(Mat& img, const vector<Point2f> prev_pts, const vector<Point2f> curr_pts)
{
	for(size_t k = 0; k < curr_pts.size(); k++)
	{
		Point2i pt1, pt2;
		pt1.x = prev_pts[k].x;
		pt1.y = prev_pts[k].y;
		pt2.x = curr_pts[k].x;
		pt2.y = curr_pts[k].y;

		circle(img, pt1, 1, CV_RGB(0,0,255), 1);
		circle(img, pt2, 3, CV_RGB(0,255,0), 1);
		line(img, pt1, pt2, CV_RGB(0,255,0));
	}
}

void draw_last_track(Mat& img, const vector<Point2f> prev_pts, const vector<Point2f> curr_pts,vector <float> radius)
{
	
	for(size_t k = 0; k < curr_pts.size(); k++)
	{	
		
		Point2i pt1, pt2; 
		pt1.x = prev_pts[k].x;
		pt1.y = prev_pts[k].y;
		pt2.x = curr_pts[k].x;
		pt2.y = curr_pts[k].y;

		
		if(!radius.empty() && k<=radius.size()){	
				
			draw_circle(img,pt2,radius[k]);
		}
		
		
		
		circle(img, pt1, 1, CV_RGB(0,0,255), 1);
		circle(img, pt2, 3, CV_RGB(0,255,0), 1);
		line(img, pt1, pt2, CV_RGB(0,255,0));
	}
}
int main(int argc, char **argv)
{
	string index_file_name;
	RGBDLoader loader;
	KLTTrackerACW tracker;
	
	Graph grafo;


	Mat frame, depth;

	if(argc != 4)
	{
		fprintf(stderr, "Usage: %s <index file> <size minimum of radius> <size maximum of radius>\n", argv[0]);
		exit(0);
	}

	index_file_name = argv[1];
	loader.processFile(index_file_name);
	tracker.radius_min=atof(argv[2]);
	tracker.radius_max=atof(argv[3]);
	//Track points on each image
	for(int i = 0; i < loader.num_images_; i++)
	{
		loader.getNextImage(frame, depth);

		tracker.track(frame);
		draw_last_track(frame, tracker.prev_pts_, tracker.curr_pts_, tracker.radius);	
				
		if(is_KeyFrame(tracker.curr_pts_.size(),grafo,0.1)){

			grafo.graph.push_back(grafo.add_new_KF(tracker.curr_pts_,frame));
			grafo.Update_Iterators();
		}
		//draw_tracks(frame, tracker.tracklets_);

		imshow("Image view", frame);
		//imshow("Depth view", depth);
		
		std::list<Vertice*>::iterator ultimo = grafo.last;
		std::list<Vertice*>::iterator penultimo = grafo.penult;

		imshow("Frame view", (*ultimo)->KeyFrame);

		
		char key = waitKey(15);
		if(key == 27 || key == 'q' || key == 'Q')
		{
			printf("Exiting.\n");
			break;
		}
	}

	return 0;
}
