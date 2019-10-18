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

/*
    In this test module is necessary to pass a index file of a dataset RGBD 
    and the number of a frame in sequece to be search in the memory.

*/

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rgbd_loader.h>
#include <surf_tracker.h>
#include <visual_memory.h>
using namespace std;

bool is_KF(vector<Point2f> curr_KPs, vector<Point2f> KPs_from_last_KF)
{
    int dif = curr_KPs.size() - KPs_from_last_KF.size();
    if (dif < 0)
        dif = -dif;
    return (
        dif >= (int)KPs_from_last_KF.size() * 60 / 100);
}
vector<char> digit_extrator(int number)
{
    vector<char> digits;
    while (number > 0)
    {
        digits.push_back(number % 10 + '0');
        number = number / 10;
    }
    std::reverse(digits.begin(), digits.end());

    return digits;
}

int main(int argc, char **argv)
{
    string index_file_name;
    RGBDLoader loader;
    SurfTracker tracker_surf(400);
    VisualMemory memory;
    Mat frame, prev_frame, depth;
    if (argc != 3)
    {
        fprintf(stderr, "Usage: %s <index file> <frame_idx_to_search\n", argv[0]);
        exit(0);
    }

    index_file_name = argv[1];

    loader.processFile(index_file_name);

    int frame_idx;
    frame_idx = atoi(argv[2]);
    vector<char> digits = digit_extrator(frame_idx);

    Mat img;

    vector<pair<Mat, int>> key_frame_set;

    vector<Point2f> last_KF_points;

    for (int i = 0; i < loader.num_images_; i++)
    {

        //Load RGB-D image
        loader.getNextImage(frame, depth);
        if (i == 200)
            break;
        if (i == frame_idx)
            img = frame;

        if (i > 0)
        {
            tracker_surf.detectAndMatch(frame, prev_frame);

            if (is_KF(tracker_surf.curr_good_Pts_, last_KF_points))
            {
                cout << "keyframe adicionado, seu índice é: " << i << endl;

                last_KF_points = tracker_surf.curr_good_Pts_;

                key_frame_set.push_back(pair<Mat, int>(frame, i));
                memory.add(tracker_surf.getLastTrainDescriptor());
            }
        }

        prev_frame = frame;
    }

    vector<int> img_idx_set = memory.searchImage(img);

    string str = "searched frame ";
    for (int i = 0; i < (int)digits.size(); i++)
        str += digits[i];
    imshow(str, img);

    for (int i = 0; i < (int)img_idx_set.size(); i++)
    {

        int idx = img_idx_set[i];
        vector<char> global_idx = digit_extrator(key_frame_set[img_idx_set[i]].second);

        string text = "correspondence ";
        for (int i = 0; i < (int)global_idx.size(); i++)
            text += global_idx[i];

        imshow(text, key_frame_set[idx].first);
        waitKey();
    }

    return 0;
}
