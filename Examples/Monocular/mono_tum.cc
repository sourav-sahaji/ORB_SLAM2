/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#define SKIP_IMAGES         0
#define SKIP_INIT_IMAGES    0

using namespace std;
using namespace cv;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

//    cv::VideoCapture cap1("/home/localuser/data/slam/panasonic-s11-changeConditions.MP4");
//    cv::VideoCapture cap1("/media/localuser/Default/workspace/data/seqSlam_test_data/s11-guiabot/left/stereo-left.avi");
//    cv::VideoCapture cap1("/home/localuser/data/vi-sensor/images/frame%04d.jpg");
    //    cv::VideoCapture cap1("/home/localuser/workspace/generalmgProc/bin/panasonic-testStepChange-darkToBright.avi");
    cv::VideoCapture cap1("/home/localuser/data/slam/sony/71-last.mkv");
//    cv::VideoCapture cap1("/media/localuser/My Passport/mm_datasets/00071.MTS");
//    cv::VideoCapture cap1("/media/localuser/My Passport/slamDatasets/sony/71-last.mkv");
//    cv::VideoCapture cap1("/home/localuser/data/slam/panasonic/panasonic-s11-changeConditions-firstTwoLaps.mkv");
//    cv::VideoCapture cap1("/media/localuser/My Passport/slamDatasets/panasonic/s11-testSeqSLAM-LC-pose/4.MP4");

    if(!cap1.isOpened())
    {
        cerr << "Video not found" << endl;
        exit(-1);
    }
    // Retrieve paths to images
//    vector<string> vstrImageFilenames;
//    vector<double> vTimestamps;
//    string strFile = string(argv[3])+"/rgb.txt";
//    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = cap1.get(CV_CAP_PROP_FRAME_COUNT);//vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Skipping images: " << SKIP_IMAGES << endl;
    cout << "Total images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0;; ni++)
    {
        while(ni<SKIP_INIT_IMAGES)
        {
            cap1.grab();
            ni++;
        }

        // Read image from file
//        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = ni;//vTimestamps[ni];
        cap1 >> im;


        if(im.empty())
        {
            cerr << endl << "Failed to load image " << endl;
//                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            break;
        }
        cv::resize(im,im,cv::Size(im.cols/2,im.rows/2));
//        im.rowRange(0,im.rows/2).copyTo(im);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);

        // Wait to load the next frame
//        double T=0;
//        if(ni<nImages-1)
//            T = ni+1-tframe;
//        else if(ni>0)
//            T = tframe-ni-1;

//        if(ttrack<T)
//            usleep((T-ttrack)*1e6);

        for(int i1=0; i1<SKIP_IMAGES; i1++)
        {
            cap1.grab();
            ni++;
        }
    }

    // Stop all threads
    SLAM.Shutdown();
    nImages = vTimesTrack.size();
    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
