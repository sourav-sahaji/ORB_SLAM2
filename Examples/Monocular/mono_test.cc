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

using namespace std;
using namespace cv;

void rectifyImage(cv::Mat& inImg,string &strSettingPath)
{
    cv::Mat mK, mDistCoef, outImg;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    cv::undistort(inImg,outImg,mK,mDistCoef);

    outImg.copyTo(inImg);
}

int main(int argc, char **argv)
{
//    VideoCapture cap1("/media/sourav/Default4/Users/n9349995/Desktop/dataset/slamData/footpath-speedVary-3/night-2.mp4");
//    VideoCapture cap1("/media/sourav/My Passport3/current/data/MLRSCD/Highway/NIR.avi");
    VideoCapture cap1("/media/sourav/Default4/Users/n9349995/Desktop/dataset/slamData/parking_amrapali/round-3.mov");
//    VideoCapture cap1("/media/sourav/Default4/Users/n9349995/Desktop/dataset/slamData/home-indoor-outdoor/dayLeft.mp4");
    if(!cap1.isOpened())
    {
        cerr << "Video not found" << endl;
        exit(-1);
    }

    int nImages = cap1.get(CV_CAP_PROP_FRAME_COUNT);

    string vocFile = "/home/sourav/workspace/ORB_SLAM2/Vocabulary/ORBvoc.txt";
//    string settingFile = "/home/sourav/workspace/ORB_SLAM2/Examples/Monocular/motom.yaml";
//    string settingFile = "/home/sourav/workspace/ORB_SLAM2/Examples/Monocular/mlrscd.yaml";
    string settingFile = "/home/sourav/workspace/ORB_SLAM2/Examples/Monocular/iphone.yaml";

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocFile,settingFile,ORB_SLAM2::System::MONOCULAR,true);
//    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    int resetCounter = 0;
    int maxImages = 999999999;

    cv::Mat im;
    int counter = 0;
    while(counter++<4400)
        cap1>>im;
//    maxImages = 4800;

    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        cap1 >> im;

//        cv::resize(im,im,cv::Size(512,288));
        double tframe = ni;

        if(ni>maxImages)
        {
            cerr << "Max limit on images set to " << maxImages << endl;
            cerr << "Stopping the system" << endl;
            break;
        }

        if(im.empty())
        {
            cerr << endl << "Failed to load image" << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

//        rectifyImage(im,settingFile);
        cv::resize(im,im,cv::Size(512,288));

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

        if(SLAM.GetTrackingState() == 3 && SLAM.GetNumKFs() >= 5)
        {
            std::cout << "Current Tracking State - LOST, Hence resetting the system after saving the trajectory..." << std::endl;
            stringstream kfTrajFileName;
            kfTrajFileName << "kfTrajFile-2-" << resetCounter << ".txt";
            SLAM.SaveKeyFrameTrajectoryTUM(kfTrajFileName.str());
            SLAM.Reset();
            resetCounter++;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
//        double T=0;
//        if(ni<nImages-1)
//            T = vTimestamps[ni+1]-tframe;
//        else if(ni>0)
//            T = tframe-vTimestamps[ni-1];

//        if(ttrack<T)
//            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

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
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory-2.txt");

    return 0;
}
