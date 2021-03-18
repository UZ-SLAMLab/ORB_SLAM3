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

///Modified by Felix Holz on 2018-08-17


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <unistd.h>
#include <math.h>


#include<opencv2/core/core.hpp>

#include "System.h"
#include "fileutils.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

cv::Vec3d GetEulerFromMatrix(const cv::Mat& matrix);

string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

int main(int argc, char **argv)
{


    std::cout << "Starting..." << std::endl;
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    std::cout << "loading images..." << std::endl;

    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
    std::cout << "loaded images" << std::endl;

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System slam(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, true);


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;


    bool slam_mode = false;



    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        if(!slam_mode)
        {
            slam.DeactivateLocalizationMode();
            slam_mode = true;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        slam.TrackMonocular(im,tframe);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);

        if(cv::waitKey(10) >= 0)
        {
            std::cout << "Key Pressed" << std::endl;
            break;
        }


        std::cout << slam.GetCurrentCameraPose() << std::endl;



        
        if(ni > nImages) break;
    }

    // Stop all threads
    std::cout << "Shutting down SLAM..." << std::endl;
    slam.Shutdown();
    std::cout << "Shut down SLAM." << std::endl;

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
    slam.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/
cv::Vec3d GetEulerFromMatrix(const cv::Mat& matrix)
{
    double heading = atan2(-matrix.at<double>(2, 0), matrix.at<double>(0, 0));
    double bank = atan2(-matrix.at<double>(1, 2), matrix.at<double>(1, 1));
    double attitude = asin(matrix.at<double>(1, 0));

    return cv::Vec3d(heading, bank, attitude);
}
