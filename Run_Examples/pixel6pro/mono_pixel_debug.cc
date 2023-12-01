/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include"include/System.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &ImagesPathsVector, vector<double> &vTimeStamps);

int main()
{
    string file_name;
    file_name = string("/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/ORBSLAM_Results/monocular");
    cout << "file name: " << file_name << endl;

    // Load all sequences:
    vector<string> ImagesPathsVector;
    vector<double> vTimestampsCam;
    int nImages;
    string pathSeq("/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/Camera/Camera0");
    string pathTimeStamps("/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/Camera/Camera0.txt");
    string pathImu = "/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/joinedRawIMU.csv";
    cout << "test pt 1" << pathTimeStamps << endl;
    LoadImages(pathSeq, pathTimeStamps, ImagesPathsVector, vTimestampsCam);
    cout << "LOADED!" << endl;

    nImages = ImagesPathsVector.size();

    cout << endl << "-------" << endl;
    cout.precision(17);

    string vocabularyPath="/home/justmohsen/Documents/SLAM/ORBSLAM/Vocabulary/ORBvoc.txt";
    string SettingsPath="/home/justmohsen/Documents/SLAM/ORBSLAM/Run_Examples/pixel6pro/Pixel_mono.yaml";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabularyPath,SettingsPath,ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    {

        // Main loop
        cv::Mat im;
        int processImage = 0;
        for(int ni=0; ni<nImages; ni++, processImage++)
        {
            // Read image from file
            im = cv::imread(ImagesPathsVector[ni], cv::IMREAD_GRAYSCALE);
            double tframe = vTimestampsCam[ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << ImagesPathsVector[ni] << endl;
                return 1;
            }

            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }

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

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestampsCam[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[ni-1];

            if(ttrack<T) {
                usleep((T-ttrack)*1e6); // 1e6
            }
        }
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    const string kf_file = file_name + "_keyframes.txt";
    const string f_file = file_name + "_frames.txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".jpg");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);

        }
    }
}
