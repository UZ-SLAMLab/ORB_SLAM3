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

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

int main()
{
    string file_name;
    file_name = string("/home/justmohsen/Documents/SLAM/Datasets/UZH/indoor_forward_3_snapdragon_results/mono_inertial");
    cout << "file name: " << file_name << endl;

    // Load all sequences:
    vector<string> ImagesPathsVector;
    vector<double> vTimestampsCam;
    vector<double> vTimestampsImu;
    vector<cv::Point3f> vAcc, vGyro;
    int nImages;
    int nImu;
    int first_imu = 0;
    string pathSeq("/home/justmohsen/Documents/SLAM/Datasets/UZH/indoor_forward_3_snapdragon");
    string pathTimeStamps("/home/justmohsen/Documents/SLAM/Datasets/UZH/indoor_forward_3_snapdragon/left_images.txt");
    string pathImu = "/home/justmohsen/Documents/SLAM/Datasets/UZH/indoor_forward_3_snapdragon/imu_reformatted.csv";

    cout << "test pt 1" << pathTimeStamps << endl;
    LoadImages(pathSeq, pathTimeStamps, ImagesPathsVector, vTimestampsCam);
    cout << "LOADED!" << endl;

    cout << "Loading IMU for sequence " << endl;
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
    cout << "LOADED!" << endl;

    nImages = ImagesPathsVector.size();
    nImu = vTimestampsImu.size();

    if((nImages<=0)||(nImu<=0))
    {
        cerr << "ERROR: Failed to load images or IMU for sequence" << endl;
        return 1;
    }

    // Find first imu to be considered, supposing imu measurements start first
    cout << vTimestampsImu[0] << endl;
    cout << vTimestampsCam[0] << endl;
    while(vTimestampsImu[first_imu]<=vTimestampsCam[0])
        first_imu++;
    first_imu--; // first imu measurement to be considered


    cout << endl << "-------" << endl;
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout.precision(17);

    string vocabularyPath="/home/justmohsen/Documents/SLAM/ORBSLAM/Vocabulary/ORBvoc.txt";
    string SettingsPath="/home/justmohsen/Documents/SLAM/ORBSLAM/Run_Examples/UZH/UZH_SNAP_mono_inertial.yaml";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabularyPath,SettingsPath,ORB_SLAM3::System::IMU_MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    {
        // Main loop
        cv::Mat im;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
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

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
            {
                // cout << "t_cam " << tframe << endl;

                while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x,vAcc[first_imu].y,vAcc[first_imu].z,
                                                             vGyro[first_imu].x,vGyro[first_imu].y,vGyro[first_imu].z,
                                                             vTimestampsImu[first_imu]));
                    first_imu++;
                }
            }

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im,tframe,vImuMeas); // TODO change to monocular_inertial

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
    string dummyLine;
    getline(fTimes, dummyLine); // dummy line
    //cout << "test pt 2" << endl;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        //cout << s << endl;
        if(!s.empty())
        {
            string itemz;
            size_t posz = 0;
            double dataz[3];
            int count = 0;
            while((posz = s.find(' ')) != string::npos){
                // cout << "test pt 4" << endl;
                itemz = s.substr(0, posz);
                dataz[count++] = stod(itemz);
                s.erase(0, posz + 1);
                // cout << "test pt 5: " << s << ":end"<< endl;
                if (count==2){
                    break;
                }
            }
            // itemz = s.substr(0, posz);
            itemz=s;
            itemz.erase(itemz.find_last_not_of(' ')+1);
            // dataz[2] = itemz;
            // cout <<"itemz: " <<itemz <<":end"<< endl;
            // stringstream ss;
            // ss << s;
            vstrImages.push_back(strImagePath + "/" + itemz);
            // double t;
            // ss >> t;
            double t=dataz[1];
            vTimeStamps.push_back(t);
            //cout << "t: " << t << endl;


        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}

