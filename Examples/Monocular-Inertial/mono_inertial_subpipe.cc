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
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const std::string &strAccPath, 
                std::vector<std::string> &vImagePaths, std::vector<double> &vTimeStamps);

void LoadIMU(const std::string &strAccPath, const std::string &strGyroPath, std::vector<double> &vTimeStamps, std::vector<cv::Point3f> &vAcc, std::vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;
int main(int argc, char *argv[])
{

    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_inertial_subpipe path_to_vocabulary path_to_settings path_to_sequence_folder_1 " << endl;
        return 1;
    }

    const int num_seq = argc-3;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "Path to files: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);
    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);
    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "..."<< endl;

        string pathSeq(argv[seq + 3]);

        string pathEstimatedState = pathSeq + "/EstimatedState.csv";
        string pathAcc  = pathSeq + "/Acceleration.csv";
        string pathGyro = pathSeq + "/AngularVelocity.csv";

        LoadImages(pathEstimatedState, vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;
        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathAcc, pathGyro, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;
        nImages[seq] = vstrImageFilenames[seq].size();
        cout << nImages[seq] << endl;

        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();
        cout << nImu[seq] << endl;

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    int proccIm=0;
    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read image from file
            im = cv::imread(string(argv[3])+"/Cam0_images/"+vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);

            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
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

                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
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
            ttrack_tot += ttrack;
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const std::string &strAccPath, std::vector<std::string> &vImagePaths, std::vector<double> &vTimeStamps) {
    std::ifstream fAcc(strAccPath);
    std::string line;

    // Skip the header
    std::getline(fAcc, line);

    while (std::getline(fAcc, line)) {
        std::istringstream ss(line);

        std::string imagePath;
        double timestamp;
        std::string dummy;  // for skipping other data fields

        // Parse image path and timestamp
        std::getline(ss, imagePath, ',');
        std::getline(ss, dummy, ','); timestamp = std::stod(dummy);

        // Skip the rest of the data in the line
        std::getline(ss, dummy);

        // Extracting just the filename from the path using dirent.h
        size_t lastSlashPos = imagePath.find_last_of('/');
        std::string fileName = (lastSlashPos != std::string::npos) ? imagePath.substr(lastSlashPos + 1) : imagePath;

        vImagePaths.push_back(fileName);
        vTimeStamps.push_back(timestamp);
    }

    fAcc.close();
}

void LoadIMU(const std::string &strAccPath, const std::string &strGyroPath, std::vector<double> &vTimeStamps, std::vector<cv::Point3f> &vAcc, std::vector<cv::Point3f> &vGyro) {
    std::ifstream fAcc, fGyro;
    fAcc.open(strAccPath);
    fGyro.open(strGyroPath);

    std::string lineAcc, lineGyro;

    // Skip headers
    std::getline(fAcc, lineAcc);
    std::getline(fGyro, lineGyro);

    while (std::getline(fAcc, lineAcc) && std::getline(fGyro, lineGyro)) {
        std::istringstream ssAcc(lineAcc), ssGyro(lineGyro);

        std::string sImageAcc, sImageGyro;
        double timestampAcc, timestampGyro;
        float accX, accY, accZ, gyroX, gyroY, gyroZ;

        std::string token;

        // Parse acceleration data
        std::getline(ssAcc, sImageAcc, ','); // Skip image path
        std::getline(ssAcc, token, ','); timestampAcc = std::stod(token);
        std::getline(ssAcc, token, ','); accX = std::stof(token);
        std::getline(ssAcc, token, ','); accY = std::stof(token);
        std::getline(ssAcc, token, ','); accZ = std::stof(token);

        // Parse gyroscope data
        std::getline(ssGyro, sImageGyro, ','); // Skip image path
        std::getline(ssGyro, token, ','); timestampGyro = std::stod(token);
        std::getline(ssGyro, token, ','); gyroX = std::stof(token);
        std::getline(ssGyro, token, ','); gyroY = std::stof(token);
        std::getline(ssGyro, token, ','); gyroZ = std::stof(token);

        // Check if timestamps match
        if (timestampAcc != timestampGyro) {
            std::cerr << "Warning: Mismatched timestamps found:" << std::endl;
            std::cout << "Image Acc: " << sImageAcc << ", Timestamp Acc: " << timestampAcc << std::endl;
            std::cout << "Image Gyro: " << sImageGyro << ", Timestamp Gyro: " << timestampGyro << std::endl;
            continue;
        }

        vTimeStamps.push_back(timestampAcc);
        vAcc.emplace_back(accX, accY, accZ);
        vGyro.emplace_back(gyroX, gyroY, gyroZ);
    }

    fAcc.close();
    fGyro.close();
}

