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

#include <iostream>
#include <algorithm>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

int LoadImages(const string &strAssociationFilename, vector<string> &masterImageFilenamesRGB,
               vector<string> &masterImageFilenamesD, vector<string> &slaveImageFilenamesRGB,
               vector<string> &slaveImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv) {
    if (argc != 5) {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association"
             << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> masterImageFilenamesRGB;
    vector<string> masterImageFilenamesD;
    vector<string> slaveImageFilenamesRGB;
    vector<string> slaveImageFilenamesD;
    vector<double> vTimestamps;
    auto const strAssociationFilename = string(argv[4]);
    auto const code = LoadImages(strAssociationFilename, masterImageFilenamesRGB, masterImageFilenamesD,
                                 slaveImageFilenamesRGB,
                                 slaveImageFilenamesD, vTimestamps);
    if (code == 1) return 1;

    // Check consistency in the number of images and depthmaps
    if (masterImageFilenamesRGB.empty() or masterImageFilenamesD.empty() or slaveImageFilenamesRGB.empty() or
        slaveImageFilenamesD.empty()) {
        cerr << endl << "No images were found in provided path." << endl;
        return 1;
    } else if (masterImageFilenamesRGB.size() != masterImageFilenamesD.size()) {
        cerr << endl << "Different number of images for rgb master and depth master." << endl;
        return 1;
    } else if (masterImageFilenamesRGB.size() != slaveImageFilenamesD.size()) {
        cerr << endl << "Different number of images for rgb master and depth slave." << endl;
        return 1;
    } else if (masterImageFilenamesRGB.size() != slaveImageFilenamesRGB.size()) {
        cerr << endl << "Different number of images for rgb master and rgb slave." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    auto const nImages = (int) masterImageFilenamesRGB.size();
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    auto const imageScale = SLAM.GetImageScale();
    for (int ni = 0; ni < nImages; ni++) {
        // Read image and depthmap from file
        auto imRGBMaster = cv::imread(string(argv[3]) + "/" + masterImageFilenamesRGB[ni],
                                      cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        auto imDMaster = cv::imread(string(argv[3]) + "/" + masterImageFilenamesD[ni],
                                    cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        // Slave
        auto imRGBSlave = cv::imread(string(argv[3]) + "/" + slaveImageFilenamesRGB[ni],
                                     cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        auto imDSlave = cv::imread(string(argv[3]) + "/" + slaveImageFilenamesD[ni],
                                   cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imRGBMaster.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << masterImageFilenamesRGB[ni] << endl;
            return 1;
        }
        if (imRGBSlave.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << slaveImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if (imageScale != 1.f) {
            int width = imRGBMaster.cols * imageScale;
            int height = imRGBMaster.rows * imageScale;
            cv::resize(imRGBMaster, imRGBMaster, cv::Size(width, height));
            cv::resize(imDMaster, imDMaster, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGBMaster, imDMaster, imRGBSlave, imDSlave, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

int LoadImages(const string &strAssociationFilename, vector<string> &masterImageFilenamesRGB,
               vector<string> &masterImageFilenamesD, vector<string> &slaveImageFilenamesRGB,
               vector<string> &slaveImageFilenamesD, vector<double> &vTimestamps) {
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    if (!fAssociation) {
        cerr << endl << "Error was occurred while opening the image" << endl;
        return 1;
    }
    while (!fAssociation.eof()) {
        string s;
        getline(fAssociation, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double timestamp;
            string sRGB, sD;
            ss >> timestamp;
            vTimestamps.push_back(timestamp);
            ss >> sRGB;
            masterImageFilenamesRGB.push_back(sRGB);
            ss >> timestamp;
            ss >> sD;
            masterImageFilenamesD.push_back(sD);
            string sRGBs, sDs;
            ss >> sRGBs;
            slaveImageFilenamesRGB.push_back(sRGBs);
            ss >> sDs;
            slaveImageFilenamesD.push_back(sDs);
        }
    }
    return 0;
}
