/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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
#include<iomanip>
#include<chrono>

#include <gflags/gflags.h>
#include <torch/script.h>
#include <torch/torch.h>

#include "iv_slam_helpers/torch_helpers.h"

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs/legacy/constants_c.h>

#include<System.h>

DECLARE_bool(help);

DEFINE_string(path_to_vocabulary,"" , "Absolute path to the ORB vocabulary.");
DEFINE_string(path_to_settings,"" , "Absolute path to the settings.");
DEFINE_string(path_to_sequences,"" , "Absolute path to the stereo image sequences.");
DEFINE_string(path_to_introspection_model,"" , "Absolute path to the introspection model");

DEFINE_bool(introspection_on, false, "Run ORB-SLAM3 with the introspection function - GPU suggested.");
DEFINE_bool(gpu_available, false, "Set to true if a GPU is available to use.");
DEFINE_bool(viewer_on, true, "Enable image and keyframe viewer.");


using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);

    // Load introspection model
    torch::jit::script::Module introspection_model;
    torch::Device device = torch::kCPU;
    if(FLAGS_introspection_on){
        // Check if we have a GPU to run on
        if (FLAGS_gpu_available && torch::cuda::is_available()) {
            device = torch::kCUDA;
            cout << "Introspection model running on GPU :)" << endl;
        } else {
            cout << "Introspection model running on CPU :(" << endl;
        }
        try {
            // Deserialize the ScriptModule from file
            introspection_model = torch::jit::load(FLAGS_path_to_introspection_model);
            introspection_model.to(device);
        } catch (const c10::Error &e) {
            cerr << "Error deserializing the ScriptModule from file" << endl;
            return -1;
        }
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(FLAGS_path_to_sequences), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(FLAGS_path_to_vocabulary, FLAGS_path_to_settings, ORB_SLAM3::System::STEREO, FLAGS_viewer_on);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }
        
        // TODO rectify and undistort OG images?

        // Feed image to model to create cost mask 
        cv::Mat cost_img_cv;
        at::Tensor cost_img;
        if(FLAGS_introspection_on){
            // Run inference on the introspection model online
            cv::Mat imLeft_RGB = imLeft;  //TODO initializae imLeft_RGB as blank instead of imLeft
            cv::cvtColor(imLeft_RGB, imLeft_RGB, CV_BGR2RGB);

            // Convert to float and normalize image
            imLeft_RGB.convertTo(imLeft_RGB, CV_32FC3, 1.0 / 255.0);
            cv::subtract(imLeft_RGB, cv::Scalar(0.485, 0.456, 0.406), imLeft_RGB); // TODO what are these numbers
            cv::divide(imLeft_RGB, cv::Scalar(0.229, 0.224, 0.225), imLeft_RGB);

            auto tensor_img = ORB_SLAM3::CVImgToTensor(imLeft_RGB);
            // Swap axis
            tensor_img = ORB_SLAM3::TransposeTensor(tensor_img, {(2), (0), (1)});
            // Add batch dim
            tensor_img.unsqueeze_(0);

            tensor_img = tensor_img.to(device);
            std::vector<torch::jit::IValue> inputs{tensor_img};
            cost_img = introspection_model.forward(inputs).toTensor();
            cost_img = (cost_img * 255.0).to(torch::kByte);
            cost_img = cost_img.to(torch::kCPU);

            cost_img_cv = ORB_SLAM3::ToCvImage(cost_img);
        }

        // TODO rectify and undistort cost images?

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        if(FLAGS_introspection_on){
            SLAM.TrackStereo(imLeft, imRight, tframe, FLAGS_introspection_on, cost_img_cv);
        } else {
            SLAM.TrackStereo(imLeft, imRight, tframe);
        }
        

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

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
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
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
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
