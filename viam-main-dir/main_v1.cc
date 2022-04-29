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

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;


bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

// void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
//                 vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadImagesRGBD(const string &pathSeq, const string &strPathTimes,
                vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimeStamps);
void SavePCD(std::vector<ORB_SLAM3::MapPoint*> mapStuff, string file_name);

int main(int argc, char **argv)
{
    
    if(argc < 5)
    {
        cerr << endl << "Usage: ./rgbd_file path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (trajectory_file_name)" << endl;
	 cerr << endl << "Example: ./initialAttempt/rgbd_file ./Vocabulary/ORBvoc.txt ./initialAttempt/realsense515_depth.yaml ./officePics Out_file.txt outputPose" << endl;
        return 1;
    }
    string path_to_vocab = string(argv[1]);
    string path_to_settings = string(argv[2]);
    string path_to_data = string(argv[3]);
    string path_to_sequence = string(argv[4]);
    string output_file_name = string(argv[5]);
    string file_name,file_nameTraj,file_nameKey;
    string slam_mode = "RGBD";

        file_name = output_file_name;
        file_nameTraj = file_name;
        file_nameKey = file_name;
        file_nameTraj = file_nameTraj.append(".txt");
        file_nameKey = file_nameKey.append("Keyframe.txt");


    
	
    if (slam_mode == "RGBD"){
        
    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(path_to_data) + "/" + string(path_to_sequence);
    string pathSeq(path_to_data);
    LoadImagesRGBD(pathSeq, strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(path_to_vocab,path_to_settings,ORB_SLAM3::System::RGBD,false, 0, file_nameTraj);
    float imageScale = SLAM.GetImageScale();

    // Main loop
    cv::Mat imRGB, imD;
    Sophus::SE3f pose;
      
    // while (!SLAM.isShutDown() && b_continue_session)
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }


        // Pass the image to the SLAM system
        pose = SLAM.TrackRGBD(imRGB,imD,tframe);


    }
    }
    else if(slam_mode == "MONO"){
        //load stuff
        int nImages=0;
        cv::Mat im;

        // Initialize SLAM
        ORB_SLAM3::System SLAM(path_to_vocab,path_to_settings,ORB_SLAM3::System::MONOCULAR,false, 0, file_nameTraj);

        // while (!SLAM.isShutDown() && b_continue_session)
        {
            // Read image and depthmap from file
            // imRGB = cv::imread(vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
            // double tframe = vTimestamps[ni];

            // if(imRGB.empty())
            // {
            //     cerr << endl << "Failed to load image at: "
            //         << vstrImageFilenamesRGB[ni] << endl;
            //     return 1;
            // }

            // if(imageScale != 1.f)
            // {
            //     int width = imRGB.cols * imageScale;
            //     int height = imRGB.rows * imageScale;
            //     cv::resize(imRGB, imRGB, cv::Size(width, height));
            // }
            // SLAM.TrackMonocular(im, timestamp);
        }

    }
    // // Stop all threads
    // std::vector<ORB_SLAM3::MapPoint*> mapStuff = SLAM.GetAtlas()->GetCurrentMap()->GetAllMapPoints();
    //     // Map* GetCurrentMap();
    //     // mapStuff = SLAM.GetTrackedMapPoints();
    //     cout << "Start to write PCD with datapoints: " << endl;
    //     cout << mapStuff.size() << endl;
    
    // SavePCD(mapStuff, file_name);

    // cout << "End to write PCD" << endl;
    // SLAM.Shutdown();
    // SLAM.SaveTrajectoryEuRoC(file_nameTraj);
    // SLAM.SaveKeyFrameTrajectoryEuRoC(file_nameKey);
    // cout << "Yo Shutting" << endl;


    return 0;
}


void LoadImagesRGBD(const string &pathSeq, const string &strPathTimes,
                vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimeStamps)
{
    string pathCam0 = pathSeq + "/rgb";
	string pathCam1 = pathSeq + "/depth";
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageFilenamesRGB.reserve(5000);
    vstrImageFilenamesD.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageFilenamesRGB.push_back(pathCam0 + "/" + ss.str());
            vstrImageFilenamesD.push_back(pathCam1 + "/" + ss.str());
            double t;
            string timestring = s.substr(0, s.find_last_of("."));
	    std::string::size_type sz;  
	    // cout << timestring << endl;
        //     timestring >> t;
	    t = std::stod(timestring,&sz);
            vTimeStamps.push_back(t);

        }
    }
}


void SavePCD(std::vector<ORB_SLAM3::MapPoint*> mapStuff, string file_name){

    string pathSaveFileName = "./";
        pathSaveFileName = pathSaveFileName.append(file_name);
        pathSaveFileName.append(".pcd");
        std::remove(pathSaveFileName.c_str());
        std::ofstream ofs(pathSaveFileName, std::ios::binary);
        // boost::archive::text_oarchive oa(ofs);
        ofs  << "VERSION .7\n"
            << "FIELDS x y z\n"
            << "SIZE 4 4 4\n"
            << "TYPE F F F\n"
            << "COUNT 1 1 1\n"
            << "WIDTH "
            << mapStuff.size()
            << "\n"
            << "HEIGHT " << 1 << "\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS "
            << mapStuff.size()
            << "\n"
            << "DATA ascii\n";
	for (auto p : mapStuff) {
		Eigen::Matrix<float, 3, 1> v = p->GetWorldPos();//ORB_SLAM3::Converter::toVector3d(p->GetWorldPos());
		// std::cout << v.x() << "," << v.y() << "," << v.z() << std::endl;
        ofs << v.x()  << " " << v.y()  << " " << v.z()  << "\n";
	}
    ofs.close();
}