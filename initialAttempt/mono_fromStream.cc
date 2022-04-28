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

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <System.h>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <google/protobuf/util/json_util.h>
#include <grpcpp/security/credentials.h>

#include "proto/api/robot/v1/robot.pb.h"
#include "proto/api/robot/v1/robot.grpc.pb.h"
#include "proto/api/component/camera/v1/camera.grpc.pb.h"
#include "proto/api/component/camera/v1/camera.pb.h"

using namespace std;
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using proto::api::robot::v1::RobotService;
using proto::api::robot::v1::StatusRequest;
using proto::api::robot::v1::StatusResponse;
using proto::api::component::camera::v1::CameraService;
using proto::api::component::camera::v1::RenderFrameRequest;
using proto::api::component::camera::v1::GetFrameRequest;
using proto::api::component::camera::v1::GetFrameResponse;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
}
// void decodeFrame(char *frame, )
int main(int argc, char **argv)
{  
       if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./rgbd_fromStream path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name,file_nameTraj,file_nameKey;
    bool bFileName = false;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
        file_nameTraj = file_name;
        file_nameKey = file_name;
        bFileName = true;
        file_nameTraj = file_nameTraj.append(".txt");
        file_nameKey = file_nameKey.append("Keyframe.txt");
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms


std::string::size_type n;



cv::Mat im,depth, rawData;

    int frameWidth,frameHeight, nSize, location;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM("./Vocabulary/ORBvoc.txt",argv[2],ORB_SLAM3::System::MONOCULAR, false, 0, file_nameTraj);

    float imageScale = SLAM.GetImageScale();
    
    double timestamp, grpcDuration, slamDuration;
    // auto timeStart = std::chrono::system_clock::now();
    std::chrono::steady_clock::time_point timeStart = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point timeNow = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point timeBegin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point timeGRPC = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point timeFRAME = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point timeSLAM = std::chrono::steady_clock::now();
    timestamp = std::chrono::duration_cast<std::chrono::duration<double> >(timeNow - timeStart).count();
    while (!SLAM.isShutDown() && b_continue_session)
    {
      
      // cout << "yo loop start" << endl;
      timeBegin = std::chrono::steady_clock::now();
      ClientContext context2;
      grpc::ChannelArguments ch_args;
      ch_args.SetMaxReceiveMessageSize(-1);
      const std::shared_ptr<Channel> channel2 = grpc::CreateCustomChannel(argv[1], grpc::InsecureChannelCredentials(),ch_args);
      const std::unique_ptr<CameraService::Stub> client2 = CameraService::NewStub(channel2);
      
      GetFrameRequest requestGet;
      GetFrameResponse responseGet;
      requestGet.set_mime_type("image/jpeg");
      // requestGet.set_name("r-vg2105m05.color");
      requestGet.set_name("color");
      const Status gStatus2 = client2->GetFrame(&context2,requestGet,&responseGet);  //render_frame(&context2, request2, &response2);
      if (!gStatus2.ok()) {
        std::cout << "Status rpc failed." << gStatus2.error_code() <<  std::endl;
      b_continue_session = false;
      break;
      }
      frameWidth = (int) responseGet.width_px();
      frameHeight = (int) responseGet.height_px();

      cout << "width: "<< frameWidth << " height: " << frameHeight << endl;

      timeGRPC = std::chrono::steady_clock::now();
      grpcDuration = std::chrono::duration_cast<std::chrono::duration<double> >(timeGRPC - timeBegin).count();
      // cout << "grpc time: " << grpcDuration << endl;
      nSize = responseGet.image().length();
      std::vector<char> bytes(responseGet.image().begin(), responseGet.image().end());
      char *buffer = &bytes[0];
      location = 0;

        // Seq loop
        double t_resize = 0;
        double t_rect = 0;
        double t_track = 0;
        int num_rect = 0;
        int proccIm = 0;
        
            cv::Mat color_frame, depth_frame;

            // Trying to get both other and aligned depth frames
        color_frame = 0; //from GRPC+process
        depth_frame = 0; //from GRPC+process

        rawData =  cv::Mat(cv::Size(1,nSize-n), CV_8UC1, (void*)buffer, cv::IMREAD_COLOR);
        im = cv::imdecode(rawData,cv::IMREAD_COLOR);


        if(im.empty())
          {
              cerr << endl << "Failed to load image " << endl;
              // return 1;
          }
        // im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
        // depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);
        if(imageScale != 1.f)
            {
              cout << "yo" << endl;
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
                // cv::resize(depth, depth, cv::Size(width, height));
            }

// cout << "yo got frames" << endl;
        // Pass the image to the SLAM system
        
        timeNow = std::chrono::steady_clock::now();
        timestamp = std::chrono::duration_cast<std::chrono::duration<double> >(timeBegin - timeStart).count();
        // SLAM.TrackRGBD(im, depth, timestamp); //, vImuMeas); depthCV
        SLAM.TrackMonocular(im, timestamp);
        timeSLAM = std::chrono::steady_clock::now();
        slamDuration = std::chrono::duration_cast<std::chrono::duration<double> >(timeNow - timeBegin).count();
        cout << "slam time: " << 1.0/slamDuration << endl;
    }
    cout << "System shutdown!\n";
    if(!b_continue_session){
      std::vector<ORB_SLAM3::MapPoint*> mapStuff = SLAM.GetAtlas()->GetCurrentMap()->GetAllMapPoints();
        // Map* GetCurrentMap();
        // mapStuff = SLAM.GetTrackedMapPoints();
        cout << "Start to write PCD with datapoints: " << endl;
        cout << mapStuff.size() << endl;
        // std::cout << "# x,y,z" << std::endl;
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
    cout << "End to write PCD" << endl;
        SLAM.Shutdown();
        SLAM.SaveTrajectoryEuRoC(file_nameTraj);
        SLAM.SaveKeyFrameTrajectoryEuRoC(file_nameKey);
    }
    cout << "Yo Shutting Down" << endl;
    

    return 0;
}
