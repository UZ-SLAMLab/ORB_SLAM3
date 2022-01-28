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

#include "proto/api/v1/robot.pb.h"
#include "proto/api/v1/robot.grpc.pb.h"
#include "proto/api/component/v1/camera.grpc.pb.h"
#include "proto/api/component/v1/camera.pb.h"

using namespace std;
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using proto::api::v1::RobotService;
using proto::api::v1::StatusRequest;
using proto::api::v1::StatusResponse;
using proto::api::component::v1::CameraService;
using proto::api::component::v1::CameraServiceRenderFrameRequest;
using proto::api::component::v1::CameraServiceGetFrameRequest;
using proto::api::component::v1::CameraServiceGetFrameResponse;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

int main(int argc, char **argv)
{  
       if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./rgbd_fromStream path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }

    struct sigaction sigIntHandler;

    // sigIntHandler.sa_handler = exit_loop_handler;
    // sigemptyset(&sigIntHandler.sa_mask);
    // sigIntHandler.sa_flags = 0;

    // sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms
   
    ClientContext context2;
  grpc::ChannelArguments ch_args;
  ch_args.SetMaxReceiveMessageSize(-1);
  const std::shared_ptr<Channel> channel2 = grpc::CreateCustomChannel("127.0.0.1:8080", grpc::InsecureChannelCredentials(),ch_args);
  const std::unique_ptr<CameraService::Stub> client2 = CameraService::NewStub(channel2);
  
//   CameraServiceRenderFrameRequest requestRender;
  CameraServiceGetFrameRequest requestGet;
  CameraServiceGetFrameResponse responseGet;
  cout << "yo yo yo" << endl;
  requestGet.set_mime_type("image/raw-depth");
  requestGet.set_name("front-raw");
  cout << requestGet.name() << endl;
  cout << "yo yo" << endl;
  const Status gStatus2 = client2->GetFrame(&context2,requestGet,&responseGet);  //render_frame(&context2, request2, &response2);
     cout << "yo" << endl;
  if (!gStatus2.ok()) {
    std::cout << "Status rpc failed." << gStatus2.error_code() <<  std::endl;
    return 1;
  }
  std::string json2;
  google::protobuf::util::MessageToJsonString(responseGet, &json2);
  std::cout << json2 << std::endl;

  return 0;
    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, true, 0, file_name);

    float imageScale = SLAM.GetImageScale();
    
    double timestamp;
    cv::Mat im, depth;
    
    while (!SLAM.isShutDown() && b_continue_session)
    {

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

        // im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
        // depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);
        if(imageScale != 1.f)
            {
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
                cv::resize(depth, depth, cv::Size(width, height));
            }


        // Pass the image to the SLAM system
        SLAM.TrackRGBD(im, depth, timestamp); //, vImuMeas); depthCV

    }
    cout << "System shutdown!\n";
    if(!b_continue_session){
        SLAM.Shutdown();
        SLAM.SaveTrajectoryEuRoC(file_name);
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }
    cout << "Yo Shutting" << endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    


    return 0;
}
