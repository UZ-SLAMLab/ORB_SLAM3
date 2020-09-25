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
#include<chrono>
#include <future>
// #include <stout/stringify.hpp>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"System.h"

#include"ViewerAR.h"

using namespace std;


ORB_SLAM3::ViewerAR viewerAR;
bool bRGB = true;
bool bARMode = true;

cv::Mat K;
cv::Mat DistCoef;

int processing(char **argv, ORB_SLAM3::System *slamPtr);
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const cv::Mat& image, const double &timestamp);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    int main_error = 0;

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono index_to_camera or path_to_video path_to_vocabulary path_to_settings" << endl;        
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[2],argv[3],ORB_SLAM3::System::MONOCULAR, !bARMode);
   
    cout << endl << endl;
    cout << "-----------------------" << endl;
    cout << "Augmented Reality Demo" << endl;
    cout << "1) Translate the camera to initialize SLAM." << endl;
    cout << "2) Look at a planar region and translate the camera." << endl;
    cout << "3) Press Insert Cube to place a virtual cube in the plane. " << endl;
    cout << endl;
    cout << "You can place several cubes in different planes." << endl;
    cout << "-----------------------" << endl;
    cout << endl;

    if(bARMode)
    {
        viewerAR.SetSLAM(&SLAM);
    
        cv::FileStorage fSettings(argv[3], cv::FileStorage::READ);
        bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
        float fps = fSettings["Camera.fps"];
        viewerAR.SetFPS(fps);

        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        viewerAR.SetCameraCalibration(fx,fy,cx,cy);

        K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;

        DistCoef = cv::Mat::zeros(4,1,CV_32F);
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

        int width = fSettings["Camera.width"];
        int height = fSettings["Camera.height"];
        viewerAR.CreatePanel(width, height);
    }
        
    if(!bARMode)
        SLAM.CreatePanelToViewer();  

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Try Case1: crashed thread, I'm not working (from: https://github.com/UZ-SLAMLab/ORB_SLAM3/pull/13#issuecomment-698177041):
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    // auto resultFuture = std::async(std::launch::async, processing, argv, &SLAM);
    // if(!bARMode)
    //     SLAM.RunViewer();
    // else
    // {
    //     viewerAR.Run();
    // }
    // cout << "main end !\n" <<endl;
    // return resultFuture.get();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Case1 end
    /////////////////////////////////////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Try Case2: I'm working:
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    ImageGrabber igb(&SLAM);
    
    cv::VideoCapture cap(argv[1]); //camera
    cv::Mat im;
    double tframe = 0;
    while (1)
    {
        if(!cap.read(im))
        {
            cerr << "read error end!" << endl;
            main_error = 1;
            break;
        }
    
        tframe = 1 / cap.get(CV_CAP_PROP_FPS);
        igb.GrabImage(im, tframe);
        
        if(bARMode)
        {
            viewerAR.Refresh();
            usleep(viewerAR.GetMT()*1000);
        }
        else
        {
            if(SLAM.RefreshViewerWithCheckFinish())
            {
                SLAM.SetViewerFinish();
                main_error = 1;
                break;
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Case2 end
    /////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return main_error;

}

int processing(char **argv, ORB_SLAM3::System *slamPtr)
{  
    ImageGrabber igb(slamPtr);
    
    cv::VideoCapture cap(argv[1]); //camera
    cv::Mat im;
    double tframe = 0;
    int main_error = 0;
    const double FPS = cap.get(CV_CAP_PROP_FPS);
    while (1)
    {
        if(!cap.read(im))
        {
            cerr << "read error end!" << endl;
            main_error = 1;
            break;
        }
    
        tframe = 1 / FPS;
        igb.GrabImage(im, tframe);
    }

    cout << "processing...\n" <<endl;
    return main_error;
}

void ImageGrabber::GrabImage(const cv::Mat& image, const double &timestamp)
{
    if(bARMode)
    {   
        cv::Mat im = image.clone();
        cv::Mat imu;
        cv::Mat Tcw = mpSLAM->TrackMonocular(image, timestamp);

        int state = mpSLAM->GetTrackingState();
        vector<ORB_SLAM3::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
        vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();

        cv::undistort(im,imu,K,DistCoef);

        if(bRGB)
            viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
        else
        {
            cv::cvtColor(imu,imu,CV_RGB2BGR);
            viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
        }  
    }  
    else
    {
        mpSLAM->TrackMonocular(image, timestamp);
    }

}


