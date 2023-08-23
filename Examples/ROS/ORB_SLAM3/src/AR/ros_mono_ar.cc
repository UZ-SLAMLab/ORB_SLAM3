#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include "../../../include/System.h"
#include "ViewerAR.h"

using namespace std;

ORB_SLAM3::ViewerAR viewerAR;
bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM) {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if (argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);

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

    viewerAR.SetSLAM(&SLAM);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
    float fps = fSettings["Camera.fps"];
    viewerAR.SetFPS(fps);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    viewerAR.SetCameraCalibration(fx, fy, cx, cy);

    K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;

    DistCoef = cv::Mat::zeros(5, 1, CV_32F); // Use 5 for k3 support
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    DistCoef.at<float>(4) = k3;

    std::thread tViewer(&ORB_SLAM3::ViewerAR::Run, &viewerAR);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ROS image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat im = cv_ptr->image.clone();
    cv::Mat imu;
    cv::Mat Tcw;
    Sophus::SE3f Tcw_SE3f = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix, Tcw);
    int state = mpSLAM->GetTrackingState();
    vector<ORB_SLAM3::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
    vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();

    cv::undistort(im, imu, K, DistCoef);

    if (bRGB)
        viewerAR.SetImagePose(imu, Tcw, state, vKeys, vMPs);
    else
    {
        cv::cvtColor(imu, imu, cv::COLOR_RGB2BGR);
        viewerAR.SetImagePose(imu, Tcw, state, vKeys, vMPs);
    }
}
