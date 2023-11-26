#include "mono_inertial_node.hpp"

#include<rclcpp/rclcpp.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/core.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui.hpp>

using std::placeholders::_1;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System* pSLAM, const bool bClahe) : 
    mpSLAM(pSLAM),
    mbClahe(bClahe), 
    Node("mono_inertial_node")
{
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 1000, std::bind(&MonoInertialNode::GrabImu, this, _1));
    sub_img0_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 1000, std::bind(&MonoInertialNode::GrabImage, this, _1));

    syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu,this);
}

MonoInertialNode::~MonoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;


    // Save camera trajectory
    mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // Stop all threads
    mpSLAM->Shutdown();

}

void MonoInertialNode::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

void MonoInertialNode::GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat MonoInertialNode::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {    
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

double MonoInertialNode::GetSeconds(builtin_interfaces::msg::Time stamp)
{
  return stamp.sec + stamp.nanosec / 1e9;
}

void MonoInertialNode::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tImg = 0;
    double tImu = 0;
    if (!img0Buf.empty()&& !imuBuf.empty())
    {
      tImg = this->GetSeconds(img0Buf.front()->header.stamp);
      if (tImg > this->GetSeconds(imuBuf.back()->header.stamp)) 
        continue;
    
      this->mBufMutex.lock();
      im = GetImage(img0Buf.front());
      img0Buf.pop();
      this->mBufMutex.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mBufMutex.lock();
      if(!imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!imuBuf.empty() && (imuBuf.front()->header.stamp.sec + imuBuf.front()->header.stamp.nanosec / 1e9) <=tImg)
        {
          double t = this->GetSeconds(imuBuf.front()->header.stamp);
          cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          
          imuBuf.pop();
        }
      }
      mBufMutex.unlock();
      
      if(mbClahe)
        mClahe->apply(im,im);
      
      mpSLAM->TrackMonocular(im, tImg, vImuMeas);

    }
    
    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}