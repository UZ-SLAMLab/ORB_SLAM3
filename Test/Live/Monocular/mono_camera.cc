/**
* Test video stream with ORB-SLAM2.
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>

using namespace std;

int main(int argc, char **argv)
{
  if(argc != 4)
  {
    cerr << endl << "Usage: ./Test path_to_vocabulary path_to_settings device_number" << endl;
    return 1;
  }

  cv::VideoCapture cap;
  // Open the camera on /dev/video#
  if(!cap.open(stoi(argv[3])))
  {
    cerr << endl << "Failed to open /dev/video" << argv[3] << endl;
    return 0;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, true);

  cout << endl << "-------" << endl;
  cout << "Start processing video stream ..." << endl;

  // Main loop
  cv::Mat frame, resized;
  for(;;)
  {
    // Get current timestamp
    double currentTs = (double)cap.get(cv::CAP_PROP_POS_MSEC);
    // Get video stream current frame
    cap >> frame;
    resize(frame, resized, cv::Size(640, 480));

    if(resized.empty())
    {
      cerr << endl << "Failed to load frame!!!" << endl;
      return 1;
    }

    // Pass the image to the SLAM system
    SLAM.TrackMonocular(resized, currentTs);

    int key = cv::waitKey(10);
    // Stop SLAM when Spacebar is pressed
    if( key == 32 ) {
      break;
    }
  }

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  // SLAM.SaveTrajectory("CameraTrajectory.dat");
  // SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.dat");

  return 0;
}