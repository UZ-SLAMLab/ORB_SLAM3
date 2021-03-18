/**
* Test video stream with ORB-SLAM2 - QR Code tracker.
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
#include <sys/stat.h>
#include <QrCodeTracker.h>

#include "realsense.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

int main(int argc, char **argv)
{
  if(argc != 1)
  {
    cerr << endl << "Usage: ./qrCode_live" << endl;
    return 1;
  }

  try {
    RealSense::sModality mode = RealSense::RGBD;
    RealSense realsense(mode);
    QrCodeTracker *qrCodeTracker = new QrCodeTracker();
    qrCodeTracker->loadQrCodeList();

    // Main loop
    for(;;)
    {
      realsense.run();

      qrCodeTracker->Track(realsense.getColorMatrix(), Point2d(2.13, 4.82));
      qrCodeTracker->display();

      int key = waitKey(10);
      // Stop everything
      if( key == 32 ) {
        qrCodeTracker->saveQrCodeList();
        cout << "Shutting down..." << endl;
        break;
      }
    }
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}