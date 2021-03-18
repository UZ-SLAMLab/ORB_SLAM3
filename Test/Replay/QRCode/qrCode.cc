#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <QrCodeTracker.h>
#include <sys/types.h>
#include <dirent.h>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

int main(int argc, char **argv)
{
  if(argc != 2)
  {
    cerr << endl << "Usage: ./qrCode_replay" << endl 
                 << "         path_to_image_file" << endl;
    return 1;
  }

  Mat inputImage;
  inputImage = imread(argv[1]);

  try {
    cout << endl << "-------" << endl;
    cout << "Start processing image file ..." << endl;
    QrCodeTracker *qrCodeTracker = new QrCodeTracker();

    // qrCodeTracker->Track(inputImage, Point2d(2.13, 4.82));
    // qrCodeTracker->saveQrCodeList();
    cv::Point test;
    test = qrCodeTracker->Detect(inputImage);
    std::cout << test << std::endl;
    qrCodeTracker->display();
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  for(;;)
  {
    int key = waitKey(10);
    if( key == 32) {
      cout << "Shutting down..." << endl;
      break;
    }
  }

  return 0;
}
