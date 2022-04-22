#include <opencv2/opencv.hpp>

#include "System.h"

#include <string>
#include <chrono>   // for time stamp
#include <iostream>

using namespace std;

string parameterFile = "./HuaWeiMatePro30.yaml";
string vocFile = "/home/lighthouse/orb_slam3/ORB_SLAM3/Vocabulary/ORBvoc.bin";

string videoFile = "/home/lighthouse/orb_slam3/testVideo/test.mp4";
string imageStorePath = "/home/lighthouse/orb_slam3/image/test/";

int main(int argc, char **argv) {

// 声明 ORB-SLAM2 系统
ORB_SLAM3::System SLAM(vocFile, parameterFile, ORB_SLAM3::System::MONOCULAR, true);
// 获取视频图像
cv::VideoCapture cap(videoFile);    // change to 1 if you want to use USB camera.
// 记录系统时间
auto start = chrono::system_clock::now();

while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据
        if (frame.empty())
            break;

        // rescale because image is too large
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(640,480));

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        double imageTimestamp = double(timestamp.count())/1000.0;

        string path = imageStorePath + to_string(imageTimestamp) + ".png";
        imwrite(path, frame_resized);

        SLAM.TrackMonocular(frame_resized, imageTimestamp);
        cv::waitKey(30);
    }

    SLAM.Shutdown();
    return 0;

}