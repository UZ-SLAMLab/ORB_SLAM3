#include <iostream>
#include <opencv2/opencv.hpp>
#include <System.h>

int main() {
    cv::VideoCapture cap("data/ITT/GX010294.MP4");
    if (!cap.isOpened()) {
        std::cerr << "Error opening video file" << std::endl;
        return -1;
    }
    
    // Get the frames per second and calculate the time interval between frames
    int fps = cap.get(cv::CAP_PROP_FPS);
    float dT = 1.f / fps;

    ORB_SLAM3::System SLAM("Vocabulary/ORBvoc.txt", "Examples/ITT/itt.yaml", ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    cap.set(cv::CAP_PROP_POS_MSEC, (10 * 60 * 1000) + (47 * 1000));
    cv::Mat im;
    while (cap.read(im)) {
        // Process image
        double tframe = cap.get(cv::CAP_PROP_POS_MSEC) / 1000.0; // Get timestamp from video
        if(imageScale != 1.f) {
            cv::resize(im, im, cv::Size(), imageScale, imageScale);
        }
        SLAM.TrackMonocular(im, tframe); // Run the SLAM algorithm on the image
        // process the frame
        cv::waitKey(dT*1000);
    }
    cap.release();
    SLAM.Shutdown();
    return 0;
}
