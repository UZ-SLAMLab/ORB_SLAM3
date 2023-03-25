#include <iostream>
#include <opencv2/opencv.hpp>
#include <System.h>

void readIMUData(const std::string &filename, std::vector<double> &vTimeStamps, std::vector<cv::Point3f> &vAcc, std::vector<cv::Point3f> &vGyro);

int main() {

    std::vector<double> vTimeStamps;
    std::vector<cv::Point3f> vAcc;
    std::vector<cv::Point3f> vGyro;
    
    readIMUData("data/k2/logs.csv", vTimeStamps, vAcc, vGyro);

    cv::VideoCapture cap("data/k2/GX010294.MP4");
    if (!cap.isOpened()) {
        std::cerr << "Error opening video file" << std::endl;
        return -1;
    }

    double time_difference = (27 * 60) + 55; // 27 minutes and 55 seconds in seconds
    for (auto &timestamp : vTimeStamps) {
        timestamp -= time_difference;
    }

    
    // Get the frames per second and calculate the time interval between frames
    int fps = cap.get(cv::CAP_PROP_FPS);
    float dT = 1.f / fps;

    ORB_SLAM3::System SLAM("Vocabulary/ORBvoc.txt", "Examples/k2.yaml", ORB_SLAM3::System::IMU_MONOCULAR, false);
    float imageScale = SLAM.GetImageScale();

    cap.set(cv::CAP_PROP_POS_MSEC, (10 * 60 * 1000) + (47 * 1000));

    size_t imu_data_idx = 0;
    double video_start_time = 10 * 60 + 47; // 10 minutes and 47 seconds in seconds

    while (imu_data_idx < vTimeStamps.size() && vTimeStamps[imu_data_idx] < video_start_time) {
        imu_data_idx++;
    }

    cv::Mat im;
    for (int i = 0; i < 5000; i++) {
        cap.read(im);

        // Get timestamp from video
        double tframe = cap.get(cv::CAP_PROP_POS_MSEC) / 1000.0;

        // Resize the image if needed
        if (imageScale != 1.f) {
            cv::resize(im, im, cv::Size(), imageScale, imageScale);
        }

        // Process and synchronize IMU data
        std::vector<ORB_SLAM3::IMU::Point> vIMUData;

        while (imu_data_idx < vTimeStamps.size() && vTimeStamps[imu_data_idx] <= tframe) {
            ORB_SLAM3::IMU::Point imuPoint(vAcc[imu_data_idx], vGyro[imu_data_idx], vTimeStamps[imu_data_idx]);
            vIMUData.push_back(imuPoint);
            imu_data_idx++;
        }

        // Run the SLAM algorithm on the image and synchronized IMU data
        SLAM.TrackMonocular(im, tframe, vIMUData);

        // You can add cv::waitKey here if you need to control the frame processing rate
        // cv::waitKey(dT * 1000);
    }
    cap.release();
    SLAM.Shutdown();
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    return 0;
}

void readIMUData(const std::string &filename, std::vector<double> &vTimeStamps, std::vector<cv::Point3f> &vAcc, std::vector<cv::Point3f> &vGyro) {
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Skip header line

    int lineNum = 2; // Start at line 2 (the first data line)
    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<float> data;

        int cellNum = 1; // Start at cell 1
        while (std::getline(lineStream, cell, ',')) {
            try {
                data.push_back(std::stof(cell));
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid argument on line " << lineNum << ", cell " << cellNum << ": " << e.what() << std::endl;
                std::cout << "Line: " << line << std::endl;
                throw e;
            }
            if (data.size() == 7) {
                break;
            }
            cellNum++;
        }

        vTimeStamps.push_back(data[0] / 1e6); // Convert microseconds to seconds
        vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
        vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
        // Ignore the remaining columns in the line

        lineNum++;
    }
}
