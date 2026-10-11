#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include <ctime>
#include <sstream>
#include<opencv2/core/core.hpp>
#include <yaml-cpp/yaml.h>

#include<System.h>
#include "ImuTypes.h"

using namespace std;
namespace ORB_SLAM3{
    using Seconds = double;
}

void LoadImages(const string &pathToSequence, const string &rgb_csv,
                vector<string> &imageFilenames, vector<ORB_SLAM3::Seconds> &timestamps,
                vector<string> &depthFilenames,
                const string cam0_name = "rgb_0", const string depth0_name = "depth_0");
std::string paddingZeros(const std::string& number, const size_t numberOfZeros = 5);

void removeSubstring(std::string& str, const std::string& substring) {
    size_t pos;
    while ((pos = str.find(substring)) != std::string::npos) {
        str.erase(pos, substring.length());
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

int main(int argc, char *argv[])
{

    const int num_seq = 1;

    // VSLAM-LAB inputs
    string sequence_path;
    string calibration_yaml;
    string rgb_csv;
    string exp_folder;
    string exp_id{"0"};
    string settings_yaml{"orbslam3_settings.yaml"};
    bool verbose{true};

    string vocabulary{"Vocabulary/ORBvoc.txt"};
    if (argc > 1 && std::string(argv[1]) == "--help") {
        cout << "Usage: vslamlab_orbslam3_rgbd_vi sequence_path:<dir> calibration_yaml:<file> rgb_csv:<file> exp_folder:<dir>"
             << " [exp_id:<n>] [settings_yaml:<file>] [verbose:<0|1>] [vocabulary:<ORBvoc.txt>]" << endl;
        return 0;
    }

    cout << endl;
    for (int i = 0; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.find("sequence_path:") != std::string::npos) {
            removeSubstring(arg, "sequence_path:");
            sequence_path =  arg;
            std::cout << "[vslamlab_orbslam3_rgbd_vi.cpp] Path to sequence = " << sequence_path << std::endl;
            continue;
        }
        if (arg.find("calibration_yaml:") != std::string::npos) {
            removeSubstring(arg, "calibration_yaml:");
            calibration_yaml =  arg;
            std::cout << "[vslamlab_orbslam3_rgbd_vi.cpp] Path to calibration.yaml = " << calibration_yaml << std::endl;
            continue;
        }
        if (arg.find("rgb_csv:") != std::string::npos) {
            removeSubstring(arg, "rgb_csv:");
            rgb_csv =  arg;
            std::cout << "[vslamlab_orbslam3_rgbd_vi.cpp] Path to rgb_csv = " << rgb_csv << std::endl;
            continue;
        }
        if (arg.find("exp_folder:") != std::string::npos) {
            removeSubstring(arg, "exp_folder:");
            exp_folder =  arg;
            std::cout << "[vslamlab_orbslam3_rgbd_vi.cpp] Path to exp_folder = " << exp_folder << std::endl;
            continue;
        }
        if (arg.find("exp_id:") != std::string::npos) {
            removeSubstring(arg, "exp_id:");
            exp_id =  arg;
            std::cout << "[vslamlab_orbslam3_rgbd_vi.cpp] Exp id = " << exp_id << std::endl;
            continue;
        }
        if (arg.find("settings_yaml:") != std::string::npos) {
            removeSubstring(arg, "settings_yaml:");
            settings_yaml =  arg;
            std::cout << "[vslamlab_orbslam3_rgbd_vi.cpp] Path to settings_yaml = " << settings_yaml << std::endl;
            continue;
        }
        if (arg.find("verbose:") != std::string::npos) {
            removeSubstring(arg, "verbose:");
            verbose = bool(std::stoi(arg));
            std::cout << "[vslamlab_orbslam3_rgbd_vi.cpp] Activate Visualization = " << verbose << std::endl;
            continue;
        }
        if (arg.find("vocabulary:") != std::string::npos) {
            removeSubstring(arg, "vocabulary:");
            vocabulary = arg;
            std::cout << "[vslamlab_orbslam3_rgbd_vi.cpp] Path to vocabulary = " << vocabulary << std::endl;
            continue;
        }
    }

    // Retrieve paths to images
    vector<string> imageFilenames{}, depthFilenames{};
    vector<ORB_SLAM3::Seconds> timestamps{};
    
    YAML::Node settings = YAML::LoadFile(settings_yaml);
    std::string cam_name = settings["cam_rgbd"].as<std::string>();

    YAML::Node calibration = YAML::LoadFile(calibration_yaml);
    const YAML::Node& cameras = calibration["cameras"];
    std::string depth_name{};
    for (int i{0}; i < cameras.size(); ++i){
        if (cameras[i]["cam_name"].as<std::string>() == cam_name) {
            depth_name = cameras[i]["depth_name"].as<std::string>();
            break;
        }
    }

    LoadImages(sequence_path, rgb_csv, imageFilenames, timestamps, depthFilenames, cam_name, depth_name);

    size_t nImages = imageFilenames.size();

    string pathImu = sequence_path + "/" + settings["imu"].as<std::string>() + ".csv";
    vector<double> timestampsImu{};
    vector<cv::Point3f> vAcc, vGyro;
    LoadIMU(pathImu, timestampsImu, vAcc, vGyro);
    int nImu= timestampsImu.size();
    int first_imu{0};

    // Find first imu to be considered, supposing imu measurements start first
    while(timestampsImu[first_imu]<=timestamps[0])
            first_imu++;
        first_imu--; // first imu measurement to be considered
    
    // Vector for tracking time statistics
    vector<ORB_SLAM3::Seconds> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    cout << "IMU measurements in the sequence: " << vAcc.size() << endl << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabulary,calibration_yaml,settings_yaml,ORB_SLAM3::System::IMU_RGBD, verbose);

    auto processSequence = [&]() {

        // Main loop
        cv::Mat im, imD;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        // proccIm = 0;
        for(int ni = 0; ni < nImages; ni++)
        {
            // Read image from file
            im = cv::imread(imageFilenames[ni], cv::IMREAD_UNCHANGED);
            imD = cv::imread(depthFilenames[ni], cv::IMREAD_UNCHANGED);
            ORB_SLAM3::Seconds tframe = timestamps[ni];

            // Load imu measurements from previous frame
            vImuMeas.clear();
            if(ni>0){
                while(timestampsImu[first_imu]<=timestamps[ni]){
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x,vAcc[first_imu].y,vAcc[first_imu].z,
                                            vGyro[first_imu].x,vGyro[first_imu].y,vGyro[first_imu].z,
                                            timestampsImu[first_imu]));
                    first_imu++;
                }
            }

            // Pass the image to the SLAM system
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            if (vImuMeas.size() > 1)
                SLAM.TrackRGBD(im, imD, tframe, vImuMeas);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            ORB_SLAM3::Seconds ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            vTimesTrack[ni] = ttrack;

            // Wait to load the next frame
            ORB_SLAM3::Seconds T = 0.0;
            if(ni < nImages-1)
                T = timestamps[ni+1] - tframe;
            else if(ni > 0)
                T = tframe - timestamps[ni-1];

            if(ttrack < T)
                std::this_thread::sleep_for(std::chrono::duration<double>(T - ttrack));
        }

        // Stop all threads
        SLAM.Shutdown();

        // Save camera trajectory
        string resultsPath_expId = exp_folder + "/" + paddingZeros(exp_id);
        SLAM.SaveKeyFrameTrajectoryVSLAMLAB(resultsPath_expId + "_" + "KeyFrameTrajectory.csv");
    };

    // On macOS with the viewer on, the viewer takes the main thread and processSequence runs in a thread
    SLAM.Run(processSequence);

    return 0;
}

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        // Simple trim for leading/trailing whitespace, often needed in real-world CSVs
        token.erase(0, token.find_first_not_of(" \t\n\r"));
        token.erase(token.find_last_not_of(" \t\n\r") + 1);
        tokens.push_back(token);
    }
    return tokens;
}

void LoadImages(const string &pathToSequence, const string &rgb_csv,
                vector<string> &imageFilenames, vector<ORB_SLAM3::Seconds> &timestamps,
                vector<string> &depthFilenames,
                const string cam0_name, const string depth0_name)
{

    imageFilenames.clear();
    timestamps.clear();
    depthFilenames.clear();
    
    std::ifstream in(rgb_csv);
    std::string line;

    // Read and map the header row to find indices
    if (!std::getline(in, line)) return; 
    if (!line.empty() && line.back() == '\r') line.pop_back();

    std::vector<std::string> headers = split(line, ',');
    std::map<std::string, int> col_map;
    for (size_t i = 0; i < headers.size(); ++i) {
        col_map[headers[i]] = i;
    }

    // Required headers
    const std::string header_ts = "ts_" + cam0_name + " (ns)";
    const std::string header_rgb0 = "path_" + cam0_name;
    const std::string header_depth0 = "path_" + depth0_name;

    // Safely get indices
    auto get_index = [&](const std::string& key) -> int {
        return col_map[key];
    };

    int ts_idx = get_index(header_ts);
    int rgb0_idx = get_index(header_rgb0);
    int depth0_idx = get_index(header_depth0);
   
    // Read and process data lines using fixed indices
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (!line.empty() && line.back() == '\r') line.pop_back();

        std::vector<std::string> tokens = split(line, ',');
        
        // Assign variables using indices, regardless of column order
        std::string t_str = tokens[ts_idx];
        std::string rel_rgb0_path = tokens[rgb0_idx];
        std::string rel_depth0_path = tokens[depth0_idx];

        ORB_SLAM3::Seconds t = static_cast<double>(std::stoll(t_str)) * 1e-9;

        timestamps.push_back(t);
        imageFilenames.push_back(pathToSequence + "/" + rel_rgb0_path);
        depthFilenames.push_back(pathToSequence + "/" + rel_depth0_path);
    }
}

void LoadIMU(const string &imu_csv, vector<ORB_SLAM3::Seconds> &timestamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{   
    
    timestamps.clear();
    vAcc.clear();
    vGyro.clear();
    
    std::ifstream in(imu_csv);
    std::string line;

    // Read and map the header row to find indices
    if (!std::getline(in, line)) return; 
    if (!line.empty() && line.back() == '\r') line.pop_back();

    std::vector<std::string> headers = split(line, ',');
    std::map<std::string, int> col_map;
    for (size_t i = 0; i < headers.size(); ++i) {
        col_map[headers[i]] = i;
    }

    // Required headers
    const std::string header_ts = "timestamp (ns)";
    const std::string header_wx= "wx (rad s^-1)";
    const std::string header_wy= "wy (rad s^-1)";
    const std::string header_wz= "wz (rad s^-1)";
    const std::string header_ax= "ax (m s^-2)";
    const std::string header_ay= "ay (m s^-2)";
    const std::string header_az= "az (m s^-2)";

    // Safely get indices
    auto get_index = [&](const std::string& key) -> int {
        return col_map[key];
    };

    int ts_idx = get_index(header_ts);
    int wx_idx = get_index(header_wx);   
    int wy_idx = get_index(header_wy);   
    int wz_idx = get_index(header_wz);     
    int ax_idx = get_index(header_ax);   
    int ay_idx = get_index(header_ay);   
    int az_idx = get_index(header_az);

    // Read and process data lines using fixed indices
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (!line.empty() && line.back() == '\r') line.pop_back();

        std::vector<std::string> tokens = split(line, ',');
        
        // Assign variables using indices, regardless of column order
        std::string t_str = tokens[ts_idx];
        ORB_SLAM3::Seconds t = static_cast<double>(std::stoll(t_str)) * 1e-9;
        double wx = std::stod(tokens[wx_idx]);
        double wy = std::stod(tokens[wy_idx]);
        double wz = std::stod(tokens[wz_idx]);
        double ax = std::stod(tokens[ax_idx]);
        double ay = std::stod(tokens[ay_idx]);
        double az = std::stod(tokens[az_idx]);

        timestamps.push_back(t);
        vAcc.push_back(cv::Point3f(ax,ay,az));
        vGyro.push_back(cv::Point3f(wx,wy,wz));
    }
}

std::string paddingZeros(const std::string& number, const size_t numberOfZeros){
    std::string zeros{};
    for(size_t iZero{}; iZero < numberOfZeros - number.size(); ++iZero)
        zeros += "0";
    return (zeros + number);
}