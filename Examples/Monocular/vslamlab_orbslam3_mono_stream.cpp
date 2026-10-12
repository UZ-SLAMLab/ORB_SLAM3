// Online monocular ORB-SLAM3 on an ARStream relay (github.com/VSLAM-LAB/arstream, `arstream publish`): frames
// arrive over ZeroMQ with their real timestamps and are tracked as they come; a subscriber slower than the stream
// keeps the newest frame. Stops at the relay's end-of-stream message or on Ctrl-C, then saves the keyframe
// trajectory like the offline entry points.
//
//   vslamlab_orbslam3_mono_stream stream_url:tcp://localhost:5556 calibration_yaml:<file> exp_folder:<dir>
//                                 [exp_id:<n>] [settings_yaml:<file>] [verbose:<0|1>] [vocabulary:<ORBvoc.txt>]
#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <vector>

#include <arstream/arstream.hpp>
#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;
namespace ORB_SLAM3 {
    using Seconds = double;
}

static arstream::Subscriber* g_subscriber = nullptr;
static void onSignal(int) { if (g_subscriber) g_subscriber->stop(); }

static std::string paddingZeros(const std::string& number, const size_t numberOfZeros = 5) {
    std::string zeros{};
    for (size_t iZero{}; iZero < numberOfZeros - number.size(); ++iZero)
        zeros += "0";
    return zeros + number;
}

static void removeSubstring(std::string& str, const std::string& substring) {
    size_t pos;
    while ((pos = str.find(substring)) != std::string::npos)
        str.erase(pos, substring.length());
}

int main(int argc, char** argv) {

    // VSLAM-LAB inputs
    string stream_url{arstream::kDefaultUrl};
    string calibration_yaml;
    string exp_folder;
    string exp_id{"0"};
    string settings_yaml{"orbslam3_settings.yaml"};
    bool verbose{true};
    string vocabulary{"Vocabulary/ORBvoc.txt"};

    if (argc > 1 && std::string(argv[1]) == "--help") {
        cout << "Usage: vslamlab_orbslam3_mono_stream stream_url:<tcp://host:port> calibration_yaml:<file> exp_folder:<dir>"
             << " [exp_id:<n>] [settings_yaml:<file>] [verbose:<0|1>] [vocabulary:<ORBvoc.txt>]" << endl;
        return 0;
    }

    cout << endl;
    for (int i = 0; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.find("stream_url:") != std::string::npos) {
            removeSubstring(arg, "stream_url:");
            stream_url = arg;
            std::cout << "[vslamlab_orbslam3_mono_stream.cpp] Stream url = " << stream_url << std::endl;
            continue;
        }
        if (arg.find("calibration_yaml:") != std::string::npos) {
            removeSubstring(arg, "calibration_yaml:");
            calibration_yaml = arg;
            std::cout << "[vslamlab_orbslam3_mono_stream.cpp] Path to calibration.yaml = " << calibration_yaml << std::endl;
            continue;
        }
        if (arg.find("exp_folder:") != std::string::npos) {
            removeSubstring(arg, "exp_folder:");
            exp_folder = arg;
            std::cout << "[vslamlab_orbslam3_mono_stream.cpp] Path to exp_folder = " << exp_folder << std::endl;
            continue;
        }
        if (arg.find("exp_id:") != std::string::npos) {
            removeSubstring(arg, "exp_id:");
            exp_id = arg;
            std::cout << "[vslamlab_orbslam3_mono_stream.cpp] Exp id = " << exp_id << std::endl;
            continue;
        }
        if (arg.find("settings_yaml:") != std::string::npos) {
            removeSubstring(arg, "settings_yaml:");
            settings_yaml = arg;
            std::cout << "[vslamlab_orbslam3_mono_stream.cpp] Path to settings_yaml = " << settings_yaml << std::endl;
            continue;
        }
        if (arg.find("verbose:") != std::string::npos) {
            removeSubstring(arg, "verbose:");
            verbose = bool(std::stoi(arg));
            std::cout << "[vslamlab_orbslam3_mono_stream.cpp] Activate Visualization = " << verbose << std::endl;
            continue;
        }
        if (arg.find("vocabulary:") != std::string::npos) {
            removeSubstring(arg, "vocabulary:");
            vocabulary = arg;
            std::cout << "[vslamlab_orbslam3_mono_stream.cpp] Path to vocabulary = " << vocabulary << std::endl;
            continue;
        }
    }
    if (calibration_yaml.empty() || exp_folder.empty()) {
        cerr << "[vslamlab_orbslam3_mono_stream.cpp] calibration_yaml: and exp_folder: are required (--help)" << endl;
        return 1;
    }

    // Subscribe before the SLAM system starts so no frame published meanwhile is missed (conflate keeps the newest)
    arstream::Subscriber subscriber(stream_url);
    g_subscriber = &subscriber;
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabulary, calibration_yaml, settings_yaml, ORB_SLAM3::System::MONOCULAR, verbose);

    auto processStream = [&]() {
        vector<ORB_SLAM3::Seconds> vTimesTrack;

        cout << endl << "-------" << endl;
        cout << "Subscribed to " << stream_url << ", waiting for frames (Ctrl-C or the relay's end of stream stops) ..." << endl << endl;

        arstream::Frame frame;
        size_t received = 0;
        while (subscriber.next(frame)) {
            ++received;
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            SLAM.TrackMonocular(frame.image, frame.timestamp);
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            vTimesTrack.push_back(std::chrono::duration_cast<std::chrono::duration<ORB_SLAM3::Seconds>>(t2 - t1).count());
        }
        cout << (subscriber.ended() ? "End of stream" : "Stopped") << " after " << received << " frames" << endl;

        // Stop all threads
        SLAM.Shutdown();

        // Tracking time statistics
        if (!vTimesTrack.empty()) {
            sort(vTimesTrack.begin(), vTimesTrack.end());
            ORB_SLAM3::Seconds totaltime = 0.0;
            for (const auto t : vTimesTrack)
                totaltime += t;
            cout << "-------" << endl << endl;
            cout << "median tracking time: " << vTimesTrack[vTimesTrack.size() / 2] << endl;
            cout << "mean tracking time: " << totaltime / vTimesTrack.size() << endl;
        }

        // Save camera trajectory
        string resultsPath_expId = exp_folder + "/" + paddingZeros(exp_id);
        SLAM.SaveKeyFrameTrajectoryVSLAMLAB(resultsPath_expId + "_" + "KeyFrameTrajectory.csv");
    };

    // On macOS with the viewer on, the viewer takes the main thread and processStream runs in a thread
    SLAM.Run(processStream);

    return 0;
}
