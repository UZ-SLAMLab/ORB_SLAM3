/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <memory>

#include <opencv2/core/core.hpp>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

// Phase-0 instrumentation (see flexiv_gdk's PARALLEL_VIO_PLAN.md): prints
// "[timing] stage=<stage> seconds=<s>" with the time since t0, one
// machine-readable line per stage, parsed by 01_run_orbslam3_vio.py.
static void PrintStageTime(const string &stage, const std::chrono::steady_clock::time_point &t0)
{
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    cout << "[timing] stage=" << stage << " seconds=" << fixed << setprecision(3) << s << defaultfloat << endl;
}


// Decodes a sequence's stereo pairs on a background thread, up to maxQueued
// frames ahead of tracking, so cv::imread's PNG decoding (~8.5ms/frame at our
// resolution) overlaps with tracking instead of adding to every frame period.
class StereoImagePrefetcher
{
public:
    StereoImagePrefetcher(const vector<string> &vstrLeft, const vector<string> &vstrRight, size_t maxQueued)
        : mvstrLeft(vstrLeft), mvstrRight(vstrRight), mMaxQueued(maxQueued),
          mThread(&StereoImagePrefetcher::Run, this)
    {
    }

    ~StereoImagePrefetcher()
    {
        {
            lock_guard<mutex> lock(mMutex);
            mbStop = true;
        }
        mCond.notify_all();
        mThread.join();
    }

    // Returns the next frame's pair, in order, blocking until it's decoded.
    // An image is empty if it failed to load.
    void Pop(cv::Mat &imLeft, cv::Mat &imRight)
    {
        unique_lock<mutex> lock(mMutex);
        mCond.wait(lock, [this]{ return !mQueue.empty(); });
        imLeft = mQueue.front().first;
        imRight = mQueue.front().second;
        mQueue.pop_front();
        mCond.notify_all();
    }

    // Total time the background thread spent decoding images so far.
    double DecodeSeconds()
    {
        lock_guard<mutex> lock(mMutex);
        return mDecodeS;
    }

private:
    void Run()
    {
        for (size_t i = 0; i < mvstrLeft.size(); i++)
        {
            {
                unique_lock<mutex> lock(mMutex);
                mCond.wait(lock, [this]{ return mbStop || mQueue.size() < mMaxQueued; });
                if (mbStop)
                    return;
            }
            const auto t0 = std::chrono::steady_clock::now();
            cv::Mat imLeft = cv::imread(mvstrLeft[i], cv::IMREAD_UNCHANGED);
            cv::Mat imRight = cv::imread(mvstrRight[i], cv::IMREAD_UNCHANGED);
            const double decodeS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            {
                lock_guard<mutex> lock(mMutex);
                mQueue.emplace_back(imLeft, imRight);
                mDecodeS += decodeS;
            }
            mCond.notify_all();
        }
    }

    const vector<string> &mvstrLeft;
    const vector<string> &mvstrRight;
    const size_t mMaxQueued;
    mutex mMutex;
    condition_variable mCond;
    deque<pair<cv::Mat, cv::Mat>> mQueue;
    bool mbStop = false;
    double mDecodeS = 0;
    // Last, so every member above is initialized before Run() starts.
    thread mThread;
};

int main(int argc, char **argv)
{
    const auto tMain = std::chrono::steady_clock::now();
    // Look for optional "--no-viewer"/"--viewer-auto-exit" flags anywhere in
    // the arguments and strip them out before the positional parsing below,
    // so argc/argv indexing for vocabulary/settings/sequence/times/session
    // stays exactly as it was regardless of where the flags appear.
    // - The viewer is opened (bUseViewer=true) by default; pass --no-viewer
    //   to disable it, e.g. for unattended/batch runs where nothing will
    //   close the window.
    // - Normally, once the viewer is open, the process waits for it to be
    //   manually closed before exiting (see the isViewerFinished() loop
    //   below), so an operator can linger and inspect the reconstructed
    //   map/trajectory. --viewer-auto-exit skips that wait: the window still
    //   shows live progress, but the process exits as soon as processing
    //   finishes instead of blocking on a manual close.
    // - --save-atlas <file> saves the resulting atlas to <file>.osa on
    //   shutdown (overrides System.SaveAtlasToFile in the settings file), so
    //   it can later be reloaded via System.LoadAtlasFromFile or --load-atlas.
    // - --load-atlas <file> loads the atlas <file>.osa at startup (overrides
    //   System.LoadAtlasFromFile). SLAM relocalizes in the loaded map, then
    //   keeps tracking and mapping in it (full SLAM, not localization mode).
    // - --prefetch-images decodes images on a background thread ahead of
    //   tracking (see StereoImagePrefetcher). Off by default: it makes a run
    //   alone ~15% faster, but with several runs at once on the same machine
    //   it slowed tracking down and delayed relocalization into a loaded
    //   atlas (see flexiv_gdk's PARALLEL_VIO_PLAN.md).
    bool bUseViewer = true;
    bool bPrefetchImages = false;
    bool bAutoExitViewer = false;
    string strSaveAtlasFile;
    string strLoadAtlasFile;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        int nConsumed = 0;
        if (arg == "--no-viewer")
        {
            bUseViewer = false;
            nConsumed = 1;
        }
        else if (arg == "--viewer-auto-exit")
        {
            bAutoExitViewer = true;
            nConsumed = 1;
        }
        else if (arg == "--prefetch-images")
        {
            bPrefetchImages = true;
            nConsumed = 1;
        }
        else if (arg == "--save-atlas" || arg == "--load-atlas")
        {
            if (i + 1 >= argc)
            {
                cerr << "ERROR: " << arg << " requires a file name" << endl;
                return 1;
            }
            if (arg == "--save-atlas")
                strSaveAtlasFile = argv[i + 1];
            else
                strLoadAtlasFile = argv[i + 1];
            nConsumed = 2;
        }
        if (nConsumed > 0)
        {
            for (int j = i; j < argc - nConsumed; j++)
                argv[j] = argv[j + nConsumed];
            argc -= nConsumed;
            i--;
        }
    }

    if(argc < 5)
    {
        cerr << endl << "Usage: ./stereo_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) [session_name] [--no-viewer] [--viewer-auto-exit] [--prefetch-images] [--save-atlas atlas_file] [--load-atlas atlas_file]" << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    const auto tLoadSeq = std::chrono::steady_clock::now();
    int seq;
    vector< vector<string> > vstrImageLeft;
    vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathCam1 = pathSeq + "/mav0/cam1/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";

        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        // first imu measurement to be considered; stays at 0 if the IMU
        // starts after the first frame
        first_imu[seq] = max(0, first_imu[seq] - 1);
    }

    PrintStageTime("load_sequence", tLoadSeq);

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const auto tInit = std::chrono::steady_clock::now();
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, bUseViewer, 0, string(), strLoadAtlasFile);
    if (!strSaveAtlasFile.empty())
        SLAM.SetSaveAtlasFile(strSaveAtlasFile);
    PrintStageTime("system_init", tInit);

    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++)
    {
        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        double t_rect = 0.f;
        double t_resize = 0.f;
        double t_track = 0.f;
        int num_rect = 0;
        int proccIm = 0;
        // Real-time pacing stats: a frame "overruns" when tracking it took
        // longer than the gap to the next frame, so this run fell further
        // behind real time; max_overrun_ms is the worst single such frame.
        const auto tTracking = std::chrono::steady_clock::now();
        vector<double> vTrackS;
        vTrackS.reserve(nImages[seq]);
        int nOverruns = 0;
        double maxOverrunS = 0;
        double imageWaitS = 0;
        unique_ptr<StereoImagePrefetcher> prefetcher;
        if (bPrefetchImages)
            prefetcher.reset(new StereoImagePrefetcher(vstrImageLeft[seq], vstrImageRight[seq], 8));
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read left and right images, from file or already decoded in the
            // background
            const auto tRead = std::chrono::steady_clock::now();
            if (prefetcher)
                prefetcher->Pop(imLeft, imRight);
            else
            {
                imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
                imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);
            }

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }

            if(imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            imageWaitS += std::chrono::duration<double>(std::chrono::steady_clock::now() - tRead).count();

            double tframe = vTimestampsCam[seq][ni];

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_rect + t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            vTrackS.push_back(ttrack);

            // Wait until the next frame is due: its timestamp gap after the
            // moment this frame was handed to tracking (t1), same as the
            // upstream usleep(T - ttrack). Frames are never fed closer
            // together than they were recorded, even after falling behind.
            // With --prefetch-images, image loading no longer adds to the
            // frame period.
            if(ni<nImages[seq]-1)
            {
                const auto tNext = t1 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(vTimestampsCam[seq][ni+1]-tframe));
                const auto tNow = std::chrono::steady_clock::now();
                if(tNow < tNext)
                    std::this_thread::sleep_until(tNext);
                else
                {
                    nOverruns++;
                    maxOverrunS = max(maxOverrunS, std::chrono::duration<double>(tNow - tNext).count());
                }
            }
        }

        PrintStageTime("tracking", tTracking);
        if(!vTrackS.empty())
        {
            const double sequenceS = vTimestampsCam[seq].back() - vTimestampsCam[seq].front();
            sort(vTrackS.begin(), vTrackS.end());
            cout << "[timing] stats seq=" << seq << " frames=" << vTrackS.size() << " overruns=" << nOverruns
                 << fixed << setprecision(3)
                 << " max_overrun_ms=" << 1e3*maxOverrunS
                 << " median_track_ms=" << 1e3*vTrackS[vTrackS.size()/2]
                 << " max_track_ms=" << 1e3*vTrackS.back()
                 << " imread_s=" << (prefetcher ? prefetcher->DecodeSeconds() : imageWaitS)
                 << " image_wait_s=" << imageWaitS
                 << " sequence_s=" << sequenceS
                 << defaultfloat << endl;
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }


    }
    // Stop all threads. Must happen before any SaveTrajectory* call below --
    // LocalMapping/LoopClosing keep mutating the map/keyframes until asked to
    // stop, and reading that data concurrently with an in-flight loop closure
    // or bundle adjustment corrupts the heap. Also saves the atlas if
    // --save-atlas (or System.SaveAtlasToFile) was given.
    const auto tShutdown = std::chrono::steady_clock::now();
    SLAM.Shutdown();
    PrintStageTime("shutdown", tShutdown);

    // Save camera trajectory. We skip SaveKeyFrameTrajectoryEuRoC here: it's
    // unused by the caller (00_run_offline_slam.py only reads f_<session>.txt)
    // and can crash or hang indefinitely (System::Shutdown() above only
    // requests LocalMapping/LoopClosing finish, it does not wait for them --
    // that wait loop is commented out upstream -- so this call can race a
    // virtual call against one of those threads still tearing down).
    const auto tSaveTraj = std::chrono::steady_clock::now();
    if (bFileName)
    {
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    }
    PrintStageTime("trajectory_save", tSaveTraj);
    PrintStageTime("total", tMain);

    // Keep the process (and its viewer -- Shutdown() above does not close it)
    // alive so the operator can keep inspecting the reconstructed
    // map/trajectory, and exit once they close the viewer window. Skipped
    // entirely with --viewer-auto-exit: the viewer still showed live
    // progress, but nothing here waits for it to be closed.
    while (!bAutoExitViewer && !SLAM.isViewerFinished())
    {
        usleep(500000);
    }
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    if(!fTimes.is_open())
    {
        // eof() never becomes true on a stream that failed to open, so the
        // loop below would spin forever
        cerr << endl << "ERROR: Failed to open times file: " << strPathTimes << endl;
        return;
    }
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    if(!fImu.is_open())
    {
        cerr << endl << "ERROR: Failed to open IMU file: " << strImuPath << endl;
        return;
    }
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
