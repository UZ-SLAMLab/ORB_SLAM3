#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<ctime>
#include<sstream>
#include<MPU6050.h>
#include<math.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<System.h>
#include "ImuTypes.h"
#include <thread>

using namespace std;
using namespace cv;

//std::mutex imu_mutex;

vector<ORB_SLAM3::IMU::Point> vImuMeas;
float ax, ay, az, gr, gp, gy;

void getImuData()
{
    //imu_mutex.lock();
    MPU6050 mpu6050(0x68);
    mpu6050.calc_yaw = true;
    //init imu
    while(true)
    {
        mpu6050.getAccelRaw(&ax, &ay, &az);
        mpu6050.getGyroRaw(&gr, &gp, &gy);
        vImuMeas.push_back(
            ORB_SLAM3::IMU::Point(
                ax, ay, az,
                (gr/360*2*M_PI), (gp/360*2*M_PI), (gy/360*2*M_PI),
                std::chrono::system_clock::now().time_since_epoch().count()/1e9
            )
        );
    }

    //imu_mutex.unlock();
}



int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    std::thread imu(getImuData);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true); //IMU_MONOCULAR MONOCULAR

    //std::mutex imu_mutex;

    //init camera
    cv::Mat im;
    cv::VideoCapture cap(13);



    double timestamp = 0;
    if(!cap.isOpened()){
        cout << "No video stream detected" << endl;
        system("pause");
        return -1;
    }

    //DUMP

    std::chrono::milliseconds timespan(1);

    while(true){//!SLAM.isShutDown()){
        try
        {
            while(vImuMeas.size() < 10)
            {
                std::this_thread::sleep_for(timespan);
            }

            cap >> im;
            timestamp = std::chrono::system_clock::now().time_since_epoch().count()/1e9;

            // printf("ax: %f\tay: %f\taz: %f\n",ax,ay,az);
            // printf("gr: %f\tgp: %f\tgy: %f\n",(gr/360*M_PI),(ay/360*M_PI),(az/360*M_PI));

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im, timestamp, vImuMeas);
            vImuMeas.clear();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            cap.release();
            SLAM.Shutdown();
            break;
        }
    }


    cap.release();
    SLAM.Shutdown();

    return 0;

    //DUMP

    /*std::mutex imu_mutex;

    int count_im_buffer = 0;
    bool image_ready = false;



















    while(!SLAM.isShutDown()){
        std::vector<float> vGyro;
        std::vector<double> vGyro_times;
        std::vector<float> vAccel;








        vGyro = ;
        vGyro_times = ;
        vAccel = ;

        for(int i=0; i<vGyro.size(); ++i){
            ORB_SLAM3::IMU::Point lastPoint(vAccel[i].x, vAccel[i].y, vAccel[i].z,
                                            vGyro[i].x, vGyro[i].y, vGyro[i].z,
                                            vGyro_times[i]);
            vImuMeas.push_back(lastPoint);

            if(isnan(vAccel[i].x) || isnan(vAccel[i].y) || isnan(vAccel[i].z) ||
               isnan(vGyro[i].x) || isnan(vGyro[i].y) || isnan(vGyro[i].z) ||
               isnan(vGyro_times[i])){
                //cerr << "NAN AT MAIN" << endl;
                exit(-1);
            }
        }

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, timestamp, vImuMeas);

        // Clear the previous IMU measurements to load the new ones
        vImuMeas.clear();
    }*/

    // cap.release();
    // //SLAM.Shutdown();

    // return 0;

    
    //vector<cv::Point3f> vAcc, vGyro;
    //vector<ORB_SLAM3::IMU::Point> vImuMeas;

    /*#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point initT = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point initT = std::chrono::monotonic_clock::now();
    #endif
    
    // Main loop
    while(true){
        cap >> im;
        if(im.empty())
            break;
        
        //vImuTimeStamp
        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point nowT_imu = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point nowT_imu = std::chrono::monotonic_clock::now();
        #endif

        //get imu data
        //mpu6050.getAccel(&data[4], &data[5], &data[6]);
        //mpu6050.getGyro(&data[1], &data[2], &data[3]);
        
        //clear prev vAcc, vGyro data
        vAcc.clear();
        vGyro.clear();

        //send imu data to vAcc, vGyro;
        vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
        vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        
        // Load imu measurements from previous frame
        vImuMeas.clear();
        
        //send vAcc, vGyro data to vImuMeas
        vImuMeas.push_back(
            ORB_SLAM3::IMU::Point(
            vAcc[0].x, vAcc[0].y,vAcc[0].z,
            vGyro[0].x, vGyro[0].y, vGyro[0].z,
            std::chrono::duration_cast<std::chrono::duration<double> >(nowT_imu-initT).count()
            )
        );
        cout << "vAcc[0].x : " << vAcc[0].x << " vAcc[0].y : " << vAcc[0].y << " vAcc[0].z : " << vAcc[0].z << endl;
        cout << "vGyro[0].x: " << vGyro[0].x << " vGyro[0].y: " << vGyro[0].y << " vGyro[0].z: " << vGyro[0].z << endl;
        cout << endl;
        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point nowT = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point nowT = std::chrono::monotonic_clock::now();
        #endif

        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif

        //SLAM.TrackMonocular(im, std::chrono::duration_cast<std::chrono::duration<double> >(nowT-initT).count(), vImuMeas);

        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif

        #ifdef REGISTER_TIMES
            t_track =  std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            //SLAM.InsertTrackTime(t_track);
        #endif
    }*/
}
