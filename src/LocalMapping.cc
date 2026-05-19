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


#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"
#include "GeometricTools.h"

#include<mutex>
#include<chrono>
#include<iomanip>
#include<algorithm>
#include<cmath>
#include<map>
#include<set>

namespace ORB_SLAM3
{
namespace
{
const char* TrackingStateNameForLog(Tracking::eTrackingState s)
{
    switch(s)
    {
        case Tracking::SYSTEM_NOT_READY: return "SYSTEM_NOT_READY";
        case Tracking::NO_IMAGES_YET: return "NO_IMAGES_YET";
        case Tracking::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case Tracking::OK: return "OK";
        case Tracking::RECENTLY_LOST: return "RECENTLY_LOST";
        case Tracking::LOST: return "LOST";
        case Tracking::OK_KLT: return "OK_KLT";
        default: return "UNKNOWN";
    }
}

struct KFCorrectionSnapshot
{
    Sophus::SE3f Tcw;
    Eigen::Vector3f velocity;
    IMU::Bias bias;
    bool has_velocity = false;
    bool has_bias = false;
};

double BiasDeltaNorm(const IMU::Bias &before, const IMU::Bias &after)
{
    const double dbax = static_cast<double>(after.bax - before.bax);
    const double dbay = static_cast<double>(after.bay - before.bay);
    const double dbaz = static_cast<double>(after.baz - before.baz);
    const double dbwx = static_cast<double>(after.bwx - before.bwx);
    const double dbwy = static_cast<double>(after.bwy - before.bwy);
    const double dbwz = static_cast<double>(after.bwz - before.bwz);
    return std::sqrt(dbax*dbax + dbay*dbay + dbaz*dbaz +
                     dbwx*dbwx + dbwy*dbwy + dbwz*dbwz);
}

double RotationDeltaDeg(const Sophus::SE3f &before, const Sophus::SE3f &after)
{
    const Eigen::Matrix3f dR = after.rotationMatrix() * before.rotationMatrix().transpose();
    double cos_angle = (static_cast<double>(dR.trace()) - 1.0) * 0.5;
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    return std::acos(cos_angle) * 180.0 / 3.14159265358979323846;
}

std::vector<KeyFrame*> CollectCorrectionKeyFrames(KeyFrame* pKF, bool bInertialBA, bool bLarge)
{
    std::vector<KeyFrame*> out;
    std::set<KeyFrame*> seen;
    if(!pKF)
        return out;

    Map* pMap = pKF->GetMap();
    if(!pMap)
        return out;
    auto addKF = [&](KeyFrame* pKFi)
    {
        if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pMap || seen.count(pKFi))
            return;
        seen.insert(pKFi);
        out.push_back(pKFi);
    };

    addKF(pKF);

    if(bInertialBA)
    {
        const int maxOpt = bLarge ? 25 : 10;
        const int Nd = std::min(static_cast<int>(pMap->KeyFramesInMap()) - 2, maxOpt);
        KeyFrame* pBack = pKF;
        for(int i = 1; i < Nd && pBack && pBack->mPrevKF; ++i)
        {
            pBack = pBack->mPrevKF;
            addKF(pBack);
        }
    }
    else
    {
        const std::vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
        for(size_t i = 0, iend = vNeighKFs.size(); i < iend; ++i)
            addKF(vNeighKFs[i]);
    }

    return out;
}

std::map<long unsigned int, KFCorrectionSnapshot> SnapshotKeyFrames(const std::vector<KeyFrame*> &vpKFs)
{
    std::map<long unsigned int, KFCorrectionSnapshot> snapshots;
    for(size_t i = 0, iend = vpKFs.size(); i < iend; ++i)
    {
        KeyFrame* pKF = vpKFs[i];
        if(!pKF || pKF->isBad())
            continue;

        KFCorrectionSnapshot snap;
        snap.Tcw = pKF->GetPose();
        snap.has_velocity = pKF->isVelocitySet();
        if(snap.has_velocity)
            snap.velocity = pKF->GetVelocity();
        snap.has_bias = pKF->bImu;
        if(snap.has_bias)
            snap.bias = pKF->GetImuBias();
        snapshots[pKF->mnId] = snap;
    }
    return snapshots;
}

void ComputeCorrectionStats(const std::map<long unsigned int, KFCorrectionSnapshot> &before,
                            const std::vector<KeyFrame*> &vpKFs,
                            double &bias_delta_norm,
                            double &velocity_delta_norm,
                            double &pose_correction_trans_norm,
                            double &pose_correction_rot_deg)
{
    bias_delta_norm = -1.0;
    velocity_delta_norm = -1.0;
    pose_correction_trans_norm = -1.0;
    pose_correction_rot_deg = -1.0;

    for(size_t i = 0, iend = vpKFs.size(); i < iend; ++i)
    {
        KeyFrame* pKF = vpKFs[i];
        if(!pKF || pKF->isBad())
            continue;

        std::map<long unsigned int, KFCorrectionSnapshot>::const_iterator it = before.find(pKF->mnId);
        if(it == before.end())
            continue;

        const KFCorrectionSnapshot &snap = it->second;
        const Sophus::SE3f TcwAfter = pKF->GetPose();
        const double trans_norm = static_cast<double>((TcwAfter.translation() - snap.Tcw.translation()).norm());
        const double rot_deg = RotationDeltaDeg(snap.Tcw, TcwAfter);
        pose_correction_trans_norm = std::max(pose_correction_trans_norm, trans_norm);
        pose_correction_rot_deg = std::max(pose_correction_rot_deg, rot_deg);

        if(snap.has_velocity && pKF->isVelocitySet())
        {
            const double vel_norm = static_cast<double>((pKF->GetVelocity() - snap.velocity).norm());
            velocity_delta_norm = std::max(velocity_delta_norm, vel_norm);
        }

        if(snap.has_bias && pKF->bImu)
        {
            const double bias_norm = BiasDeltaNorm(snap.bias, pKF->GetImuBias());
            bias_delta_norm = std::max(bias_delta_norm, bias_norm);
        }
    }
}
}

LocalMapping::LocalMapping(System* pSys, Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName):
    mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial), mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), bInitializing(false),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
    mIdxInit(0), mScale(1.0), mInitSect(0), mbNotBA1(true), mbNotBA2(true), mIdxIteration(0), infoInertial(Eigen::MatrixXd::Zero(9,9))
{
    mnMatchesInliers = 0;

    mbBadImu = false;

    mTinit = 0.f;

    mNumLM = 0;
    mNumKFCulling=0;

#ifdef REGISTER_TIMES
    nLBA_exec = 0;
    nLBA_abort = 0;
#endif

    // Open per-iteration CSV log
    mLMIteration     = 0;
    mCsvKFCulled     = 0;
    mCsvMeanReprojErr = 0.0;
    mCsvMaxReprojErr  = 0.0;
    mCsvOptimMs       = 0.0;
    mCsvBADone        = 0;
    mCsvBAAborted     = 0;
    mCsvBAIters       = 0;
    mCsvFirstTimestamp = -1.0;
    f_lm_csv.open("localmapping_log.csv");
    f_lm_csv << "localmappingnumber,kf_id,track_frame_id,timestamp_ms,"
                "kf_insertion_ms,"
                "map_points_total,new_map_points,queue_length,kf_culled,"
                "mean_reprojection_error,max_reprojection_error,"
                "optimization_time_ms,ba_executed,ba_aborted,ba_iters,tracking_lost,"
                "source,event_type,timestamp,rel_time_s,map_id,tracking_state,imu_initialized,"
                "ba_type,num_opt_kf,num_fixed_kf,num_mappoints,num_edges,"
                "visual_chi2_before,visual_chi2_after,imu_chi2_before,imu_chi2_after,"
                "imu_chi2_mean,imu_chi2_max,imu_rot_res_mean,imu_vel_res_mean,imu_pos_res_mean,"
                "gyro_rw_chi2,acc_rw_chi2,bias_delta_norm,velocity_delta_norm,"
                "pose_correction_trans_norm,pose_correction_rot_deg,"
                "init_scale,init_bg_norm,init_ba_norm,init_prior_g,init_prior_a,init_full_ba\n";
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{
    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames() && !mbBadImu)
        {
#ifdef REGISTER_TIMES
            double timeLBA_ms = 0;
            double timeKFCulling_ms = 0;

            std::chrono::steady_clock::time_point time_StartProcessKF = std::chrono::steady_clock::now();
#endif
            // BoW conversion and insertion in Map
            std::chrono::steady_clock::time_point csv_t0 = std::chrono::steady_clock::now();
            ProcessNewKeyFrame();
            double csvKFInsert_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(
                std::chrono::steady_clock::now() - csv_t0).count();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndProcessKF = std::chrono::steady_clock::now();

            double timeProcessKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndProcessKF - time_StartProcessKF).count();
            vdKFInsert_ms.push_back(timeProcessKF);
#endif

            // Check recent MapPoints
            MapPointCulling();//開關
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCulling = std::chrono::steady_clock::now();

            double timeMPCulling = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCulling - time_EndProcessKF).count();
            vdMPCulling_ms.push_back(timeMPCulling);
#endif

            // Triangulate new MapPoints
            CreateNewMapPoints();

            mbAbortBA = false;

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCreation = std::chrono::steady_clock::now();

            double timeMPCreation = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCreation - time_EndMPCulling).count();
            vdMPCreation_ms.push_back(timeMPCreation);
#endif

            bool b_doneLBA = false;
            int num_FixedKF_BA = 0;
            int num_OptKF_BA = 0;
            int num_MPs_BA = 0;
            int num_edges_BA = 0;
            string csvBAType = "none";
            Optimizer::InertialResidualStats csvBAStatsBefore;
            Optimizer::InertialResidualStats csvBAStatsAfter;
            std::vector<KeyFrame*> csvCorrectionKFs;
            std::map<long unsigned int, KFCorrectionSnapshot> csvCorrectionBefore;
            double csvBiasDeltaNorm = -1.0;
            double csvVelocityDeltaNorm = -1.0;
            double csvPoseCorrectionTransNorm = -1.0;
            double csvPoseCorrectionRotDeg = -1.0;
            mCsvOptimMs       = 0.0;
            mCsvMeanReprojErr = 0.0;
            mCsvMaxReprojErr  = 0.0;
            mCsvBADone        = 0;
            mCsvBAAborted     = 0;
            mCsvBAIters       = 0;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                if(mpAtlas->KeyFramesInMap()>2)
                {
                    if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
                    {
                        float dist = (mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()).norm() +
                                (mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter()).norm();

                        if(dist>0.05)
                            mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                        {
                            if((mTinit<10.f) && (dist<0.02))
                            {
                                cout << "Not enough motion for initializing. Reseting..." << endl;
                                unique_lock<mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap = true;
                                mpMapToReset = mpCurrentKeyFrame->GetMap();
                                mbBadImu = true;
                            }
                        }

                        bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);
                        csvBAType = "LocalInertialBA";
                        csvCorrectionKFs = CollectCorrectionKeyFrames(mpCurrentKeyFrame, true, bLarge);
                        csvCorrectionBefore = SnapshotKeyFrames(csvCorrectionKFs);
                        auto csv_ba_t0 = std::chrono::steady_clock::now();
                        Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, mCsvBAIters, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2(), &csvBAStatsAfter, &csvBAStatsBefore);
                        mCsvOptimMs = std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(
                            std::chrono::steady_clock::now() - csv_ba_t0).count();
                        b_doneLBA = true;
                    }
                    else
                    {
                        csvBAType = "LocalBundleAdjustment";
                        csvCorrectionKFs = CollectCorrectionKeyFrames(mpCurrentKeyFrame, false, false);
                        csvCorrectionBefore = SnapshotKeyFrames(csvCorrectionKFs);
                        auto csv_ba_t0 = std::chrono::steady_clock::now();
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, mCsvBAIters, &csvBAStatsAfter, &csvBAStatsBefore);
                        mCsvOptimMs = std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(
                            std::chrono::steady_clock::now() - csv_ba_t0).count();
                        b_doneLBA = true;
                    }

                    mCsvBADone    = b_doneLBA ? 1 : 0;
                    mCsvBAAborted = (b_doneLBA && mbAbortBA) ? 1 : 0;
                    if(b_doneLBA)
                    {
                        ComputeCorrectionStats(csvCorrectionBefore, csvCorrectionKFs,
                                               csvBiasDeltaNorm,
                                               csvVelocityDeltaNorm,
                                               csvPoseCorrectionTransNorm,
                                               csvPoseCorrectionRotDeg);
                    }

                    if(b_doneLBA && !mbAbortBA)
                        ComputeLocalWindowReprojErrors(mCsvMeanReprojErr, mCsvMaxReprojErr);
                }
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndLBA = std::chrono::steady_clock::now();

                if(b_doneLBA)
                {
                    timeLBA_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLBA - time_EndMPCreation).count();
                    vdLBA_ms.push_back(timeLBA_ms);

                    nLBA_exec += 1;
                    if(mbAbortBA)
                    {
                        nLBA_abort += 1;
                    }
                    vnLBA_edges.push_back(num_edges_BA);
                    vnLBA_KFopt.push_back(num_OptKF_BA);
                    vnLBA_KFfixed.push_back(num_FixedKF_BA);
                    vnLBA_MPs.push_back(num_MPs_BA);
                }

#endif

                // Initialize IMU here
                if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
                {
                    if (mbMonocular)
                        InitializeIMU(1e2, 1e10, true);
                    else
                        InitializeIMU(1e2, 1e5, true);
                }


                // Check redundant local Keyframes
                mCsvKFCulled = 0;
                KeyFrameCulling();//開關

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndKFCulling = std::chrono::steady_clock::now();

                timeKFCulling_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndKFCulling - time_EndLBA).count();
                vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif

                if ((mTinit<50.0f) && mbInertial)
                {
                    if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK) // Enter here everytime local-mapping is called
                    {
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
                            if (mTinit>5.0f)
                            {
                                cout << "start VIBA 1" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                                if (mbMonocular)
                                    InitializeIMU(1.f, 1e5, true);
                                else
                                    InitializeIMU(1.f, 1e5, true);

                                cout << "end VIBA 1" << endl;
                            }
                        }
                        else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
                            if (mTinit>15.0f){
                                cout << "start VIBA 2" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
                                if (mbMonocular)
                                    InitializeIMU(0.f, 0.f, true);
                                else
                                    InitializeIMU(0.f, 0.f, true);

                                cout << "end VIBA 2" << endl;
                            }
                        }

                        // scale refinement
                        if (((mpAtlas->KeyFramesInMap())<=200) &&
                                ((mTinit>25.0f && mTinit<25.5f)||
                                (mTinit>35.0f && mTinit<35.5f)||
                                (mTinit>45.0f && mTinit<45.5f)||
                                (mTinit>55.0f && mTinit<55.5f)||
                                (mTinit>65.0f && mTinit<65.5f)||
                                (mTinit>75.0f && mTinit<75.5f))){
                            if (mbMonocular)
                                ScaleRefinement();
                        }
                    }
                }
            }

#ifdef REGISTER_TIMES
            vdLBASync_ms.push_back(timeKFCulling_ms);
            vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif
//開關loopclosing的插入keyframe，讓localmapping的迴圈更快結束，方便debug localmapping的部分
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLocalMap = std::chrono::steady_clock::now();

            double timeLocalMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLocalMap - time_StartProcessKF).count();
            vdLMTotal_ms.push_back(timeLocalMap);
#endif

            // Write CSV row
            if(f_lm_csv.is_open())
            {
                auto csv_wall = std::chrono::system_clock::now();
                long long ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    csv_wall.time_since_epoch()).count();

                int queueLen;
                {
                    unique_lock<mutex> lk(mMutexNewKFs);
                    queueLen = (int)mlNewKeyFrames.size();
                }

              int csvTrackingLost = (mpTracker &&
                    (mpTracker->mState == Tracking::LOST ||
                     mpTracker->mState == Tracking::RECENTLY_LOST)) ? 1 : 0;
                const double kfTimestamp = mpCurrentKeyFrame->mTimeStamp;
                if(mCsvFirstTimestamp < 0.0)
                    mCsvFirstTimestamp = kfTimestamp;
                const double relTimeS = kfTimestamp - mCsvFirstTimestamp;
                Map* pCsvMap = mpCurrentKeyFrame->GetMap();
                const long unsigned int csvMapId = pCsvMap ? pCsvMap->GetId() : 0;
                const bool csvImuInitialized = pCsvMap && pCsvMap->isImuInitialized();
                const string csvEventType = b_doneLBA ?
                    (csvBAType == "LocalInertialBA" ? "LOCAL_INERTIAL_BA" : "LOCAL_BA") :
                    "KF_PROCESSED";
                const char* csvTrackingState = mpTracker ? TrackingStateNameForLog(mpTracker->mState) : "UNKNOWN";

                f_lm_csv
                    << ++mLMIteration                              << ","
                    << (long long)mpCurrentKeyFrame->mnId           << ","
                    << (long long)mpCurrentKeyFrame->mnFrameId      << ","
                    << ts_ms                                       << ","
                    << std::fixed << std::setprecision(3)
                    << csvKFInsert_ms                              << ","
                    << (int)mpAtlas->MapPointsInMap()              << ","
                    << (int)mlpRecentAddedMapPoints.size()         << ","
                    << queueLen                                    << ","
                    << mCsvKFCulled                                << ","
                    << std::setprecision(4)
                    << mCsvMeanReprojErr                           << ","
                    << mCsvMaxReprojErr                            << ","
                    << std::setprecision(3)
                    << mCsvOptimMs                                 << ","
                    << mCsvBADone                                  << ","
                    << mCsvBAAborted                               << ","
                    << mCsvBAIters                                 << ","
                    << csvTrackingLost                             << ","
                    << "LocalMapping"                              << ","
                    << csvEventType                                << ","
                    << std::setprecision(6)
                    << kfTimestamp                                 << ","
                    << relTimeS                                    << ","
                    << csvMapId                                    << ","
                    << csvTrackingState                            << ","
                    << (csvImuInitialized ? "yes" : "no")          << ","
                    << csvBAType                                   << ","
                    << num_OptKF_BA                                << ","
                    << num_FixedKF_BA                              << ","
                    << num_MPs_BA                                  << ","
                    << num_edges_BA                                << ","
                    << csvBAStatsBefore.visual_chi2_mean           << ","
                    << csvBAStatsAfter.visual_chi2_mean            << ","
                    << csvBAStatsBefore.chi2_mean                  << ","
                    << csvBAStatsAfter.chi2_mean                   << ","
                    << csvBAStatsAfter.chi2_mean                   << ","
                    << csvBAStatsAfter.chi2_max                    << ","
                    << csvBAStatsAfter.rot_norm                    << ","
                    << csvBAStatsAfter.vel_norm                    << ","
                    << csvBAStatsAfter.pos_norm                    << ","
                    << csvBAStatsAfter.gyro_rw_chi2                << ","
                    << csvBAStatsAfter.acc_rw_chi2                 << ","
                    << csvBiasDeltaNorm                            << ","
                    << csvVelocityDeltaNorm                        << ","
                    << csvPoseCorrectionTransNorm                  << ","
                    << csvPoseCorrectionRotDeg                     << ","
                    << -1.0                                       << ","
                    << -1.0                                       << ","
                    << -1.0                                       << ","
                    << -1.0                                       << ","
                    << -1.0                                       << ","
                    << 0                                          << "\n";
                if(mLMIteration % 30 == 0)  // flush every 30 iterations
                    f_lm_csv.flush();
            }
        }
        else if(Stop() && !mbBadImu)
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    if(f_lm_csv.is_open())
    {
        f_lm_csv << "#final_keyframes," << mpAtlas->KeyFramesInMap() << "\n";
        // Write surviving KF list
        f_lm_csv << "#survive_kf_id,survive_track_frame_id\n";
        auto vpSurvived = mpAtlas->GetAllKeyFrames();
        std::sort(vpSurvived.begin(), vpSurvived.end(),
            [](KeyFrame* a, KeyFrame* b){ return a->mnId < b->mnId; });
        for(auto* pKF : vpSurvived)
            if(pKF && !pKF->isBad())
                f_lm_csv << pKF->mnId << "," << pKF->mnFrameId << "\n";
        f_lm_csv.flush();
        f_lm_csv.close();
    }
    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::EmptyQueue()
{
    while(CheckNewKeyFrames())
        ProcessNewKeyFrame();
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    int borrar = mlpRecentAddedMapPoints.size();

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;

        if(pMP->isBad())
            lit = mlpRecentAddedMapPoints.erase(lit);
        else if(pMP->GetFoundRatio()<0.25f)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
        {
            lit++;
            borrar--;
        }
    }
}


void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    // int nn = 10;// ← 20（stereo 增加鄰近 KF 搜尋數）開關
    int nn = 1;
    // For stereo inertial case
    if(mbMonocular)
        nn=30;
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    if (mbInertial)
    {
        KeyFrame* pKF = mpCurrentKeyFrame;
        int count=0;
        while((vpNeighKFs.size()<=nn)&&(pKF->mPrevKF)&&(count++<nn))
        {
            vector<KeyFrame*>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
            if(it==vpNeighKFs.end())
                vpNeighKFs.push_back(pKF->mPrevKF);
            pKF = pKF->mPrevKF;
        }
    }

    float th = 0.6f;

    ORBmatcher matcher(th,false);

    Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
    Eigen::Matrix<float,3,4> eigTcw1 = sophTcw1.matrix3x4();
    Eigen::Matrix<float,3,3> Rcw1 = eigTcw1.block<3,3>(0,0);
    Eigen::Matrix<float,3,3> Rwc1 = Rcw1.transpose();
    Eigen::Vector3f tcw1 = sophTcw1.translation();
    Eigen::Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;
    int countStereo = 0;
    int countStereoGoodProj = 0;
    int countStereoAttempt = 0;
    int totalStereoPts = 0;
    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())//開關原版
            return;

        // if(i>=2 )//開關
        // return;



        KeyFrame* pKF2 = vpNeighKFs[i];

        GeometricCamera* pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

        // Check first that baseline is not too short
        Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
        Eigen::Vector3f vBaseline = Ow2-Ow1;
        const float baseline = vBaseline.norm();

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
                continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        bool bCoarse = mbInertial && mpTracker->mState==Tracking::RECENTLY_LOST && mpCurrentKeyFrame->GetMap()->GetIniertialBA2();

        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,vMatchedIndices,false,bCoarse);

        Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
        Eigen::Matrix<float,3,4> eigTcw2 = sophTcw2.matrix3x4();
        Eigen::Matrix<float,3,3> Rcw2 = eigTcw2.block<3,3>(0,0);
        Eigen::Matrix<float,3,3> Rwc2 = Rcw2.transpose();
        Eigen::Vector3f tcw2 = sophTcw2.translation();

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = (mpCurrentKeyFrame -> NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]
                                                                         : (idx1 < mpCurrentKeyFrame -> NLeft) ? mpCurrentKeyFrame -> mvKeys[idx1]
                                                                                                               : mpCurrentKeyFrame -> mvKeysRight[idx1 - mpCurrentKeyFrame -> NLeft];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur>=0);
            const bool bRight1 = (mpCurrentKeyFrame -> NLeft == -1 || idx1 < mpCurrentKeyFrame -> NLeft) ? false
                                                                                                         : true;

            const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                            : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                     : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];

            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur>=0);
            const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                               : true;

            if(mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2){
                if(bRight1 && bRight2){
                    sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera2;
                }
                else if(bRight1 && !bRight2){
                    sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera;
                }
                else if(!bRight1 && bRight2){
                    sophTcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera2;
                }
                else{
                    sophTcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera;
                }
                eigTcw1 = sophTcw1.matrix3x4();
                Rcw1 = eigTcw1.block<3,3>(0,0);
                Rwc1 = Rcw1.transpose();
                tcw1 = sophTcw1.translation();

                eigTcw2 = sophTcw2.matrix3x4();
                Rcw2 = eigTcw2.block<3,3>(0,0);
                Rwc2 = Rcw2.transpose();
                tcw2 = sophTcw2.translation();
            }

            // Check parallax between rays
            Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
            Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

            Eigen::Vector3f ray1 = Rwc1 * xn1;
            Eigen::Vector3f ray2 = Rwc2 * xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            if (bStereo1 || bStereo2) totalStereoPts++;
            
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            Eigen::Vector3f x3D;

            bool goodProj = false;
            bool bPointStereo = false;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 ||
                                                                          (cosParallaxRays<0.9996 && mbInertial) || (cosParallaxRays<0.9998 && !mbInertial)))
            {
                goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
                if(!goodProj)
                    continue;
            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                countStereoAttempt++;
                bPointStereo = true;
                goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                countStereoAttempt++;
                bPointStereo = true;
                goodProj = pKF2->UnprojectStereo(idx2, x3D);
            }
            else
            {
                continue; //No stereo and very low parallax
            }

            if(goodProj && bPointStereo)
                countStereoGoodProj++;

            if(!goodProj)
                continue;

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3D)+tcw1(0);
            const float y1 = Rcw1.row(1).dot(x3D)+tcw1(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1));
                float errX1 = uv1.x - kp1.pt.x;
                float errY1 = uv1.y - kp1.pt.y;

                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;

            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3D)+tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3D)+tcw2(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            Eigen::Vector3f normal1 = x3D - Ow1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = x3D - Ow2;
            float dist2 = normal2.norm();

            if(dist1==0 || dist2==0)
                continue;

            if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // MODIFICATION
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
            if(idx1 >= 0 && idx1 < (int)mpCurrentKeyFrame->mvGrayValues.size())//將當前 KeyFrame（關鍵幀）中的灰度值 (Gray-scale value) 同步給 MapPoint（地圖點）
                pMP->mGray = mpCurrentKeyFrame->mvGrayValues[idx1];
            if (bPointStereo)
                countStereo++;
            
            pMP->AddObservation(mpCurrentKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }    
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=30;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for(int i=0, imax=vpTargetKFs.size(); i<imax; i++)
    {
        const vector<KeyFrame*> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);//20這裡是將第一層鄰居的前五個鄰居加入到第二層鄰居中,開關這裡可以讓 localmapping 的迴圈更快結束，方便 debug localmapping 的部分
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
            pKFi2->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
        }
        if (mbAbortBA)
            break;
    }

    // Extend to temporal neighbors
    if(mbInertial)
    {
        KeyFrame* pKFi = mpCurrentKeyFrame->mPrevKF;
        while(vpTargetKFs.size()<20 && pKFi)
        {
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF==mpCurrentKeyFrame->mnId)
            {
                pKFi = pKFi->mPrevKF;
                continue;
            }
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
            pKFi = pKFi->mPrevKF;
        }
    }

    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
        if(pKFi->NLeft != -1) matcher.Fuse(pKFi,vpMapPointMatches,true);
    }


    if (mbAbortBA)
        return;

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);
    if(mpCurrentKeyFrame->NLeft != -1) matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates,true);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    const int Nd = 21;
    mpCurrentKeyFrame->UpdateBestCovisibles();
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    float redundant_th;
    if(!mbInertial)
        redundant_th = 0.9;
    else if (mbMonocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = mpAtlas->isImuInitialized();
    int count=0;

    // Compoute last KF from optimizable window:
    unsigned int last_ID;
    if (mbInertial)
    {
        int count = 0;
        KeyFrame* aux_KF = mpCurrentKeyFrame;
        while(count<Nd && aux_KF->mPrevKF)
        {
            aux_KF = aux_KF->mPrevKF;
            count++;
        }
        last_ID = aux_KF->mnId;
    }



    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        count++;
        KeyFrame* pKF = *vit;

        if((pKF->mnId==pKF->GetMap()->GetInitKFid()) || pKF->isBad())
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)//開關
                    {
                        const int &scaleLevel = (pKF -> NLeft == -1) ? pKF->mvKeysUn[i].octave
                                                                     : (i < pKF -> NLeft) ? pKF -> mvKeys[i].octave
                                                                                          : pKF -> mvKeysRight[i].octave;
                        const map<KeyFrame*, tuple<int,int>> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            tuple<int,int> indexes = mit->second;
                            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
                            int scaleLeveli = -1;
                            if(pKFi -> NLeft == -1)
                                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                            else {
                                if (leftIndex != -1) {
                                    scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                                }
                                if (rightIndex != -1) {
                                    int rightLevel = pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                                    scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel
                                                                                                  : scaleLeveli;
                                }
                            }

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>thObs)//開關
                                    break;
                            }
                        }
                        if(nObs>thObs)//開關
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>redundant_th*nMPs)
        {
            if (mbInertial)
            {
                if (mpAtlas->KeyFramesInMap()<=Nd)
                    continue;

                if(pKF->mnId>(mpCurrentKeyFrame->mnId-2))
                    continue;

                if(pKF->mPrevKF && pKF->mNextKF)
                {
                    const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;

                    if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                        ++mCsvKFCulled;
                    }
                    else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && ((pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition()).norm()<0.02) && (t<3))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                        ++mCsvKFCulled;
                    }
                }
            }
            else
            {
                pKF->SetBadFlag();
                ++mCsvKFCulled;
            }
        }
        if((count > 20 && mbAbortBA) || count>100)
        {
            break;
        }
    }
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Map reset recieved" << endl;
        mbResetRequested = true;
    }
    cout << "LM: Map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Map reset, Done!!!" << endl;
}

void LocalMapping::RequestResetActiveMap(Map* pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Active map reset recieved" << endl;
        mbResetRequestedActiveMap = true;
        mpMapToReset = pMap;
    }
    cout << "LM: Active map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequestedActiveMap)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Active map reset, Done!!!" << endl;
}

void LocalMapping::ResetIfRequested()
{
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            executed_reset = true;

            cout << "LM: Reseting Atlas in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested = false;
            mbResetRequestedActiveMap = false;

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mIdxInit=0;

            cout << "LM: End reseting Local Mapping..." << endl;
        }

        if(mbResetRequestedActiveMap) {
            executed_reset = true;
            cout << "LM: Reseting current map in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mbResetRequested = false;
            mbResetRequestedActiveMap = false;
            cout << "LM: End reseting Local Mapping..." << endl;
        }
    }
    if(executed_reset)
        cout << "LM: Reset free the mutex" << endl;

}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)
{
    if (mbResetRequested)
        return;

    float minTime;
    int nMinKF;
    if (mbMonocular)
    {
        minTime = 2.0;
        nMinKF = 10;
    }
    else
    {
        minTime = 1.0;
        nMinKF = 10;
    }


    if(mpAtlas->KeyFramesInMap()<nMinKF)
        return;

    // Retrieve all keyframe in temporal order 按時間順序擷取所有關鍵幀
    //建立KF鏈
    list<KeyFrame*> lpKF;
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)//mPrevKF 是一條單向鏈，從最新指向最舊，只包含有預積分連接的 KF。
    {
        lpKF.push_front(pKF);// 插到最前面 
        pKF = pKF->mPrevKF; // 往前一個 
    }
    lpKF.push_front(pKF);// 把最舊的 KF 也加進去
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    if(vpKF.size()<nMinKF)//確認數量
        return;

    mFirstTs=vpKF.front()->mTimeStamp;//vpKF.front() = 最舊的 KF（KF0）
    if(mpCurrentKeyFrame->mTimeStamp-mFirstTs<minTime)//確認時間
        return;

    bInitializing = true;//設定初始化旗標，告訴其他執行緒「我正在初始化，不要打擾」

    while(CheckNewKeyFrames())// 把 queue 裡還沒處理的 KF 全部消化掉，一起納入初始化
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();// N：參與初始化的 KF 總數 
    IMU::Bias b(0,0,0,0,0,0);//b：一個全零的 bias，作為後續估計重力方向時的初始假設

    // Compute and KF velocities mRwg estimation
    if (!mpCurrentKeyFrame->GetMap()->isImuInitialized())
    {
        Eigen::Matrix3f Rwg;
        Eigen::Vector3f dirG;
        dirG.setZero();//dirG：一個用來累積估計重力方向的向量，初始為零
        for(vector<KeyFrame*>::iterator itKF = vpKF.begin(); itKF!=vpKF.end(); itKF++)//遍歷所有參與初始化的 KF，利用它們的預積分 IMU 資訊來估計重力方向和 KF 的速度
        {
            if (!(*itKF)->mpImuPreintegrated)// 沒有 preintegration 資料
                continue;//跳過無效 KF
            if (!(*itKF)->mPrevKF)
                continue;//跳過無效 KF

            dirG -= (*itKF)->mPrevKF->GetImuRotation() * (*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();//累加重力方向，GetUpdatedDeltaVelocity()在 IMU body frame 下，從 prevKF 到此 KF 的速度變化 ΔV 
            Eigen::Vector3f _vel = ((*itKF)->GetImuPosition() - (*itKF)->mPrevKF->GetImuPosition())/(*itKF)->mpImuPreintegrated->dT;//粗估速度 _vel ：平均速度估計（位移 / 時間）
            (*itKF)->SetVelocity(_vel);//把粗估速度存回 KF，供後續優化使用
            (*itKF)->mPrevKF->SetVelocity(_vel);//把粗估速度也存回 prevKF，供後續優化使用
        }

        dirG = dirG/dirG.norm();//歸一化重力方向  變成單位向量，只保留方向。
        Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);//gI：IMU座標系下的重力方向，初始為(0,0,-1) 定義標準重力方向:理想世界座標系下，重力應該是沿 -Z 軸，這是目標方向。 
        Eigen::Vector3f v = gI.cross(dirG);// 旋轉軸
        const float nv = v.norm();
        const float cosg = gI.dot(dirG);//cos(θ)
        const float ang = acos(cosg);// θ（旋轉角）
        Eigen::Vector3f vzg = v*ang/nv;//組成軸角向量 一個向量同時編碼了轉哪個方向和轉多少角度。
        Rwg = Sophus::SO3f::exp(vzg).matrix();//轉成SO(3) 旋轉矩陣
        //Rwg 記錄了「需要旋轉多少」才能讓 -Z 對齊重力
        mRwg = Rwg.cast<double>();//轉成 double 精度並儲存結果
        mTinit = mpCurrentKeyFrame->mTimeStamp-mFirstTs;//記錄此次初始化的時間跨度，供外層決定下一階段 
    }
    else
    {
        mRwg = Eigen::Matrix3d::Identity();//重力方向在第一次初始化時已經對齊過了，世界座標系的 -Z 已經是重力方向，不需要再旋轉，所以 Rwg = 單位矩陣（不旋轉）。 
        mbg = mpCurrentKeyFrame->GetGyroBias().cast<double>();//繼承已有的 bias 估計
        mba = mpCurrentKeyFrame->GetAccBias().cast<double>();//繼承已有的 bias 估計
    }

    mScale=1.0;

    mInitTime = mpTracker->mLastFrame.mTimeStamp-vpKF.front()->mTimeStamp;//未使用到的變數 vpKF.front()（最舊 KF）mLastFrame（Tracker 的最後一幀，包含非 KF 的普通幀）

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mbMonocular, infoInertial, false, false, priorG, priorA);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1)//檢查尺度有沒有被inertialoptimization弄壞
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }

    // Before this line we are not changing the map
    {
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);//在這之前只是計算，不動地圖。從這行開始要修改地圖，需要加鎖防止 Tracking 同時讀取。
        if ((fabs(mScale - 1.f) > 0.00001) || !mbMonocular) {
            Sophus::SE3f Twg(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());//建立重力對齊變換
            mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);//對地圖裡所有 KF 位姿、速度、MapPoint 位置同時套用：旋轉 Twg（重力對齊） 縮放 mScale（尺度修正）
            mpTracker->UpdateFrameIMU(mScale, vpKF[0]->GetImuBias(), mpCurrentKeyFrame);//把 scale 和 bias 同步回 Tracking 執行緒的當前幀，確保兩個執行緒狀態一致。
        }

        // Check if initialization OK
        if (!mpAtlas->isImuInitialized())// 只在第一次初始化時執行（VIBA 1/2 時 isImuInitialized() 已經是 true，跳過）。
            for (int i = 0; i < N; i++) {
                KeyFrame *pKF2 = vpKF[i];
                pKF2->bImu = true;//// 標記這個 KF 參與了 IMU 初始化 bImu = true 讓後續的優化（BA）知道這些 KF 可以加入 IMU 約束邊。
            }
    }
    //標記 IMU 初始化完成
    mpTracker->UpdateFrameIMU(1.0,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    if (!mpAtlas->isImuInitialized())
    {
        mpAtlas->SetImuInitialized(); // 全局旗標設為 true
        mpTracker->t0IMU = mpTracker->mCurrentFrame.mTimeStamp;// 記錄初始化時間點    
        mpCurrentKeyFrame->bImu = true;// 當前 KF 標記為 IMU KF 
    }

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    if (bFIBA)//true，意味著每次初始化都會跑 Full Inertial BA。 
    {
        if (priorA!=0.f)
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, mpCurrentKeyFrame->mnId, NULL, true, priorG, priorA);
        else
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, mpCurrentKeyFrame->mnId, NULL, false);
    }

    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();

    if(f_lm_csv.is_open() && mpCurrentKeyFrame)
    {
        const double kfTimestamp = mpCurrentKeyFrame->mTimeStamp;
        if(mCsvFirstTimestamp < 0.0)
            mCsvFirstTimestamp = kfTimestamp;
        Map* pCsvMap = mpCurrentKeyFrame->GetMap();
        const long unsigned int csvMapId = pCsvMap ? pCsvMap->GetId() : 0;
        const bool csvImuInitialized = pCsvMap && pCsvMap->isImuInitialized();
        const double initOptimMs = std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(
            t5 - t0).count();
        const int queueLen = KeyframesInQueue();
        const char* csvTrackingState = mpTracker ? TrackingStateNameForLog(mpTracker->mState) : "UNKNOWN";
        const string csvBAType = bFIBA ? "InertialOptimizationInit+FullInertialBA" : "InertialOptimizationInit";

        f_lm_csv
            << ++mLMIteration                              << ","
            << (long long)mpCurrentKeyFrame->mnId           << ","
            << (long long)mpCurrentKeyFrame->mnFrameId      << ","
            << std::fixed << std::setprecision(3)
            << kfTimestamp * 1000.0                         << ","
            << -1.0                                        << ","
            << (int)mpAtlas->MapPointsInMap()               << ","
            << (int)mlpRecentAddedMapPoints.size()          << ","
            << queueLen                                     << ","
            << 0                                            << ","
            << std::setprecision(4)
            << -1.0                                        << ","
            << -1.0                                        << ","
            << std::setprecision(3)
            << initOptimMs                                  << ","
            << 1                                            << ","
            << 0                                            << ","
            << (bFIBA ? 100 : -1)                           << ","
            << 0                                            << ","
            << "LocalMapping"                               << ","
            << "IMU_INITIALIZATION"                         << ","
            << std::setprecision(6)
            << kfTimestamp                                  << ","
            << (kfTimestamp - mCsvFirstTimestamp)            << ","
            << csvMapId                                     << ","
            << csvTrackingState                             << ","
            << (csvImuInitialized ? "yes" : "no")           << ","
            << csvBAType                                    << ","
            << N                                            << ","
            << -1                                           << ","
            << (int)mpAtlas->MapPointsInMap()               << ","
            << -1                                           << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << -1.0                                        << ","
            << mScale                                      << ","
            << mbg.norm()                                  << ","
            << mba.norm()                                  << ","
            << priorG                                      << ","
            << priorA                                      << ","
            << (bFIBA ? 1 : 0)                              << "\n";
        f_lm_csv.flush();
    }

    Verbose::PrintMess("Global Bundle Adjustment finished\nUpdating map ...", Verbose::VERBOSITY_NORMAL);

    // Get Map Mutex
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    unsigned long GBAid = mpCurrentKeyFrame->mnId;

    // Process keyframes in the queue
    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    // Correct keyframes starting at map first keyframe  這行用它來初始化 lpKFtoCheck，作為後續廣度優先遍歷（BFS）整棵 KF spanning tree 的起點：  
    list<KeyFrame*> lpKFtoCheck(mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.begin(),mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.end());

    while(!lpKFtoCheck.empty())
    {
        KeyFrame* pKF = lpKFtoCheck.front();
        const set<KeyFrame*> sChilds = pKF->GetChilds();
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
        {
            KeyFrame* pChild = *sit;
            if(!pChild || pChild->isBad())
                continue;

            if(pChild->mnBAGlobalForKF!=GBAid)
            {
                Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

                Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                if(pChild->isVelocitySet()){
                    pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                }
                else {
                    Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);
                }

                pChild->mBiasGBA = pChild->GetImuBias();
                pChild->mnBAGlobalForKF = GBAid;

            }
            lpKFtoCheck.push_back(pChild);
        }

        pKF->mTcwBefGBA = pKF->GetPose();
        pKF->SetPose(pKF->mTcwGBA);

        if(pKF->bImu)
        {
            pKF->mVwbBefGBA = pKF->GetVelocity();
            pKF->SetVelocity(pKF->mVwbGBA);
            pKF->SetNewBias(pKF->mBiasGBA);
        } else {
            cout << "KF " << pKF->mnId << " not set to inertial!! \n";
        }

        lpKFtoCheck.pop_front();
    }

    // Correct MapPoints
    const vector<MapPoint*> vpMPs = mpAtlas->GetCurrentMap()->GetAllMapPoints();

    for(size_t i=0; i<vpMPs.size(); i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())//跳過無效 MP 
            continue;

        if(pMP->mnBAGlobalForKF==GBAid)//mnBAGlobalForKF  記錄這個 MP 上次被哪次 BA 優化（用 KF id 標記）,GBAid  這次 BA 的 id（mpCurrentKeyFrame->mnId）
        {
            // If optimized by Global BA, just update
            pMP->SetWorldPos(pMP->mPosGBA);// mPosGBA BA 優化後的新位置
        }
        else
        {
            // Update according to the correction of its reference keyframe
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

            if(pRefKF->mnBAGlobalForKF!=GBAid)
                continue;

            // Map to non-corrected camera
            Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

            // Backproject using corrected camera
            pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
        }
    }

    Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);

    mnKFs=vpKF.size();// 記錄這次初始化用了幾個 KF
    mIdxInit++;// 初始化執行次數 +1（粗初始化/VIBA1/VIBA2 各算一次）

    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)//清理初始化期間積累的新 KF  
    {
        (*lit)->SetBadFlag();// 標記為壞 KF
        delete *lit;// 釋放記憶體 
    }
    mlNewKeyFrames.clear();//mlNewKeyFrames 是初始化執行期間 Tracking 送進來但還沒處理的 KF。 這些 KF 在初始化過程中沒有被納入優化，位姿可能不一致，直接丟棄讓系統重新建立。  

    mpTracker->mState=Tracking::OK;// 通知 Tracking 初始化完成，可以繼續追蹤
    bInitializing = false;// 解除初始化鎖，LocalMapping 恢復正常工作  bInitializing = true 期間，LocalMapping 會拒絕某些操作（如 Loop Closing 觸發的 BA），這裡解除。  

    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();// 通知地圖已變更   地圖的 ChangeIndex +1，讓 Tracking 偵測到地圖有更新：  

    return;
}

void LocalMapping::ScaleRefinement()
{
    // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
    // unique_lock<mutex> lock0(mMutexImuInit);
    if (mbResetRequested)
        return;

    // Retrieve all keyframes in temporal order
    list<KeyFrame*> lpKF;
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();

    mRwg = Eigen::Matrix3d::Identity();
    mScale=1.0;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1) // 1e-1
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }
    
    Sophus::SO3d so3wg(mRwg);
    // Before this line we are not changing the map
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.002)||!mbMonocular)
    {
        Sophus::SE3f Tgw(mRwg.cast<float>().transpose(),Eigen::Vector3f::Zero());
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Tgw,mScale,true);
        mpTracker->UpdateFrameIMU(mScale,mpCurrentKeyFrame->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    double t_inertial_only = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();

    // To perform pose-inertial opt w.r.t. last keyframe
    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}



bool LocalMapping::IsInitializing()
{
    return bInitializing;
}


double LocalMapping::GetCurrKFTime()
{

    if (mpCurrentKeyFrame)
    {
        return mpCurrentKeyFrame->mTimeStamp;
    }
    else
        return 0.0;
}

KeyFrame* LocalMapping::GetCurrKF()
{
    return mpCurrentKeyFrame;
}

void LocalMapping::ComputeLocalWindowReprojErrors(double &mean_err, double &max_err)
{
    mean_err = 0.0;
    max_err  = 0.0;
    if(!mpCurrentKeyFrame) return;

    // Local window: current KF + its best covisible KFs
    vector<KeyFrame*> vpLocalKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(10);
    vpLocalKFs.push_back(mpCurrentKeyFrame);

    double sum_err = 0.0;
    int    count   = 0;

    for(KeyFrame* pKF : vpLocalKFs)
    {
        if(!pKF || pKF->isBad()) continue;

        Sophus::SE3f Tcw = pKF->GetPose();
        const float fx = pKF->fx, fy = pKF->fy;
        const float cx = pKF->cx, cy = pKF->cy;

        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
        // For stereo/fisheye rigs, only consider the left camera keypoints
        int nLeft = (pKF->NLeft == -1) ? (int)vpMPs.size() : pKF->NLeft;

        for(int i = 0; i < nLeft && i < (int)vpMPs.size(); ++i)
        {
            MapPoint* pMP = vpMPs[i];
            if(!pMP || pMP->isBad()) continue;

            Eigen::Vector3f Pw = pMP->GetWorldPos();
            Eigen::Vector3f Pc = Tcw * Pw;
            if(Pc(2) <= 0.0f) continue;

            float invz = 1.0f / Pc(2);
            float u = fx * Pc(0) * invz + cx;
            float v = fy * Pc(1) * invz + cy;

            const cv::KeyPoint& kp = pKF->mvKeysUn[i];
            float du = u - kp.pt.x;
            float dv = v - kp.pt.y;
            double err = std::sqrt((double)(du*du + dv*dv));

            sum_err += err;
            if(err > max_err) max_err = err;
            ++count;
        }
    }

    if(count > 0)
        mean_err = sum_err / count;
}

} //namespace ORB_SLAM
