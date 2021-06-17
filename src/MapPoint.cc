/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM3
{

long unsigned int MapPoint::nNextId=0;
std::mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint():
    mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL))
{
    mpReplaced = static_cast<MapPoint*>(NULL);
}

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId())
{
    Pos.copyTo(mWorldPos);
    mWorldPosx = cv::Matx31f(Pos.at<float>(0), Pos.at<float>(1), Pos.at<float>(2));
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);
    mNormalVectorx = cv::Matx31f::zeros();

    mbTrackInViewR = false;
    mbTrackInView = false;

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId())
{
    mInvDepth=invDepth;
    mInitU=(double)uv_init.x;
    mInitV=(double)uv_init.y;
    mpHostKF = pHostKF;

    mNormalVector = cv::Mat::zeros(3,1,CV_32F);
    mNormalVectorx = cv::Matx31f::zeros();

    // Worldpos is not set
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId())
{
    Pos.copyTo(mWorldPos);
    mWorldPosx = cv::Matx31f(Pos.at<float>(0), Pos.at<float>(1), Pos.at<float>(2));

    cv::Mat Ow;
    if(pFrame -> Nleft == -1 || idxF < pFrame -> Nleft){
        Ow = pFrame->GetCameraCenter();
    }
    else{
        cv::Mat Rwl = pFrame -> mRwc;
        cv::Mat tlr = pFrame -> mTlr.col(3);
        cv::Mat twl = pFrame -> mOw;

        Ow = Rwl * tlr + twl;
    }
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);
    mNormalVectorx = cv::Matx31f(mNormalVector.at<float>(0), mNormalVector.at<float>(1), mNormalVector.at<float>(2));


    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
                                              : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
                                                                         : pFrame -> mvKeysRight[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    std::unique_lock<std::mutex> lock2(mGlobalMutex);
    std::unique_lock<std::mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
    mWorldPosx = cv::Matx31f(Pos.at<float>(0), Pos.at<float>(1), Pos.at<float>(2));
}

cv::Mat MapPoint::GetWorldPos()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

cv::Matx31f MapPoint::GetWorldPos2()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mWorldPosx;
}

cv::Matx31f MapPoint::GetNormal2()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mNormalVectorx;
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, int idx)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    std::tuple<int,int> indexes;

    if(mObservations.count(pKF)){
        indexes = mObservations[pKF];
    }
    else{
        indexes = std::tuple<int,int>(-1,-1);
    }

    if(pKF -> NLeft != -1 && idx >= pKF -> NLeft){
        std::get<1>(indexes) = idx;
    }
    else{
        std::get<0>(indexes) = idx;
    }

    mObservations[pKF]=indexes;

    if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        std::unique_lock<std::mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            std::tuple<int,int> indexes = mObservations[pKF];
            int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

            if(leftIndex != -1){
                if(!pKF->mpCamera2 && pKF->mvuRight[leftIndex]>=0)
                    nObs-=2;
                else
                    nObs--;
            }
            if(rightIndex != -1){
                nObs--;
            }

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}


std::map<KeyFrame*, std::tuple<int,int>>  MapPoint::GetObservations()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    std::map<KeyFrame*, std::tuple<int,int>> obs;
    {
        std::unique_lock<std::mutex> lock1(mMutexFeatures);
        std::unique_lock<std::mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(std::map<KeyFrame*, std::tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        int leftIndex = std::get<0>(mit -> second), rightIndex = std::get<1>(mit -> second);
        if(leftIndex != -1){
            pKF->EraseMapPointMatch(leftIndex);
        }
        if(rightIndex != -1){
            pKF->EraseMapPointMatch(rightIndex);
        }
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    std::unique_lock<std::mutex> lock1(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    std::map<KeyFrame*,std::tuple<int,int>> obs;
    {
        std::unique_lock<std::mutex> lock1(mMutexFeatures);
        std::unique_lock<std::mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(std::map<KeyFrame*,std::tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        std::tuple<int,int> indexes = mit -> second;
        int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

        if(!pMP->IsInKeyFrame(pKF))
        {
            if(leftIndex != -1){
                pKF->ReplaceMapPointMatch(leftIndex, pMP);
                pMP->AddObservation(pKF,leftIndex);
            }
            if(rightIndex != -1){
                pKF->ReplaceMapPointMatch(rightIndex, pMP);
                pMP->AddObservation(pKF,rightIndex);
            }
        }
        else
        {
            if(leftIndex != -1){
                pKF->EraseMapPointMatch(leftIndex);
            }
            if(rightIndex != -1){
                pKF->EraseMapPointMatch(rightIndex);
            }
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    std::unique_lock<std::mutex> lock1(mMutexFeatures,std::defer_lock);
    std::unique_lock<std::mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2);

    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    std::vector<cv::Mat> vDescriptors;

    std::map<KeyFrame*,std::tuple<int,int>> observations;

    {
        std::unique_lock<std::mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(std::map<KeyFrame*,std::tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad()){
            std::tuple<int,int> indexes = mit -> second;
            int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

            if(leftIndex != -1){
                vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
            }
            if(rightIndex != -1){
                vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
            }
        }
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        std::vector<int> vDists(Distances[i],Distances[i]+N);
        std::sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        std::unique_lock<std::mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

std::tuple<int,int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return std::tuple<int,int>(-1,-1);
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    std::map<KeyFrame*,std::tuple<int,int>> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        std::unique_lock<std::mutex> lock1(mMutexFeatures);
        std::unique_lock<std::mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(std::map<KeyFrame*,std::tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        std::tuple<int,int> indexes = mit -> second;
        int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

        if(leftIndex != -1){
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali/cv::norm(normali);
            n++;
        }
        if(rightIndex != -1){
            cv::Mat Owi = pKF->GetRightCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali/cv::norm(normali);
            n++;
        }
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);

    std::tuple<int ,int> indexes = observations[pRefKF];
    int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);
    int level;
    if(pRefKF -> NLeft == -1){
        level = pRefKF->mvKeysUn[leftIndex].octave;
    }
    else if(leftIndex != -1){
        level = pRefKF -> mvKeys[leftIndex].octave;
    }
    else{
        level = pRefKF -> mvKeysRight[rightIndex - pRefKF -> NLeft].octave;
    }

    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        std::unique_lock<std::mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
        mNormalVectorx = cv::Matx31f(mNormalVector.at<float>(0), mNormalVector.at<float>(1), mNormalVector.at<float>(2));
    }
}

void MapPoint::SetNormalVector(cv::Mat& normal)
{
    std::unique_lock<std::mutex> lock3(mMutexPos);
    mNormalVector = normal;
    mNormalVectorx = cv::Matx31f(mNormalVector.at<float>(0), mNormalVector.at<float>(1), mNormalVector.at<float>(2));
}

float MapPoint::GetMinDistanceInvariance()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        std::unique_lock<std::mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        std::unique_lock<std::mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

Map* MapPoint::GetMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mpMap;
}

void MapPoint::UpdateMap(Map* pMap)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mpMap = pMap;
}

} //namespace ORB_SLAM
