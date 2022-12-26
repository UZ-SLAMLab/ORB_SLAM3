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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM3
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

/** 
 * @brief 构造函数
 */
MapPoint::MapPoint():
    mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL))
{
    mpReplaced = static_cast<MapPoint*>(NULL);
}

/** 
 * @brief 构造函数
 */
MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId())
{
    SetWorldPos(Pos);

    mNormalVector.setZero();

    mbTrackInViewR = false;
    mbTrackInView = false;

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

/** 
 * @brief 构造函数
 */
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

    mNormalVector.setZero();

    // Worldpos is not set
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

/** 
 * @brief 构造函数
 */
MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId())
{
    SetWorldPos(Pos);

    Eigen::Vector3f Ow;
    if(pFrame -> Nleft == -1 || idxF < pFrame -> Nleft){
        Ow = pFrame->GetCameraCenter();
    }
    else{
        Eigen::Matrix3f Rwl = pFrame->GetRwc();
        Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
        Eigen::Vector3f twl = pFrame->GetOw();

        Ow = Rwl * tlr + twl;
    }
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / mNormalVector.norm();

    Eigen::Vector3f PC = mWorldPos - Ow;
    const float dist = PC.norm();
    const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
                                              : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
                                                                         : pFrame -> mvKeysRight[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

/**
 * @brief 设定该mp的世界坐标
 * @param Pos         坐标值
 */
void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos = Pos;
}

/**
 * @brief 返回该mp的世界坐标
 */
Eigen::Vector3f MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos;
}

/**
 * @brief 获取平均观测方向
 */
Eigen::Vector3f MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector;
}


KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

/**
 * @brief 给地图点添加观测
 *
 * 记录哪些 KeyFrame 的那个特征点能观测到该 地图点
 * 并增加观测的相机数目nObs，单目+1，双目或者rgbd+2
 * 这个函数是建立关键帧共视关系的核心函数，能共同观测到某些地图点的关键帧是共视关键帧
 * @param pKF KeyFrame
 * @param idx MapPoint在KeyFrame中的索引
 */
void MapPoint::AddObservation(KeyFrame* pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    tuple<int,int> indexes;

    if(mObservations.count(pKF)){
        indexes = mObservations[pKF];
    }
    else{
        indexes = tuple<int,int>(-1,-1);
    }

    if(pKF -> NLeft != -1 && idx >= pKF -> NLeft){
        get<1>(indexes) = idx;
    }
    else{
        get<0>(indexes) = idx;
    }

    // 如果没有添加过观测，记录下能观测到该MapPoint的KF和该MapPoint在KF中的索引
    mObservations[pKF]=indexes;

    if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

// 删除某个关键帧对当前地图点的观测
void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        // 查找这个要删除的观测,根据单目和双目类型的不同从其中删除当前地图点的被观测次数
        if(mObservations.count(pKF))
        {
            tuple<int,int> indexes = mObservations[pKF];
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

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

            // 如果该keyFrame是参考帧，该Frame被删除后重新指定RefFrame
            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            // 当观测到该点的相机数目少于2时，丢弃该点
            if(nObs<=2)
                bBad=true;
        }
    }
    // 告知可以观测到该MapPoint的Frame，该MapPoint已被删除
    if(bBad)
        SetBadFlag();
}

// 能够观测到当前地图点的所有关键帧及该地图点在KF中的索引
std::map<KeyFrame*, std::tuple<int,int>>  MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

/**
 * @brief 返回被观测次数，双目一帧算两次，左右目各算各的
 * @return nObs
 */
int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

/**
 * @brief 告知可以观测到该MapPoint的Frame，该MapPoint已被删除
 * 
 */
void MapPoint::SetBadFlag()
{
    map<KeyFrame*, tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        // 把mObservations转存到obs，obs和mObservations里存的是指针，赋值过程为浅拷贝
        obs = mObservations;
        // 把mObservations指向的内存释放，obs作为局部变量之后自动删除
        mObservations.clear();
    }
    for(map<KeyFrame*, tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        int leftIndex = get<0>(mit -> second), rightIndex = get<1>(mit -> second);
        if(leftIndex != -1){
            pKF->EraseMapPointMatch(leftIndex);
        }
        if(rightIndex != -1){
            pKF->EraseMapPointMatch(rightIndex);
        }
    }

    // 擦除该MapPoint申请的内存
    mpMap->EraseMapPoint(this);
}

/**
 * @brief 判断该点是否已经被替换，因为替换并没有考虑普通帧的替换，不利于下一帧的跟踪，所以要坐下标记
 * @return 替换的新的点
 */
MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

/**
 * @brief 替换地图点，更新观测关系
 * 
 * @param[in] pMP       用该地图点来替换当前地图点
 */
void MapPoint::Replace(MapPoint* pMP)
{
    // 同一个地图点则跳过
    if(pMP->mnId==this->mnId)
        return;

    //要替换当前地图点,有两个工作:
    // 1. 将当前地图点的观测数据等其他数据都"叠加"到新的地图点上
    // 2. 将观测到当前地图点的关键帧的信息进行更新


    // 清除当前地图点的信息，这一段和SetBadFlag函数相同
    int nvisible, nfound;
    map<KeyFrame*,tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        // 清除当前地图点的原有观测
        mObservations.clear();
        // 当前的地图点被删除了
        mbBad=true;
        // 暂存当前地图点的可视次数和被找到的次数
        nvisible = mnVisible;
        nfound = mnFound;
        // 指明当前地图点已经被指定的地图点替换了
        mpReplaced = pMP;
    }

    // 所有能观测到原地图点的关键帧都要复制到替换的地图点上
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        // 2.1 判断新点是否已经在pKF里面
        if(!pMP->IsInKeyFrame(pKF))
        {
            // 如果不在，替换特征点与mp的匹配关系
            if(leftIndex != -1)
            {
                pKF->ReplaceMapPointMatch(leftIndex, pMP);
                pMP->AddObservation(pKF,leftIndex);
            }
            if(rightIndex != -1)
            {
                pKF->ReplaceMapPointMatch(rightIndex, pMP);
                pMP->AddObservation(pKF,rightIndex);
            }
        }
        // 如果新的MP在之前MP对应的关键帧里面，就撞车了。
        // 本来目的想新旧MP融为一个，这样以来一个点有可能对应两个特征点，这样是决不允许的，所以删除旧的，不动新的
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
    //- 将当前地图点的观测数据等其他数据都"叠加"到新的地图点上
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    // 描述子更新
    pMP->ComputeDistinctiveDescriptors();

    // 告知地图,删掉我
    mpMap->EraseMapPoint(this);
}

/**
 * @brief 判断这个点是否设置为bad了
 */
bool MapPoint::isBad()
{
    unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2);

    return mbBad;
}

/**
 * @brief Increase Visible
 *
 * Visible表示：
 * 1. 该MapPoint在某些帧的视野范围内，通过Frame::isInFrustum()函数判断
 * 2. 该MapPoint被这些帧观测到，但并不一定能和这些帧的特征点匹配上
 *    例如：有一个MapPoint（记为M），在某一帧F的视野范围内，
 *    但并不表明该点M可以和F这一帧的某个特征点能匹配上
 */
void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

/**
 * @brief Increase Found
 *
 * 能找到该点的帧数+n，n默认为1
 * @see Tracking::TrackLocalMap()
 */
void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

/**
 * @brief 返回被找到/被看到
 * @return 被找到/被看到
 */
float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

/**
 * @brief 计算地图点具有代表性的描述子
 *
 * 由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要判断是否更新当前点的最适合的描述子 
 * 先获得当前点的所有描述子，然后计算描述子之间的两两距离，最好的描述子与其他描述子应该具有最小的距离中值
 */
void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,tuple<int,int>> observations;

    // Step 1 获取所有观测，跳过坏点
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    // Step 2 遍历观测到3d点的所有关键帧，获得orb描述子，并插入到vDescriptors中
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        // mit->first取观测到该地图点的关键帧
        // mit->second取该地图点在关键帧中的索引
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
        {
            tuple<int,int> indexes = mit -> second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1)
            {
                vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
            }
            if(rightIndex != -1)
            {
                vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
            }
        }
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    // Step 3 获得这些描述子两两之间的距离
    // N表示为一共多少个描述子
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
         // 和自己的距离当然是0
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    // Step 4 选择最有代表性的描述子，它与其他描述子应该具有最小的距离中值
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        // 第i个描述子到其它所有所有描述子之间的距离
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        // 获得中值
        int median = vDists[0.5*(N-1)];

        // 寻找最小的中值
        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        // 最好的描述子，该描述子相对于其他描述子有最小的距离中值
        // 简化来讲，中值代表了这个描述子到其它描述子的平均距离
        // 最好的描述子就是和其它描述子的平均距离最小
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

/**
 * @brief 返回描述子
 */
cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

/**
 * @brief 返回这个点在关键帧中对应的特征点id
 */
tuple<int,int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return tuple<int,int>(-1,-1);
}

/**
 * @brief return (mObservations.count(pKF));
 */
bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

/**
 * @brief 更新平均观测方向以及观测距离范围
 *
 * 由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要更新相应变量
 * 创建新的关键帧的时候会调用
 */
void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,tuple<int,int>> observations;
    KeyFrame* pRefKF;
    Eigen::Vector3f Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations = mObservations; // 获得观测到该地图点的所有关键帧
        pRefKF = mpRefKF;             // 观测到该点的参考关键帧（第一次创建时的关键帧）
        Pos = mWorldPos;              // 地图点在世界坐标系中的位置
    }

    if(observations.empty())
        return;

    // Step 2 计算该地图点的法线方向，也就是朝向等信息。
    // 能观测到该地图点的所有关键帧，对该点的观测方向归一化为单位向量，然后进行求和得到该地图点的朝向
    // 初始值为0向量，累加为归一化向量，最后除以总数n
    Eigen::Vector3f normal;
    normal.setZero();
    int n = 0;
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(leftIndex != -1)
        {
            Eigen::Vector3f Owi = pKF->GetCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
        if(rightIndex != -1)
        {
            Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
    }

    Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();  // 参考关键帧相机指向地图点的向量（在世界坐标系下的表示）
    const float dist = PC.norm();  // 该点到参考关键帧相机的距离

    tuple<int ,int> indexes = observations[pRefKF];
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int level;
    if(pRefKF -> NLeft == -1)
    {
        level = pRefKF->mvKeysUn[leftIndex].octave;
    }
    else if(leftIndex != -1)
    {
        level = pRefKF -> mvKeys[leftIndex].octave;
    }
    else{
        level = pRefKF -> mvKeysRight[rightIndex - pRefKF -> NLeft].octave;
    }

    //const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];  // 当前金字塔层对应的缩放倍数
    const int nLevels = pRefKF->mnScaleLevels;  // 金字塔层数

    {
        unique_lock<mutex> lock3(mMutexPos);
        // 使用方法见PredictScale函数前的注释
        mfMaxDistance = dist*levelScaleFactor;  // 观测到该点的距离上限
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];  // 观测到该点的距离下限
        mNormalVector = normal/n;  // 获得地图点平均的观测方向
    }
}

/**
 * @brief 设置平均观测方向
 * @param normal         观测方向
 */
void MapPoint::SetNormalVector(const Eigen::Vector3f& normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    mNormalVector = normal;
}

/**
 * @brief 返回最近距离
 */
float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
}

/**
 * @brief 返回最远距离
 */
float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
}

// 下图中横线的大小表示不同图层图像上的一个像素表示的真实物理空间中的大小
//              ____
// Nearer      /____\     level:n-1 --> dmin
//            /______\                       d/dmin = 1.2^(n-1-m)
//           /________\   level:m   --> d
//          /__________\                     dmax/d = 1.2^m
// Farther /____________\ level:0   --> dmax
//
//           log(dmax/d)
// m = ceil(------------)
//            log(1.2)
// 这个函数的作用:
// 在进行投影匹配的时候会给定特征点的搜索范围,考虑到处于不同尺度(也就是距离相机远近,位于图像金字塔中不同图层)的特征点受到相机旋转的影响不同,
// 因此会希望距离相机近的点的搜索范围更大一点,距离相机更远的点的搜索范围更小一点,所以要在这里,根据点到关键帧/帧的距离来估计它在当前的关键帧/帧中,
// 会大概处于哪个尺度
int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        // mfMaxDistance = ref_dist*levelScaleFactor 为参考帧考虑上尺度后的距离
        // ratio = mfMaxDistance/currentDist = ref_dist/cur_dist
        ratio = mfMaxDistance/currentDist;
    }

    // 同时取log线性化
    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

/**
 * @brief 根据地图点到光心的距离来预测一个类似特征金字塔的尺度
 * 
 * @param[in] currentDist       地图点到光心的距离
 * @param[in] pF                当前帧
 * @return int                  尺度
 */
int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    // 同时取log线性化
    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

void MapPoint::PrintObservations()
{
    cout << "MP_OBS: MP " << mnId << endl;
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=mObservations.begin(), mend=mObservations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKFi = mit->first;
        tuple<int,int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
    }
}

Map* MapPoint::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void MapPoint::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

/**
 * @brief 预保存
 * @param spKF 地图中所有关键帧
 * @param spMP 地图中所有三维点
 */
void MapPoint::PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP)
{
    // 1. 备份替代的MP id
    mBackupReplacedId = -1;
    if(mpReplaced && spMP.find(mpReplaced) != spMP.end())
        mBackupReplacedId = mpReplaced->mnId;

    // 2. 备份观测
    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
    // Save the id and position in each KF who view it
    for(std::map<KeyFrame*,std::tuple<int,int> >::const_iterator it = mObservations.begin(), end = mObservations.end(); it != end; ++it)
    {
        KeyFrame* pKFi = it->first;
        if(spKF.find(pKFi) != spKF.end())
        {
            mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
            mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
        }
        else
        {
            EraseObservation(pKFi);
        }
    }

    // Save the id of the reference KF
    // 3. 备份参考关键帧ID
    if(spKF.find(mpRefKF) != spKF.end())
    {
        mBackupRefKFId = mpRefKF->mnId;
    }
}

/**
 * @brief 后加载
 */
void MapPoint::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid)
{
    // 1. 根据保存的ID加载参考关键帧与替代点
    mpRefKF = mpKFid[mBackupRefKFId];
    if(!mpRefKF)
    {
        cout << "ERROR: MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
    }
    mpReplaced = static_cast<MapPoint*>(NULL);
    if(mBackupReplacedId>=0)
    {
        map<long unsigned int, MapPoint*>::iterator it = mpMPid.find(mBackupReplacedId);
        if (it != mpMPid.end())
            mpReplaced = it->second;
    }

    // 2. 加载观测
    mObservations.clear();

    for(map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end(); it != end; ++it)
    {
        KeyFrame* pKFi = mpKFid[it->first];
        map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
        std::tuple<int, int> indexes = tuple<int,int>(it->second,it2->second);
        if(pKFi)
        {
           mObservations[pKFi] = indexes;
        }
    }

    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
}

} //namespace ORB_SLAM
