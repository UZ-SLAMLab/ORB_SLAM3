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


#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM3
{

    const int ORBmatcher::TH_HIGH = 100;
    const int ORBmatcher::TH_LOW = 50;
    const int ORBmatcher::HISTO_LENGTH = 30;

    // 构造函数,参数默认值为0.6,true
    ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
    {
    }

    // 用于Tracking::SearchLocalPoints中匹配更多地图点
    int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th, const bool bFarPoints, const float thFarPoints)
    {
        int nmatches=0, left = 0, right = 0;

        // 如果 th！=1 (RGBD 相机或者刚刚进行过重定位), 需要扩大范围搜索
        const bool bFactor = th!=1.0;

        // Step 1 遍历有效的局部地图点
        for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
        {
            MapPoint* pMP = vpMapPoints[iMP];
            if(!pMP->mbTrackInView && !pMP->mbTrackInViewR)
                continue;

            if(bFarPoints && pMP->mTrackDepth>thFarPoints)
                continue;

            if(pMP->isBad())
                continue;

            if(pMP->mbTrackInView)
            {
                // 通过距离预测的金字塔层数，该层数相对于当前的帧
                const int &nPredictedLevel = pMP->mnTrackScaleLevel;

                // The size of the window will depend on the viewing direction
                // Step 2 设定搜索搜索窗口的大小。取决于视角, 若当前视角和平均视角夹角较小时, r取一个较小的值
                float r = RadiusByViewingCos(pMP->mTrackViewCos);

                // 如果需要扩大范围搜索，则乘以阈值th
                if(bFactor)
                    r*=th;

                // Step 3 通过投影点以及搜索窗口和预测的尺度进行搜索, 找出搜索半径内的候选匹配点索引
                const vector<size_t> vIndices =
                        F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,      // 该地图点投影到一帧上的坐标
                                            r*F.mvScaleFactors[nPredictedLevel],    // 认为搜索窗口的大小和该特征点被追踪到时所处的尺度也有关系
                                            nPredictedLevel-1,nPredictedLevel);     // 搜索的图层范围

                // 没找到候选的,就放弃对当前点的匹配
                if(!vIndices.empty()){
                    const cv::Mat MPdescriptor = pMP->GetDescriptor();

                    // 最优的次优的描述子距离和index
                    int bestDist=256;
                    int bestLevel= -1;
                    int bestDist2=256;
                    int bestLevel2 = -1;
                    int bestIdx =-1 ;

                    // Get best and second matches with near keypoints
                    // Step 4 寻找候选匹配点中的最佳和次佳匹配点
                    for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
                    {
                        const size_t idx = *vit;

                        // 如果Frame中的该兴趣点已经有对应的MapPoint了,则退出该次循环
                        if(F.mvpMapPoints[idx])
                            if(F.mvpMapPoints[idx]->Observations()>0)
                                continue;

                        //如果是双目数据
                        if(F.Nleft == -1 && F.mvuRight[idx]>0)
                        {
                            //计算在X轴上的投影误差
                            const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                            //超过阈值,说明这个点不行,丢掉.
                            //这里的阈值定义是以给定的搜索范围r为参考,然后考虑到越近的点(nPredictedLevel越大), 相机运动时对其产生的影响也就越大,
                            //因此需要扩大其搜索空间.
                            //当给定缩放倍率为1.2的时候, mvScaleFactors 中的数据是: 1 1.2 1.2^2 1.2^3 ... 
                            if(er>r*F.mvScaleFactors[nPredictedLevel])
                                continue;
                        }

                        const cv::Mat &d = F.mDescriptors.row(idx);

                        // 计算地图点和候选投影点的描述子距离
                        const int dist = DescriptorDistance(MPdescriptor,d);

                        // 寻找描述子距离最小和次小的特征点和索引
                        if(dist<bestDist)
                        {
                            bestDist2=bestDist;
                            bestDist=dist;
                            bestLevel2 = bestLevel;
                            bestLevel = (F.Nleft == -1) ? F.mvKeysUn[idx].octave
                                                        : (idx < F.Nleft) ? F.mvKeys[idx].octave
                                                                        : F.mvKeysRight[idx - F.Nleft].octave;
                            bestIdx=idx;
                        }
                        else if(dist<bestDist2)
                        {
                            bestLevel2 = (F.Nleft == -1) ? F.mvKeysUn[idx].octave
                                                        : (idx < F.Nleft) ? F.mvKeys[idx].octave
                                                                        : F.mvKeysRight[idx - F.Nleft].octave;
                            bestDist2=dist;
                        }
                    }

                    // Apply ratio to second match (only if best and second are in the same scale level)
                    // Step 5 筛选最佳匹配点
                    // 最佳匹配距离还需要满足在设定阈值内
                    if(bestDist<=TH_HIGH)
                    {
                        // 条件1：bestLevel==bestLevel2 表示 最佳和次佳在同一金字塔层级
                        // 条件2：bestDist>mfNNratio*bestDist2 表示最佳和次佳距离不满足阈值比例。理论来说 bestDist/bestDist2 越小越好
                        if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                            continue;

                        if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){
                            F.mvpMapPoints[bestIdx]=pMP;

                            if(F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                                F.mvpMapPoints[F.mvLeftToRightMatch[bestIdx] + F.Nleft] = pMP;
                                nmatches++;
                                right++;
                            }

                            nmatches++;
                            left++;
                        }
                    }
                }
            }

            if(F.Nleft != -1 && pMP->mbTrackInViewR){
                const int &nPredictedLevel = pMP->mnTrackScaleLevelR;
                if(nPredictedLevel != -1){
                    float r = RadiusByViewingCos(pMP->mTrackViewCosR);

                    const vector<size_t> vIndices =
                            F.GetFeaturesInArea(pMP->mTrackProjXR,pMP->mTrackProjYR,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel,true);

                    if(vIndices.empty())
                        continue;

                    const cv::Mat MPdescriptor = pMP->GetDescriptor();

                    int bestDist=256;
                    int bestLevel= -1;
                    int bestDist2=256;
                    int bestLevel2 = -1;
                    int bestIdx =-1 ;

                    // Get best and second matches with near keypoints
                    for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
                    {
                        const size_t idx = *vit;

                        if(F.mvpMapPoints[idx + F.Nleft])
                            if(F.mvpMapPoints[idx + F.Nleft]->Observations()>0)
                                continue;


                        const cv::Mat &d = F.mDescriptors.row(idx + F.Nleft);

                        const int dist = DescriptorDistance(MPdescriptor,d);

                        if(dist<bestDist)
                        {
                            bestDist2=bestDist;
                            bestDist=dist;
                            bestLevel2 = bestLevel;
                            bestLevel = F.mvKeysRight[idx].octave;
                            bestIdx=idx;
                        }
                        else if(dist<bestDist2)
                        {
                            bestLevel2 = F.mvKeysRight[idx].octave;
                            bestDist2=dist;
                        }
                    }

                    // Apply ratio to second match (only if best and second are in the same scale level)
                    if(bestDist<=TH_HIGH)
                    {
                        if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                            continue;

                        if(F.Nleft != -1 && F.mvRightToLeftMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                            F.mvpMapPoints[F.mvRightToLeftMatch[bestIdx]] = pMP;
                            nmatches++;
                            left++;
                        }


                        F.mvpMapPoints[bestIdx + F.Nleft]=pMP;
                        nmatches++;
                        right++;
                    }
                }
            }
        }
        return nmatches;
    }

    // 根据观察的视角来计算匹配的时的搜索窗口大小
    float ORBmatcher::RadiusByViewingCos(const float &viewCos)
    {
        // 当视角相差小于3.6°，对应cos(3.6°)=0.998，搜索范围是2.5，否则是4
        if(viewCos>0.998)
            return 2.5;
        else
            return 4.0;
    }

    /**
     * @brief 通过词袋，对关键帧的特征点进行跟踪
     * 
     * @param[in] pKF               关键帧
     * @param[in] F                 当前普通帧
     * @param[in] vpMapPointMatches F中地图点对应的匹配，NULL表示未匹配
     * @return int                  成功匹配的数量
     */
    int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
    {
        // 获取该关键帧的地图点
        const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

        // 和普通帧F特征点的索引一致
        vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));

        // 取出关键帧的词袋特征向量
        const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

        int nmatches=0;

        // 特征点角度旋转差统计用的直方图
        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        // 将0~360的数转换到0~HISTO_LENGTH的系数
        //! 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码  
        // const float factor = HISTO_LENGTH/360.0f;
        const float factor = 1.0f/HISTO_LENGTH;

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        // 将属于同一节点(特定层)的ORB特征进行匹配
        DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
        DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
        DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
        DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

        while(KFit != KFend && Fit != Fend)
        {
            // Step 1：分别取出属于同一node的ORB特征点(只有属于同一node，才有可能是匹配点)
            // first 元素就是node id，遍历
            if(KFit->first == Fit->first) 
            {
                // second 是该node内存储的feature index
                const vector<unsigned int> vIndicesKF = KFit->second;
                const vector<unsigned int> vIndicesF = Fit->second;

                // Step 2：遍历KF中属于该node的特征点
                for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
                {
                    // 关键帧该节点中特征点的索引
                    const unsigned int realIdxKF = vIndicesKF[iKF];

                    // 取出KF中该特征对应的地图点
                    MapPoint* pMP = vpMapPointsKF[realIdxKF];

                    if(!pMP)
                        continue;

                    if(pMP->isBad())
                        continue;
                    // 取出关键帧KF中该特征对应的描述子
                    const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF); 

                    int bestDist1=256; // 最好的距离（最小距离）
                    int bestIdxF =-1 ;
                    int bestDist2=256; // 次好距离（倒数第二小距离）

                    int bestDist1R=256;
                    int bestIdxFR =-1 ;
                    int bestDist2R=256;
                    // Step 3：遍历F中属于该node的特征点，寻找最佳匹配点
                    for(size_t iF=0; iF<vIndicesF.size(); iF++)
                    {
                        if(F.Nleft == -1){
                            // 这里的realIdxF是指普通帧该节点中特征点的索引
                            const unsigned int realIdxF = vIndicesF[iF];

                            // 如果地图点存在，说明这个点已经被匹配过了，不再匹配，加快速度
                            if(vpMapPointMatches[realIdxF])
                                continue;
                            // 取出普通帧F中该特征对应的描述子
                            const cv::Mat &dF = F.mDescriptors.row(realIdxF);
                            // 计算描述子的距离
                            const int dist =  DescriptorDistance(dKF,dF);

                            // 遍历，记录最佳距离、最佳距离对应的索引、次佳距离等
                            // 如果 dist < bestDist1 < bestDist2，更新bestDist1 bestDist2
                            if(dist<bestDist1)
                            {
                                bestDist2=bestDist1;
                                bestDist1=dist;
                                bestIdxF=realIdxF;
                            }
                            // 如果bestDist1 < dist < bestDist2，更新bestDist2
                            else if(dist<bestDist2)
                            {
                                bestDist2=dist;
                            }
                        }
                        else{
                            const unsigned int realIdxF = vIndicesF[iF];

                            if(vpMapPointMatches[realIdxF])
                                continue;

                            const cv::Mat &dF = F.mDescriptors.row(realIdxF);

                            const int dist =  DescriptorDistance(dKF,dF);

                            if(realIdxF < F.Nleft && dist<bestDist1){
                                bestDist2=bestDist1;
                                bestDist1=dist;
                                bestIdxF=realIdxF;
                            }
                            else if(realIdxF < F.Nleft && dist<bestDist2){
                                bestDist2=dist;
                            }

                            if(realIdxF >= F.Nleft && dist<bestDist1R){
                                bestDist2R=bestDist1R;
                                bestDist1R=dist;
                                bestIdxFR=realIdxF;
                            }
                            else if(realIdxF >= F.Nleft && dist<bestDist2R){
                                bestDist2R=dist;
                            }
                        }

                    }
                    // Step 4：根据阈值 和 角度投票剔除误匹配
                    // Step 4.1：第一关筛选：匹配距离必须小于设定阈值
                    if(bestDist1<=TH_LOW)
                    {
                        // Step 4.2：第二关筛选：最佳匹配比次佳匹配明显要好，那么最佳匹配才真正靠谱
                        if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                        {
                            // Step 4.3：记录成功匹配特征点的对应的地图点(来自关键帧)
                            vpMapPointMatches[bestIdxF]=pMP;

                            // 这里的realIdxKF是当前遍历到的关键帧的特征点id
                            const cv::KeyPoint &kp =
                                    (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF] :
                                    (realIdxKF >= pKF -> NLeft) ? pKF -> mvKeysRight[realIdxKF - pKF -> NLeft]
                                                                : pKF -> mvKeys[realIdxKF];
                            // Step 4.4：计算匹配点旋转角度差所在的直方图
                            if(mbCheckOrientation)
                            {
                                cv::KeyPoint &Fkp =
                                        (!pKF->mpCamera2 || F.Nleft == -1) ? F.mvKeys[bestIdxF] :
                                        (bestIdxF >= F.Nleft) ? F.mvKeysRight[bestIdxF - F.Nleft]
                                                            : F.mvKeys[bestIdxF];
                                // 所有的特征点的角度变化应该是一致的，通过直方图统计得到最准确的角度变化值
                                float rot = kp.angle-Fkp.angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);// 将rot分配到bin组, 四舍五入, 其实就是离散到对应的直方图组中
                                if(bin==HISTO_LENGTH)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdxF);
                            }
                            nmatches++;
                        }

                        if(bestDist1R<=TH_LOW)
                        {
                            if(static_cast<float>(bestDist1R)<mfNNratio*static_cast<float>(bestDist2R) || true)
                            {
                                vpMapPointMatches[bestIdxFR]=pMP;

                                const cv::KeyPoint &kp =
                                        (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF] :
                                        (realIdxKF >= pKF -> NLeft) ? pKF -> mvKeysRight[realIdxKF - pKF -> NLeft]
                                                                    : pKF -> mvKeys[realIdxKF];

                                if(mbCheckOrientation)
                                {
                                    cv::KeyPoint &Fkp =
                                            (!F.mpCamera2) ? F.mvKeys[bestIdxFR] :
                                            (bestIdxFR >= F.Nleft) ? F.mvKeysRight[bestIdxFR - F.Nleft]
                                                                : F.mvKeys[bestIdxFR];

                                    float rot = kp.angle-Fkp.angle;
                                    if(rot<0.0)
                                        rot+=360.0f;
                                    int bin = round(rot*factor);
                                    if(bin==HISTO_LENGTH)
                                        bin=0;
                                    assert(bin>=0 && bin<HISTO_LENGTH);
                                    rotHist[bin].push_back(bestIdxFR);
                                }
                                nmatches++;
                            }
                        }
                    }

                }

                KFit++;
                Fit++;
            }
            else if(KFit->first < Fit->first)
            {
                // 对齐
                KFit = vFeatVecKF.lower_bound(Fit->first);
            }
            else
            {
                // 对齐
                Fit = F.mFeatVec.lower_bound(KFit->first);
            }
        }

        // Step 5 根据方向剔除误匹配的点
        if(mbCheckOrientation)
        {
            // index
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                // 如果特征点的旋转角度变化量属于这三个组，则保留
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;

                // 剔除掉不在前三的匹配对，因为他们不符合“主流旋转方向”  
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }

        return nmatches;
    }

    int ORBmatcher::SearchByProjection(KeyFrame* pKF, Sophus::Sim3f &Scw, const vector<MapPoint*> &vpPoints,
                                       vector<MapPoint*> &vpMatched, int th, float ratioHamming)
    {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Decompose Scw
        // Step 1 分解Sim变换矩阵
        //? 为什么要剥离尺度信息？
        Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(),Scw.translation()/Scw.scale());
        Eigen::Vector3f Ow = Tcw.inverse().translation();  // 世界坐标系下相机光心坐标

        // Set of MapPoints already found in the KeyFrame
        // 使用set类型，记录前面已经成功的匹配关系，避免重复匹配。并去除其中无效匹配关系（NULL）
        set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
        spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

        int nmatches=0;

        // For each Candidate MapPoint Project and Match
        // Step 2 遍历闭环KF及其共视KF的所有地图点（不考虑当前KF已经匹配的地图点）投影到当前KF
        for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
        {
            MapPoint* pMP = vpPoints[iMP];

            // Discard Bad MapPoints and already found
            // Step 2.1 丢弃坏点，跳过当前KF已经匹配上的地图点
            if(pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // Get 3D Coords.
            // Step 2.2 投影到当前KF的图像坐标并判断是否有效
            Eigen::Vector3f p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            Eigen::Vector3f p3Dc = Tcw * p3Dw;

            // Depth must be positive
            // 深度值必须为正
            if(p3Dc(2)<0.0)
                continue;

            // Project into Image
            const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

            // Point must be inside the image
            // 在图像范围内
            if(!pKF->IsInImage(uv(0),uv(1)))
                continue;

            // Depth must be inside the scale invariance region of the point
            // 判断距离是否在有效距离内
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            // 地图点到相机光心的向量
            Eigen::Vector3f PO = p3Dw-Ow;
            const float dist = PO.norm();

            if(dist<minDistance || dist>maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            // 观察角度小于60°
            Eigen::Vector3f Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist)
                continue;

            // 根据当前这个地图点距离当前KF光心的距离,预测该点在当前KF中的尺度(图层)
            int nPredictedLevel = pMP->PredictScale(dist,pKF);

            // Search in a radius
            // 根据尺度确定搜索半径
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            //  Step 2.3 搜索候选匹配点
            const vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0),uv(1),radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;
            //  Step 2.4 遍历候选匹配点，找到最佳匹配点
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;
                if(vpMatched[idx])
                    continue;

                const int &kpLevel= pKF->mvKeysUn[idx].octave;

                // 不在一个尺度也不行
                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            // 该MapPoint与bestIdx对应的特征点匹配成功
            if(bestDist<=TH_LOW*ratioHamming)
            {
                vpMatched[bestIdx]=pMP;
                nmatches++;
            }

        }
        //  Step 3 返回新的成功匹配的点对的数目
        return nmatches;
    }

    int ORBmatcher::SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs,
                                       std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming)
    {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(),Scw.translation()/Scw.scale());
        Eigen::Vector3f Ow = Tcw.inverse().translation();

        // Set of MapPoints already found in the KeyFrame
        set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
        spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

        int nmatches=0;

        // For each Candidate MapPoint Project and Match
        for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
        {
            MapPoint* pMP = vpPoints[iMP];
            KeyFrame* pKFi = vpPointsKFs[iMP];

            // Discard Bad MapPoints and already found
            if(pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // Get 3D Coords.
            Eigen::Vector3f p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            Eigen::Vector3f p3Dc = Tcw * p3Dw;

            // Depth must be positive
            if(p3Dc(2)<0.0)
                continue;

            // Project into Image
            const float invz = 1/p3Dc(2);
            const float x = p3Dc(0)*invz;
            const float y = p3Dc(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            if(!pKF->IsInImage(u,v))
                continue;

            // Depth must be inside the scale invariance region of the point
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            Eigen::Vector3f PO = p3Dw-Ow;
            const float dist = PO.norm();

            if(dist<minDistance || dist>maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            Eigen::Vector3f Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist)
                continue;

            int nPredictedLevel = pMP->PredictScale(dist,pKF);

            // Search in a radius
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;
                if(vpMatched[idx])
                    continue;

                const int &kpLevel= pKF->mvKeysUn[idx].octave;

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_LOW*ratioHamming)
            {
                vpMatched[bestIdx] = pMP;
                vpMatchedKF[bestIdx] = pKFi;
                nmatches++;
            }

        }

        return nmatches;
    }

    int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
    {
        int nmatches=0;
        // F1中特征点和F2中匹配关系，注意是按照F1特征点数目分配空间
        vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

        // Step 1 构建旋转直方图，HISTO_LENGTH = 30
        vector<int> rotHist[HISTO_LENGTH];
        // 每个bin里预分配500个，因为使用的是vector不够的话可以自动扩展容量
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        //! 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码
        // const float factor = HISTO_LENGTH/360.0f;
        const float factor = 1.0f/HISTO_LENGTH;


        // 匹配点对距离，注意是按照F2特征点数目分配空间
        vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
        // 从帧2到帧1的反向匹配，注意是按照F2特征点数目分配空间
        vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

        // 遍历帧1中的所有特征点
        for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
        {
            cv::KeyPoint kp1 = F1.mvKeysUn[i1];
            int level1 = kp1.octave;
            // 只使用原始图像上提取的特征点
            if(level1>0)
                continue;

            // Step 2 在半径窗口内搜索当前帧F2中所有的候选匹配特征点 
            // vbPrevMatched 输入的是参考帧 F1的特征点
            // windowSize = 100，输入最大最小金字塔层级 均为0
            vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

            // 没有候选特征点，跳过
            if(vIndices2.empty())
                continue;

            // 取出参考帧F1中当前遍历特征点对应的描述子
            cv::Mat d1 = F1.mDescriptors.row(i1);

            int bestDist = INT_MAX;     //最佳描述子匹配距离，越小越好
            int bestDist2 = INT_MAX;    //次佳描述子匹配距离
            int bestIdx2 = -1;          //最佳候选特征点在F2中的index

            // Step 3 遍历搜索搜索窗口中的所有潜在的匹配候选点，找到最优的和次优的
            for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
            {
                size_t i2 = *vit;
                // 取出候选特征点对应的描述子
                cv::Mat d2 = F2.mDescriptors.row(i2);
                // 计算两个特征点描述子距离
                int dist = DescriptorDistance(d1,d2);

                if(vMatchedDistance[i2]<=dist)
                    continue;
                // 如果当前匹配距离更小，更新最佳次佳距离
                if(dist<bestDist)
                {
                    bestDist2=bestDist;
                    bestDist=dist;
                    bestIdx2=i2;
                }
                else if(dist<bestDist2)
                {
                    bestDist2=dist;
                }
            }

            // Step 4 对最优次优结果进行检查，满足阈值、最优/次优比例，删除重复匹配
            // 即使算出了最佳描述子匹配距离，也不一定保证配对成功。要小于设定阈值
            if(bestDist<=TH_LOW)
            {
                // 最佳距离比次佳距离要小于设定的比例，这样特征点辨识度更高
                if(bestDist<(float)bestDist2*mfNNratio)
                {
                    // 如果找到的候选特征点对应F1中特征点已经匹配过了，说明发生了重复匹配，将原来的匹配也删掉
                    if(vnMatches21[bestIdx2]>=0)
                    {
                        vnMatches12[vnMatches21[bestIdx2]]=-1;
                        nmatches--;
                    }
                    // 次优的匹配关系，双向建立
                    // vnMatches12保存参考帧F1和F2匹配关系，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
                    vnMatches12[i1]=bestIdx2;
                    vnMatches21[bestIdx2]=i1;
                    vMatchedDistance[bestIdx2]=bestDist;
                    nmatches++;

                    // Step 5 计算匹配点旋转角度差所在的直方图
                    if(mbCheckOrientation)
                    {
                        // 计算匹配特征点的角度差，这里单位是角度°，不是弧度
                        float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        // 前面factor = HISTO_LENGTH/360.0f 
                        // bin = rot / 360.of * HISTO_LENGTH 表示当前rot被分配在第几个直方图bin  
                        int bin = round(rot*factor);
                        // 如果bin 满了又是一个轮回
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(i1);
                    }
                }
            }

        }

        // Step 6 筛除旋转直方图中“非主流”部分
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;
            // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;
                // 剔除掉不在前三的匹配对，因为他们不符合“主流旋转方向”    
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    int idx1 = rotHist[i][j];
                    if(vnMatches12[idx1]>=0)
                    {
                        vnMatches12[idx1]=-1;
                        nmatches--;
                    }
                }
            }

        }

        //Update prev matched
        // Step 7 将最后通过筛选的匹配好的特征点保存到vbPrevMatched
        for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
            if(vnMatches12[i1]>=0)
                vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

        return nmatches;
    }

    /*
    * @brief 通过词袋，对关键帧的特征点进行跟踪，该函数用于闭环检测时两个关键帧间的特征点匹配
    * @details 通过bow对pKF和F中的特征点进行快速匹配（不属于同一node的特征点直接跳过匹配） 
    * 对属于同一node的特征点通过描述子距离进行匹配 
    * 通过距离阈值、比例阈值和角度投票进行剔除误匹配
    * @param  pKF1               KeyFrame1
    * @param  pKF2               KeyFrame2
    * @param  vpMatches12        pKF2中与pKF1匹配的MapPoint，vpMatches12[i]表示匹配的地图点，null表示没有匹配，i表示匹配的pKF1 特征点索引
    * @return                    成功匹配的数量
    */
    int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
    {
        // Step 1 分别取出两个关键帧的特征点、BoW 向量、地图点、描述子
        const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
        const cv::Mat &Descriptors1 = pKF1->mDescriptors;

        const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
        const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
        const cv::Mat &Descriptors2 = pKF2->mDescriptors;

        // 保存匹配结果
        vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL));
        vector<bool> vbMatched2(vpMapPoints2.size(),false);

        // Step 2 构建旋转直方图，HISTO_LENGTH = 30
        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        //! 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码   
        // const float factor = HISTO_LENGTH/360.0f;
        const float factor = 1.0f/HISTO_LENGTH;

        int nmatches = 0;

        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        while(f1it != f1end && f2it != f2end)
        {
            // Step 3 开始遍历，分别取出属于同一node的特征点(只有属于同一node，才有可能是匹配点)
            if(f1it->first == f2it->first)
            {
                // 遍历KF中属于该node的特征点
                for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
                {
                    const size_t idx1 = f1it->second[i1];
                    if(pKF1 -> NLeft != -1 && idx1 >= pKF1 -> mvKeysUn.size()){
                        continue;
                    }

                    MapPoint* pMP1 = vpMapPoints1[idx1];
                    if(!pMP1)
                        continue;
                    if(pMP1->isBad())
                        continue;

                    const cv::Mat &d1 = Descriptors1.row(idx1);

                    int bestDist1=256;
                    int bestIdx2 =-1 ;
                    int bestDist2=256;

                    // Step 4 遍历KF2中属于该node的特征点，找到了最优及次优匹配点
                    for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                    {
                        const size_t idx2 = f2it->second[i2];

                        if(pKF2 -> NLeft != -1 && idx2 >= pKF2 -> mvKeysUn.size()){
                            continue;
                        }

                        MapPoint* pMP2 = vpMapPoints2[idx2];

                        // 如果已经有匹配的点，或者遍历到的特征点对应的地图点无效
                        if(vbMatched2[idx2] || !pMP2)
                            continue;

                        if(pMP2->isBad())
                            continue;

                        const cv::Mat &d2 = Descriptors2.row(idx2);

                        int dist = DescriptorDistance(d1,d2);

                        if(dist<bestDist1)
                        {
                            bestDist2=bestDist1;
                            bestDist1=dist;
                            bestIdx2=idx2;
                        }
                        else if(dist<bestDist2)
                        {
                            bestDist2=dist;
                        }
                    }

                    // Step 5 对匹配结果进行检查，满足阈值、最优/次优比例，记录旋转直方图信息
                    if(bestDist1<TH_LOW)
                    {
                        if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                        {
                            vpMatches12[idx1]=vpMapPoints2[bestIdx2];
                            vbMatched2[bestIdx2]=true;

                            if(mbCheckOrientation)
                            {
                                float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);
                                if(bin==HISTO_LENGTH)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LENGTH);
                                rotHist[bin].push_back(idx1);
                            }
                            nmatches++;
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            else if(f1it->first < f2it->first)
            {
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else
            {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        // Step 6 检查旋转直方图分布，剔除差异较大的匹配
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }

        return nmatches;
    }

    int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2,
                                           vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse)
    {
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

        //Compute epipole in second image
        // Step 1 计算KF1的相机中心在KF2图像平面的二维像素坐标
        Sophus::SE3f T1w = pKF1->GetPose();
        Sophus::SE3f T2w = pKF2->GetPose();
        Sophus::SE3f Tw2 = pKF2->GetPoseInverse(); // for convenience
        Eigen::Vector3f Cw = pKF1->GetCameraCenter();
        Eigen::Vector3f C2 = T2w * Cw;

        Eigen::Vector2f ep = pKF2->mpCamera->project(C2);
        Sophus::SE3f T12;
        Sophus::SE3f Tll, Tlr, Trl, Trr;
        Eigen::Matrix3f R12; // for fastest computation
        Eigen::Vector3f t12; // for fastest computation

        GeometricCamera* pCamera1 = pKF1->mpCamera, *pCamera2 = pKF2->mpCamera;

        if(!pKF1->mpCamera2 && !pKF2->mpCamera2){
            T12 = T1w * Tw2;
            R12 = T12.rotationMatrix();
            t12 = T12.translation();
        }
        else{
            Sophus::SE3f Tr1w = pKF1->GetRightPose();
            Sophus::SE3f Twr2 = pKF2->GetRightPoseInverse();
            Tll = T1w * Tw2;
            Tlr = T1w * Twr2;
            Trl = Tr1w * Tw2;
            Trr = Tr1w * Twr2;
        }
        Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr  = Tlr.rotationMatrix(), Rrl  = Trl.rotationMatrix(), Rrr  = Trr.rotationMatrix();
        Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(), trl = Trl.translation(), trr = Trr.translation();

        // Find matches between not tracked keypoints
        // Matching speed-up by ORB Vocabulary
        // Compare only ORB that share the same node

        int nmatches=0;
        // 记录匹配是否成功，避免重复匹配
        vector<bool> vbMatched2(pKF2->N,false);        
        vector<int> vMatches12(pKF1->N,-1);
        // 用于统计匹配点对旋转差的直方图
        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        //! 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码   
        // const float factor = HISTO_LENGTH/360.0f;
        const float factor = 1.0f/HISTO_LENGTH;

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        // Step 2 利用BoW加速匹配：只对属于同一节点(特定层)的ORB特征进行匹配
        // FeatureVector其实就是一个map类，那就可以直接获取它的迭代器进行遍历
        // FeatureVector的数据结构类似于：{(node1,feature_vector1) (node2,feature_vector2)...}
        // f1it->first对应node编号，f1it->second对应属于该node的所有特特征点编号
        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        // Step 2.1：遍历pKF1和pKF2中的node节点
        while(f1it!=f1end && f2it!=f2end)
        {
            // 如果f1it和f2it属于同一个node节点才会进行匹配，这就是BoW加速匹配原理
            if(f1it->first == f2it->first)
            {
                // Step 2.2：遍历属于同一node节点(id：f1it->first)下的所有特征点
                for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
                {
                    // 获取pKF1中属于该node节点的所有特征点索引
                    const size_t idx1 = f1it->second[i1];
                    
                    // Step 2.3：通过特征点索引idx1在pKF1中取出对应的MapPoint
                    MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
                    
                    // If there is already a MapPoint skip
                    // 由于寻找的是未匹配的特征点，所以pMP1应该为NULL
                    if(pMP1)
                    {
                        continue;
                    }
                    // 如果mvuRight中的值大于0，表示是双目，且该特征点有深度值
                    const bool bStereo1 = (!pKF1->mpCamera2 && pKF1->mvuRight[idx1]>=0);

                    if(bOnlyStereo)
                        if(!bStereo1)
                            continue;

                    // Step 2.4：通过特征点索引idx1在pKF1中取出对应的特征点
                    const cv::KeyPoint &kp1 = (pKF1 -> NLeft == -1) ? pKF1->mvKeysUn[idx1]
                                                                    : (idx1 < pKF1 -> NLeft) ? pKF1 -> mvKeys[idx1]
                                                                                            : pKF1 -> mvKeysRight[idx1 - pKF1 -> NLeft];

                    const bool bRight1 = (pKF1 -> NLeft == -1 || idx1 < pKF1 -> NLeft) ? false
                                                                                    : true;
                    //if(bRight1) continue;
                    // 通过特征点索引idx1在pKF1中取出对应的特征点的描述子
                    const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);
                    
                    int bestDist = TH_LOW;
                    int bestIdx2 = -1;
                    
                    // Step 2.5：遍历该node节点下(f2it->first)对应KF2中的所有特征点
                    for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                    {
                        // 获取pKF2中属于该node节点的所有特征点索引
                        size_t idx2 = f2it->second[i2];
                        
                        // 通过特征点索引idx2在pKF2中取出对应的MapPoint
                        MapPoint* pMP2 = pKF2->GetMapPoint(idx2);
                        
                        // If we have already matched or there is a MapPoint skip
                        // 如果pKF2当前特征点索引idx2已经被匹配过或者对应的3d点非空，那么跳过这个索引idx2
                        if(vbMatched2[idx2] || pMP2)
                            continue;

                        const bool bStereo2 = (!pKF2->mpCamera2 &&  pKF2->mvuRight[idx2]>=0);

                        if(bOnlyStereo)
                            if(!bStereo2)
                                continue;
                        
                        // 通过特征点索引idx2在pKF2中取出对应的特征点的描述子
                        const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);
                        
                        // Step 2.6 计算idx1与idx2在两个关键帧中对应特征点的描述子距离
                        const int dist = DescriptorDistance(d1,d2);
                        
                        if(dist>TH_LOW || dist>bestDist)
                            continue;

                        // 通过特征点索引idx2在pKF2中取出对应的特征点
                        const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                                        : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                                : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];
                        const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                                        : true;
                        //? 为什么双目就不需要判断像素点到极点的距离的判断？
                        // 因为双目模式下可以左右互匹配恢复三维点
                        if(!bStereo1 && !bStereo2 && !pKF1->mpCamera2)
                        {
                            const float distex = ep(0)-kp2.pt.x;
                            const float distey = ep(1)-kp2.pt.y;
                            // Step 2.7 极点e2到kp2的像素距离如果小于阈值th,认为kp2对应的MapPoint距离pKF1相机太近，跳过该匹配点对
                            // 作者根据kp2金字塔尺度因子(scale^n，scale=1.2，n为层数)定义阈值th
                            // 金字塔层数从0到7，对应距离 sqrt(100*pKF2->mvScaleFactors[kp2.octave]) 是10-20个像素
                            //? 对这个阈值的有效性持怀疑态度
                            if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                            {
                                continue;
                            }
                        }

                        if(pKF1->mpCamera2 && pKF2->mpCamera2){
                            if(bRight1 && bRight2){
                                R12 = Rrr;
                                t12 = trr;
                                T12 = Trr;

                                pCamera1 = pKF1->mpCamera2;
                                pCamera2 = pKF2->mpCamera2;
                            }
                            else if(bRight1 && !bRight2){
                                R12 = Rrl;
                                t12 = trl;
                                T12 = Trl;

                                pCamera1 = pKF1->mpCamera2;
                                pCamera2 = pKF2->mpCamera;
                            }
                            else if(!bRight1 && bRight2){
                                R12 = Rlr;
                                t12 = tlr;
                                T12 = Tlr;

                                pCamera1 = pKF1->mpCamera;
                                pCamera2 = pKF2->mpCamera2;
                            }
                            else{
                                R12 = Rll;
                                t12 = tll;
                                T12 = Tll;

                                pCamera1 = pKF1->mpCamera;
                                pCamera2 = pKF2->mpCamera;
                            }

                        }

                        // Step 2.8 计算特征点kp2到kp1对应极线的距离是否小于阈值
                        if(bCoarse || pCamera1->epipolarConstrain(pCamera2,kp1,kp2,R12,t12,pKF1->mvLevelSigma2[kp1.octave],pKF2->mvLevelSigma2[kp2.octave])) // MODIFICATION_2
                        {
                            // bestIdx2，bestDist 是 kp1 对应 KF2中的最佳匹配点 index及匹配距离
                            bestIdx2 = idx2;
                            bestDist = dist;
                        }
                    }
                    
                    if(bestIdx2>=0)
                    {
                        const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[bestIdx2]
                                                                        : (bestIdx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[bestIdx2]
                                                                                        : pKF2 -> mvKeysRight[bestIdx2 - pKF2 -> NLeft];
                        // 记录匹配结果 
                        vMatches12[idx1]=bestIdx2;
                        // !记录已经匹配，避免重复匹配。原作者漏掉！可以添加下面代码
                        // vbMatched2[bestIdx2]=true;
                        nmatches++;

                        // 记录旋转差直方图信息
                        if(mbCheckOrientation)
                        {
                            // angle：角度，表示匹配点对的方向差。
                            float rot = kp1.angle-kp2.angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            else if(f1it->first < f2it->first)
            {
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else
            {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        // Step 3 用旋转差直方图来筛掉错误匹配对
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {              
                    vbMatched2[vMatches12[rotHist[i][j]]] = false;  // !清除匹配关系。原作者漏掉！
                    vMatches12[rotHist[i][j]]=-1;
                    nmatches--;
                }
            }

        }

        // Step 4 存储匹配关系，下标是关键帧1的特征点id，存储的是关键帧2的特征点id
        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);

        for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
        {
            if(vMatches12[i]<0)
                continue;
            vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
        }

        return nmatches;
    }

    int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th, const bool bRight)
    {
        GeometricCamera* pCamera;
        Sophus::SE3f Tcw;
        Eigen::Vector3f Ow;

        if(bRight){
            Tcw = pKF->GetRightPose();
            Ow = pKF->GetRightCameraCenter();
            pCamera = pKF->mpCamera2;
        }
        else{
            Tcw = pKF->GetPose();
            Ow = pKF->GetCameraCenter();
            pCamera = pKF->mpCamera;
        }

        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;
        const float &bf = pKF->mbf;

        int nFused=0;

        const int nMPs = vpMapPoints.size();

        // For debbuging
        int count_notMP = 0, count_bad=0, count_isinKF = 0, count_negdepth = 0, count_notinim = 0, count_dist = 0, count_normal=0, count_notidx = 0, count_thcheck = 0;
        // 遍历所有的待投影地图点
        for(int i=0; i<nMPs; i++)
        {
            // Step 1 判断地图点的有效性 
            MapPoint* pMP = vpMapPoints[i];

            if(!pMP)
            {
                count_notMP++;
                continue;
            }
            // 地图点无效 或 已经是该帧的地图点（无需融合），跳过
            if(pMP->isBad())
            {
                count_bad++;
                continue;
            }
            else if(pMP->IsInKeyFrame(pKF))
            {
                count_isinKF++;
                continue;
            }

            // 将地图点变换到关键帧的相机坐标系下
            Eigen::Vector3f p3Dw = pMP->GetWorldPos();
            Eigen::Vector3f p3Dc = Tcw * p3Dw;

            // Depth must be positive
            // 深度值为负，跳过
            if(p3Dc(2)<0.0f)
            {
                count_negdepth++;
                continue;
            }
            // Step 2 得到地图点投影到关键帧的图像坐标
            const float invz = 1/p3Dc(2);

            const Eigen::Vector2f uv = pCamera->project(p3Dc);

            // Point must be inside the image
            // 投影点需要在有效范围内
            if(!pKF->IsInImage(uv(0),uv(1)))
            {
                count_notinim++;
                continue;
            }

            const float ur = uv(0)-bf*invz;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            Eigen::Vector3f PO = p3Dw-Ow;
            const float dist3D = PO.norm();

            // Depth must be inside the scale pyramid of the image
            // Step 3 地图点到关键帧相机光心距离需满足在有效范围内
            if(dist3D<minDistance || dist3D>maxDistance) {
                count_dist++;
                continue;
            }

            // Viewing angle must be less than 60 deg
            // Step 4 地图点的平均观测方向（正视程度）要小于60°
            Eigen::Vector3f Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist3D)
            {
                count_normal++;
                continue;
            }

            int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

            // Search in a radius
            // 根据MapPoint的深度确定尺度，从而确定搜索范围
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
            // Step 5 在投影点附近搜索窗口内找到候选匹配点的索引
            const vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0),uv(1),radius,bRight);

            if(vIndices.empty())
            {
                count_notidx++;
                continue;
            }

            // Match to the most similar keypoint in the radius
            // Step 6 遍历寻找最佳匹配点（最小描述子距离）
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                size_t idx = *vit;
                const cv::KeyPoint &kp = (pKF -> NLeft == -1) ? pKF->mvKeysUn[idx]
                                                            : (!bRight) ? pKF -> mvKeys[idx]
                                                                        : pKF -> mvKeysRight[idx];

                const int &kpLevel= kp.octave;
                // 金字塔层级要接近（同一层或小一层），否则跳过
                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                // 计算投影点与候选匹配特征点的距离，如果偏差很大，直接跳过
                if(pKF->mvuRight[idx]>=0)
                {
                    // Check reprojection error in stereo
                    // 双目情况
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float &kpr = pKF->mvuRight[idx];
                    const float ex = uv(0)-kpx;
                    const float ey = uv(1)-kpy;
                    // 右目数据的偏差也要考虑进去
                    const float er = ur-kpr;
                    const float e2 = ex*ex+ey*ey+er*er;

                    //自由度为3, 误差小于1个像素,这种事情95%发生的概率对应卡方检验阈值为7.82
                    if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
                        continue;
                }
                else
                {
                    // 单目情况
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float ex = uv(0)-kpx;
                    const float ey = uv(1)-kpy;
                    const float e2 = ex*ex+ey*ey;

                    // 自由度为2的，卡方检验阈值5.99（假设测量有一个像素的偏差）
                    if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                        continue;
                }

                if(bRight) idx += pKF->NLeft;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)// 找MapPoint在该区域最佳匹配的特征点
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            // If there is already a MapPoint replace otherwise add new measurement
            // Step 7 找到投影点对应的最佳匹配特征点，根据是否存在地图点来融合
            // 最佳匹配距离要小于阈值
            if(bestDist<=TH_LOW)
            {
                MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
                if(pMPinKF)
                {
                    // 如果最佳匹配点有对应有效地图点，选择被观测次数最多的那个替换
                    if(!pMPinKF->isBad())
                    {
                        if(pMPinKF->Observations()>pMP->Observations())
                            pMP->Replace(pMPinKF);
                        else
                            pMPinKF->Replace(pMP);
                    }
                }
                else
                {
                    // 如果最佳匹配点没有对应地图点，添加观测信息
                    pMP->AddObservation(pKF,bestIdx);
                    pKF->AddMapPoint(pMP,bestIdx);
                }
                nFused++;
            }
            else
                count_thcheck++;

        }

        return nFused;
    }


    /**
     * @brief 闭环矫正中使用。将当前关键帧闭环匹配上的关键帧及其共视关键帧组成的地图点投影到当前关键帧，融合地图点
     * 
     * @param[in] pKF                   当前关键帧
     * @param[in] Scw                   当前关键帧经过闭环Sim3 后的世界到相机坐标系的Sim变换
     * @param[in] vpPoints              与当前关键帧闭环匹配上的关键帧及其共视关键帧组成的地图点
     * @param[in] th                    搜索范围系数
     * @param[out] vpReplacePoint       替换的地图点
     * @return int                      融合（替换和新增）的地图点数目
     */
    int ORBmatcher::Fuse(KeyFrame *pKF, Sophus::Sim3f &Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)
    {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Decompose Scw
        // Step 1 将Sim3转化为SE3并分解
        Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(),Scw.translation()/Scw.scale());
        Eigen::Vector3f Ow = Tcw.inverse().translation();

        // Set of MapPoints already found in the KeyFrame
        // 当前帧已有的匹配地图点
        const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

        int nFused=0;
        // 与当前帧闭环匹配上的关键帧及其共视关键帧组成的地图点
        const int nPoints = vpPoints.size();

        // For each candidate MapPoint project and match
        // 遍历所有的地图点
        for(int iMP=0; iMP<nPoints; iMP++)
        {
            MapPoint* pMP = vpPoints[iMP];

            // Discard Bad MapPoints and already found
            // 地图点无效 或 已经是该帧的地图点（无需融合），跳过
            if(pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // Get 3D Coords.
            // Step 2 地图点变换到当前相机坐标系下
            Eigen::Vector3f p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            Eigen::Vector3f p3Dc = Tcw * p3Dw;

            // Depth must be positive
            if(p3Dc(2)<0.0f)
                continue;

            // Project into Image
            // Step 3 得到地图点投影到当前帧的图像坐标
            const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

            // Point must be inside the image
            // 投影点必须在图像范围内
            if(!pKF->IsInImage(uv(0),uv(1)))
                continue;

            // Depth must be inside the scale pyramid of the image
            // Step 4 根据距离是否在图像合理金字塔尺度范围内和观测角度是否小于60度判断该地图点是否有效
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            Eigen::Vector3f PO = p3Dw-Ow;
            const float dist3D = PO.norm();

            if(dist3D<minDistance || dist3D>maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            Eigen::Vector3f Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist3D)
                continue;

            // Compute predicted scale level
            const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

            // Search in a radius
            // 计算搜索范围
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            // Step 5 在当前帧内搜索匹配候选点
            const vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0),uv(1),radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            // Step 6 寻找最佳匹配点（没有用到次佳匹配的比例）
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
            {
                const size_t idx = *vit;
                const int &kpLevel = pKF->mvKeysUn[idx].octave;

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            // If there is already a MapPoint replace otherwise add new measurement
            // Step 7 替换或新增地图点
            if(bestDist<=TH_LOW)
            {
                MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
                if(pMPinKF)
                {
                    // 如果这个地图点已经存在，则记录要替换信息
                    // 这里不能直接替换，原因是需要对地图点加锁后才能替换，否则可能会crash。所以先记录，在加锁后替换
                    if(!pMPinKF->isBad())
                        vpReplacePoint[iMP] = pMPinKF;
                }
                else
                {
                    // 如果这个地图点不存在，直接添加
                    pMP->AddObservation(pKF,bestIdx);
                    pKF->AddMapPoint(pMP,bestIdx);
                }
                nFused++;
            }
        }
        // 融合（替换和新增）的地图点数目
        return nFused;
    }

    /**
     * @brief 通过Sim3变换，搜索两个关键帧中更多的匹配点对
     * （之前使用SearchByBoW进行特征点匹配时会有漏匹配）
     * @param[in] pKF1              当前帧
     * @param[in] pKF2              闭环候选帧
     * @param[in] vpMatches12       i表示匹配的pKF1 特征点索引，vpMatches12[i]表示匹配的地图点，null表示没有匹配
     * @param[in] s12               pKF2 到 pKF1 的Sim 变换中的尺度
     * @param[in] R12               pKF2 到 pKF1 的Sim 变换中的旋转矩阵
     * @param[in] t12               pKF2 到 pKF1 的Sim 变换中的平移向量
     * @param[in] th                搜索窗口的倍数
     * @return int                  新增的匹配点对数目
     */
    int ORBmatcher::SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const Sophus::Sim3f &S12, const float th)
    {
        // Step 1： 准备工作
        const float &fx = pKF1->fx;
        const float &fy = pKF1->fy;
        const float &cx = pKF1->cx;
        const float &cy = pKF1->cy;

        // 从world到camera1的变换
        // 从world到camera2的变换
        // Camera 1 & 2 from world
        Sophus::SE3f T1w = pKF1->GetPose();
        Sophus::SE3f T2w = pKF2->GetPose();

        //Transformation between cameras
        // Sim3 的逆
        Sophus::Sim3f S21 = S12.inverse();

        // 取出关键帧中的地图点
        const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
        const int N1 = vpMapPoints1.size();

        const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
        const int N2 = vpMapPoints2.size();

        // pKF1中特征点的匹配情况，有匹配为true，否则false
        vector<bool> vbAlreadyMatched1(N1,false);
        // pKF2中特征点的匹配情况，有匹配为true，否则false
        vector<bool> vbAlreadyMatched2(N2,false);

        // Step 2：记录已经匹配的特征点
        for(int i=0; i<N1; i++)
        {
            MapPoint* pMP = vpMatches12[i];
            if(pMP)
            {
                // pKF1中第i个特征点已经匹配成功
                vbAlreadyMatched1[i]=true;
                // 得到该地图点在关键帧pkF2 中的id
                int idx2 = get<0>(pMP->GetIndexInKeyFrame(pKF2));
                if(idx2>=0 && idx2<N2)
                    // pKF2中第idx2个特征点在pKF1中有匹配
                    vbAlreadyMatched2[idx2]=true;
            }
        }

        vector<int> vnMatch1(N1,-1);
        vector<int> vnMatch2(N2,-1);

        // Transform from KF1 to KF2 and search
        // Step 3：通过Sim变换，寻找 pKF1 中特征点和 pKF2 中的新的匹配
        // 之前使用SearchByBoW进行特征点匹配时会有漏匹配
        for(int i1=0; i1<N1; i1++)
        {
            MapPoint* pMP = vpMapPoints1[i1];

            // 该特征点已经有匹配点了，直接跳过
            if(!pMP || vbAlreadyMatched1[i1])
                continue;

            if(pMP->isBad())
                continue;

            // Step 3.1：通过Sim变换，将pKF1的地图点投影到pKF2中的图像坐标
            Eigen::Vector3f p3Dw = pMP->GetWorldPos();
            // 把pKF1的地图点从world坐标系变换到camera1坐标系
            Eigen::Vector3f p3Dc1 = T1w * p3Dw;
            // 再通过Sim3将该地图点从camera1变换到camera2坐标系
            Eigen::Vector3f p3Dc2 = S21 * p3Dc1;

            // Depth must be positive
            if(p3Dc2(2)<0.0)
                continue;

            // 投影到camera2图像坐标 (u,v)
            const float invz = 1.0/p3Dc2(2);
            const float x = p3Dc2(0)*invz;
            const float y = p3Dc2(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            if(!pKF2->IsInImage(u,v))
                continue;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const float dist3D = p3Dc2.norm();

            // Depth must be inside the scale invariance region
            if(dist3D<minDistance || dist3D>maxDistance )
                continue;

            // Compute predicted octave
            // Step 3.2：预测投影的点在图像金字塔哪一层
            const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);

            // Search in a radius
            // 计算特征点搜索半径
            const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];

            // Step 3.3：搜索该区域内的所有候选匹配特征点
            const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            // Step 3.4：遍历所有候选特征点，寻找最佳匹配点（并未使用次佳最佳比例约束）
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

                if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_HIGH)
            {
                vnMatch1[i1]=bestIdx;
            }
        }

        // Transform from KF2 to KF1 and search
        // Step 4：通过Sim变换，寻找 pKF2 中特征点和 pKF1 中的新的匹配
        // 具体步骤同上
        for(int i2=0; i2<N2; i2++)
        {
            MapPoint* pMP = vpMapPoints2[i2];

            if(!pMP || vbAlreadyMatched2[i2])
                continue;

            if(pMP->isBad())
                continue;

            Eigen::Vector3f p3Dw = pMP->GetWorldPos();
            Eigen::Vector3f p3Dc2 = T2w * p3Dw;
            Eigen::Vector3f p3Dc1 = S12 * p3Dc2;

            // Depth must be positive
            if(p3Dc1(2)<0.0)
                continue;

            const float invz = 1.0/p3Dc1(2);
            const float x = p3Dc1(0)*invz;
            const float y = p3Dc1(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            if(!pKF1->IsInImage(u,v))
                continue;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const float dist3D = p3Dc1.norm();

            // Depth must be inside the scale pyramid of the image
            if(dist3D<minDistance || dist3D>maxDistance)
                continue;

            // Compute predicted octave
            const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);

            // Search in a radius of 2.5*sigma(ScaleLevel)
            const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

                if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_HIGH)
            {
                vnMatch2[i2]=bestIdx;
            }
        }

        // Check agreement
        // Step 5： 一致性检查,只有在两次互相匹配中都出现才能够认为是可靠的匹配
        int nFound = 0;

        for(int i1=0; i1<N1; i1++)
        {
            int idx2 = vnMatch1[i1];

            if(idx2>=0)
            {
                int idx1 = vnMatch2[idx2];
                if(idx1==i1)
                {
                    // 更新匹配的地图点
                    vpMatches12[i1] = vpMapPoints2[idx2];
                    nFound++;
                }
            }
        }

        return nFound;
    }

    /**
     * @brief 将上一帧跟踪的地图点投影到当前帧，并且搜索匹配点。用于跟踪前一帧
     * 步骤
     * Step 1 建立旋转直方图，用于检测旋转一致性
     * Step 2 计算当前帧和前一帧的平移向量
     * Step 3 对于前一帧的每一个地图点，通过相机投影模型，得到投影到当前帧的像素坐标
     * Step 4 根据相机的前后前进方向来判断搜索尺度范围
     * Step 5 遍历候选匹配点，寻找距离最小的最佳匹配点 
     * Step 6 计算匹配点旋转角度差所在的直方图
     * Step 7 进行旋转一致检测，剔除不一致的匹配
     * @param[in] CurrentFrame          当前帧
     * @param[in] LastFrame             上一帧
     * @param[in] th                    搜索范围阈值，默认单目为7，双目15
     * @param[in] bMono                 是否为单目
     * @return int                      成功匹配的数量
     */
    int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
    {
        int nmatches = 0;

        // Rotation Histogram (to check rotation consistency)
        // Step 1 建立旋转直方图，用于检测旋转一致性
        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        //! 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码
        // const float factor = HISTO_LENGTH/360.0f;
        const float factor = 1.0f/HISTO_LENGTH;

        // Step 2 计算当前帧和前一帧的平移向量
        //当前帧的相机位姿
        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        const Eigen::Vector3f twc = Tcw.inverse().translation();

        const Sophus::SE3f Tlw = LastFrame.GetPose();
        const Eigen::Vector3f tlc = Tlw * twc; 

        // 判断前进还是后退
        const bool bForward = tlc(2)>CurrentFrame.mb && !bMono;     // 非单目情况，如果Z大于基线，则表示相机明显前进
        const bool bBackward = -tlc(2)>CurrentFrame.mb && !bMono;   // 非单目情况，如果-Z小于基线，则表示相机明显后退

        //  Step 3 对于前一帧的每一个地图点，通过相机投影模型，得到投影到当前帧的像素坐标
        for(int i=0; i<LastFrame.N; i++)
        {
            MapPoint* pMP = LastFrame.mvpMapPoints[i];

            if(pMP)
            {
                if(!LastFrame.mvbOutlier[i])
                {
                    // 对上一帧有效的MapPoints投影到当前帧坐标系
                    Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                    Eigen::Vector3f x3Dc = Tcw * x3Dw;

                    const float xc = x3Dc(0);
                    const float yc = x3Dc(1);
                    const float invzc = 1.0/x3Dc(2);

                    if(invzc<0)
                        continue;

                    // 投影到当前帧中
                    Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

                    if(uv(0)<CurrentFrame.mnMinX || uv(0)>CurrentFrame.mnMaxX)
                        continue;
                    if(uv(1)<CurrentFrame.mnMinY || uv(1)>CurrentFrame.mnMaxY)
                        continue;
                    // 认为投影前后地图点的尺度信息不变
                    int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave
                                                                                    : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

                    // Search in a window. Size depends on scale
                    // 单目：th = 7，双目：th = 15
                    float radius = th*CurrentFrame.mvScaleFactors[nLastOctave]; // 尺度越大，搜索范围越大

                    // 记录候选匹配点的id
                    vector<size_t> vIndices2;

                    // Step 4 根据相机的前后前进方向来判断搜索尺度范围。
                    // 以下可以这么理解，例如一个有一定面积的圆点，在某个尺度n下它是一个特征点
                    // 当相机前进时，圆点的面积增大，在某个尺度m下它是一个特征点，由于面积增大，则需要在更高的尺度下才能检测出来
                    // 当相机后退时，圆点的面积减小，在某个尺度m下它是一个特征点，由于面积减小，则需要在更低的尺度下才能检测出来
                    if(bForward)  // 前进,则上一帧兴趣点在所在的尺度nLastOctave<=nCurOctave
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave);
                    else if(bBackward)  // 后退,则上一帧兴趣点在所在的尺度0<=nCurOctave<=nLastOctave
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, 0, nLastOctave);
                    else  // 在[nLastOctave-1, nLastOctave+1]中搜索
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave-1, nLastOctave+1);

                    if(vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    // Step 5 遍历候选匹配点，寻找距离最小的最佳匹配点 
                    for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                    {
                        const size_t i2 = *vit;

                        // 如果该特征点已经有对应的MapPoint了,则退出该次循环
                        if(CurrentFrame.mvpMapPoints[i2])
                            if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                                continue;

                        if(CurrentFrame.Nleft == -1 && CurrentFrame.mvuRight[i2]>0)
                        {
                            // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                            const float ur = uv(0) - CurrentFrame.mbf*invzc;
                            const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                            if(er>radius)
                                continue;
                        }

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP,d);

                        if(dist<bestDist)
                        {
                            bestDist=dist;
                            bestIdx2=i2;
                        }
                    }

                    // 最佳匹配距离要小于设定阈值
                    if(bestDist<=TH_HIGH)
                    {
                        CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                        nmatches++;

                        // Step 6 计算匹配点旋转角度差所在的直方图
                        if(mbCheckOrientation)
                        {
                            cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                                                                        : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
                                                                                                : LastFrame.mvKeysRight[i - LastFrame.Nleft];

                            cv::KeyPoint kpCF = (CurrentFrame.Nleft == -1) ? CurrentFrame.mvKeysUn[bestIdx2]
                                                                        : (bestIdx2 < CurrentFrame.Nleft) ? CurrentFrame.mvKeys[bestIdx2]
                                                                                                            : CurrentFrame.mvKeysRight[bestIdx2 - CurrentFrame.Nleft];
                            float rot = kpLF.angle-kpCF.angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }
                    if(CurrentFrame.Nleft != -1){
                        Eigen::Vector3f x3Dr = CurrentFrame.GetRelativePoseTrl() * x3Dc;
                        Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dr);

                        int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave
                                                                                        : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

                        // Search in a window. Size depends on scale
                        float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                        vector<size_t> vIndices2;

                        if(bForward)
                            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave, -1,true);
                        else if(bBackward)
                            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, 0, nLastOctave, true);
                        else
                            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave-1, nLastOctave+1, true);

                        const cv::Mat dMP = pMP->GetDescriptor();

                        int bestDist = 256;
                        int bestIdx2 = -1;

                        for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                        {
                            const size_t i2 = *vit;
                            if(CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft])
                                if(CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft]->Observations()>0)
                                    continue;

                            const cv::Mat &d = CurrentFrame.mDescriptors.row(i2 + CurrentFrame.Nleft);

                            const int dist = DescriptorDistance(dMP,d);

                            if(dist<bestDist)
                            {
                                bestDist=dist;
                                bestIdx2=i2;
                            }
                        }

                        if(bestDist<=TH_HIGH)
                        {
                            CurrentFrame.mvpMapPoints[bestIdx2 + CurrentFrame.Nleft]=pMP;
                            nmatches++;
                            if(mbCheckOrientation)
                            {
                                cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                                                                            : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
                                                                                                    : LastFrame.mvKeysRight[i - LastFrame.Nleft];

                                cv::KeyPoint kpCF = CurrentFrame.mvKeysRight[bestIdx2];

                                float rot = kpLF.angle-kpCF.angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);
                                if(bin==HISTO_LENGTH)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdx2  + CurrentFrame.Nleft);
                            }
                        }

                    }
                }
            }
        }

        //Apply rotation consistency
        //  Step 7 进行旋转一致检测，剔除不一致的匹配
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                // 对于数量不是前3个的点对，剔除
                if(i!=ind1 && i!=ind2 && i!=ind3)
                {
                    for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                        nmatches--;
                    }
                }
            }
        }

        return nmatches;
    }

    /**
     * @brief 通过投影的方式将关键帧中未匹配的地图点投影到当前帧中,进行匹配，并通过旋转直方图进行筛选
     * 
     * @param[in] CurrentFrame          当前帧
     * @param[in] pKF                   参考关键帧
     * @param[in] sAlreadyFound         已经找到的地图点集合，不会用于PNP
     * @param[in] th                    匹配时搜索范围，会乘以金字塔尺度
     * @param[in] ORBdist               匹配的ORB描述子距离应该小于这个阈值    
     * @return int                      成功匹配的数量
     */
    int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)
    {
        int nmatches = 0;

        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        Eigen::Vector3f Ow = Tcw.inverse().translation();

        // Rotation Histogram (to check rotation consistency)
        // Step 1 建立旋转直方图，用于检测旋转一致性
        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;

        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        // Step 2 遍历关键帧中的每个地图点，通过相机投影模型，得到投影到当前帧的像素坐标
        for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMPs[i];

            if(pMP)
            {
                // 地图点存在 并且 不在已有地图点集合里
                if(!pMP->isBad() && !sAlreadyFound.count(pMP))
                {
                    //Project
                    Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                    Eigen::Vector3f x3Dc = Tcw * x3Dw;

                    const Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

                    if(uv(0)<CurrentFrame.mnMinX || uv(0)>CurrentFrame.mnMaxX)
                        continue;
                    if(uv(1)<CurrentFrame.mnMinY || uv(1)>CurrentFrame.mnMaxY)
                        continue;

                    // Compute predicted scale level
                    Eigen::Vector3f PO = x3Dw-Ow;
                    float dist3D = PO.norm();

                    const float maxDistance = pMP->GetMaxDistanceInvariance();
                    const float minDistance = pMP->GetMinDistanceInvariance();

                    // Depth must be inside the scale pyramid of the image
                    if(dist3D<minDistance || dist3D>maxDistance)
                        continue;

                    //预测尺度
                    int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);

                    // Search in a window
                    // 搜索半径和尺度相关
                    const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

                    //  Step 3 搜索候选匹配点
                    const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nPredictedLevel-1, nPredictedLevel+1);

                    if(vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;
                    // Step 4 遍历候选匹配点，寻找距离最小的最佳匹配点 
                    for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                    {
                        const size_t i2 = *vit;
                        if(CurrentFrame.mvpMapPoints[i2])
                            continue;

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP,d);

                        if(dist<bestDist)
                        {
                            bestDist=dist;
                            bestIdx2=i2;
                        }
                    }

                    if(bestDist<=ORBdist)
                    {
                        CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                        nmatches++;
                        // Step 5 计算匹配点旋转角度差所在的直方图
                        if(mbCheckOrientation)
                        {
                            float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }

                }
            }
        }

        //  Step 6 进行旋转一致检测，剔除不一致的匹配
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i!=ind1 && i!=ind2 && i!=ind3)
                {
                    for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                        nmatches--;
                    }
                }
            }
        }

        return nmatches;
    }

    /**
     * @brief 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
     * 
     * @param[in] histo         匹配特征点对旋转方向差直方图
     * @param[in] L             直方图尺寸
     * @param[in & out] ind1          bin值第一大对应的索引
     * @param[in & out] ind2          bin值第二大对应的索引
     * @param[in & out] ind3          bin值第三大对应的索引
     */
    void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1=0;
        int max2=0;
        int max3=0;

        for(int i=0; i<L; i++)
        {
            const int s = histo[i].size();
            if(s>max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s>max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s>max3)
            {
                max3=s;
                ind3=i;
            }
        }

        // 如果差距太大了,说明次优的非常不好,这里就索性放弃了,都置为-1
        if(max2<0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3<0.1f*(float)max1)
        {
            ind3=-1;
        }
    }


    // Bit set count operation from
    // Hamming distance：两个二进制串之间的汉明距离，指的是其不同位数的个数
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist=0;

        // 8*32=256bit

        for(int i=0; i<8; i++, pa++, pb++)
        {
            unsigned  int v = *pa ^ *pb;        // 相等为0,不等为1
            // 下面的操作就是计算其中bit为1的个数了,这个操作看上面的链接就好
            // 其实我觉得也还阔以直接使用8bit的查找表,然后做32次寻址操作就完成了;不过缺点是没有利用好CPU的字长
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

} //namespace ORB_SLAM
