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

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3
{

Atlas::Atlas()
{
    mpCurrentMap = static_cast<Map *>(NULL);
}

Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid), mHasViewer(false)
{
    mpCurrentMap = static_cast<Map *>(NULL);
    CreateNewMap();
}

Atlas::~Atlas()
{
    for (std::set<Map *>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map *pMi = *it;

        if (pMi)
        {
            delete pMi;
            pMi = static_cast<Map *>(NULL);

            it = mspMaps.erase(it);
        }
        else
            ++it;
    }
}

/**
 * @brief 创建新地图，如果当前活跃地图有效，先存储当前地图为不活跃地图，然后新建地图；否则，可以直接新建地图。
 * 
 */
void Atlas::CreateNewMap()
{
    // 锁住地图集
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId << endl;
    // 如果当前活跃地图有效，先存储当前地图为不活跃地图后退出
    if (mpCurrentMap)
    {
        // mnLastInitKFidMap为当前地图创建时第1个关键帧的id，它是在上一个地图最大关键帧id的基础上增加1
        if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1; // The init KF is the next of current maximum

        // 将当前地图储存起来，其实就是把mIsInUse标记为false
        mpCurrentMap->SetStoredMap();
        cout << "Stored map with ID: " << mpCurrentMap->GetId() << endl;

        // if(mHasViewer)
        //     mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);  //新建地图
    mpCurrentMap->SetCurrentMap();              //设置为活跃地图
    mspMaps.insert(mpCurrentMap);               //插入地图集
}

void Atlas::ChangeMap(Map *pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Change to map with id: " << pMap->GetId() << endl;
    if (mpCurrentMap)
    {
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame *pKF)
{
    Map *pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint *pMP)
{
    Map *pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

/**
 * @brief 添加相机，跟保存地图相关
 * @param pCam 相机
 */
GeometricCamera *Atlas::AddCamera(GeometricCamera *pCam)
{
    // Check if the camera already exists
    bool bAlreadyInMap = false;
    int index_cam = -1;
    // 遍历地图中现有的相机看看跟输入的相机一不一样，不一样的话则向mvpCameras添加
    for (size_t i = 0; i < mvpCameras.size(); ++i)
    {
        GeometricCamera *pCam_i = mvpCameras[i];
        if (!pCam)
            std::cout << "Not pCam" << std::endl;
        if (!pCam_i)
            std::cout << "Not pCam_i" << std::endl;
        if (pCam->GetType() != pCam_i->GetType())
            continue;

        if (pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            if (((Pinhole *)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
        else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            if (((KannalaBrandt8 *)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
    }

    if (bAlreadyInMap)
    {
        return mvpCameras[index_cam];
    }
    else
    {
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

std::vector<GeometricCamera *> Atlas::GetAllCameras()
{
    return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame *> Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint *> Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint *> Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

vector<Map *> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map *elem1, Map *elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<Map *> vMaps(mspMaps.begin(), mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap = static_cast<Map *>(NULL);
    mnLastInitKFidMap = 0;
}

Map *Atlas::GetCurrentMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    if (!mpCurrentMap)
        CreateNewMap();
    while (mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

void Atlas::SetMapBad(Map *pMap)
{
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps()
{
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

bool Atlas::isInertial()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexAtlas);
    // 接着又调用Map类的SetInertialSensor成员函数,将其设置为imu属性,以后的跟踪和预积分将和这个标志有关
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

/**
 * @brief 预保存，意思是在保存成地图文件之前，要保存到对应变量里面
 */
void Atlas::PreSave()
{
    // 1. 更新mnLastInitKFidMap
    if (mpCurrentMap)
    {
        if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1; // The init KF is the next of current maximum
    }

    // 比较map id
    struct compFunctor
    {
        inline bool operator()(Map *elem1, Map *elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
    // 2. 按照id从小到大排列
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set<GeometricCamera *> spCams(mvpCameras.begin(), mvpCameras.end());

    // 3. 遍历所有地图，执行每个地图的预保存
    for (Map *pMi : mvpBackupMaps)
    {
        if (!pMi || pMi->IsBad())
            continue;

        // 如果地图为空，则跳过
        if (pMi->GetAllKeyFrames().size() == 0)
        {
            // Empty map, erase before of save it.
            SetMapBad(pMi);
            continue;
        }
        pMi->PreSave(spCams);
    }

    // 4. 删除坏地图
    RemoveBadMaps();
}

/**
 * @brief 后加载，意思是读取地图文件后加载各个信息
 */
void Atlas::PostLoad()
{
    // 1. 读取当前所有相机
    map<unsigned int, GeometricCamera *> mpCams;
    for (GeometricCamera *pCam : mvpCameras)
    {
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    // 2. 加载各个地图
    for (Map *pMi : mvpBackupMaps)
    {
        mspMaps.insert(pMi);
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }
    mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase *pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase *Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary *pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

/**
 * @brief 以下函数暂未用到
 */
ORBVocabulary *Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map *pMap_i : mspMaps)
    {
        num += pMap_i->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map *pMap_i : mspMaps)
    {
        num += pMap_i->GetAllMapPoints().size();
    }

    return num;
}

map<long unsigned int, KeyFrame *> Atlas::GetAtlasKeyframes()
{
    map<long unsigned int, KeyFrame *> mpIdKFs;
    for (Map *pMap_i : mvpBackupMaps)
    {
        vector<KeyFrame *> vpKFs_Mi = pMap_i->GetAllKeyFrames();

        for (KeyFrame *pKF_j_Mi : vpKFs_Mi)
        {
            mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
        }
    }

    return mpIdKFs;
}

} // namespace ORB_SLAM3
