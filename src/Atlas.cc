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

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3
{

Atlas::Atlas(){
    mpCurrentMap = static_cast<Map*>(NULL);
}

Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{
    mpCurrentMap = static_cast<Map*>(NULL);
    CreateNewMap();
}

Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map* pMi = *it;

        if(pMi)
        {
            delete pMi;
            pMi = static_cast<Map*>(NULL);

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

void Atlas::CreateNewMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    std::cout << "Creation of new map with id: " << Map::nNextId << std::endl;
    if(mpCurrentMap){
        std::cout << "Exits current map " << std::endl;
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        std::cout << "Saved map with ID: " << mpCurrentMap->GetId() << std::endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    std::cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << std::endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(Map* pMap)
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    std::cout << "Chage to map with id: " << pMap->GetId() << std::endl;
    if(mpCurrentMap){
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP)
{
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

void Atlas::AddCamera(GeometricCamera* pCam)
{
    mvpCameras.push_back(pCam);
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

std::vector<Map*> Atlas::GetAllMaps()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    std::sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap = static_cast<Map*>(NULL);
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap();
    while(mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

void Atlas::SetMapBad(Map* pMap)
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
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* mMAPi : mspMaps)
    {
        num += mMAPi->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map *mMAPi : mspMaps) {
        num += mMAPi->GetAllMapPoints().size();
    }

    return num;
}

} //namespace ORB_SLAM3
