/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
*of Zaragoza) && Shaifali Parashar, Adrien Bartoli (Université Clermont Auvergne)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include <mutex>

namespace ORB_SLAM2
{
long unsigned int Map::nNextId=0;

Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0),
mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), 
mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid):
mnInitKFid(initKFid), mnMaxKFid(initKFid),mnLastLoopKFid(initKFid), mnBigChangeIdx(0), mIsInUse(false),
mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::~Map()
{
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if(mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte*>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if (pKF->mnId > mnMaxKFid)
        mnMaxKFid = pKF->mnId;
}

void Map::addMapPoint(MapPoint *pMP)
{
unique_lock<mutex> lock(mMutexMap);
mspMapPoints.insert(pMP);
}

void Map::eraseMapPoint(MapPoint *pMP)
{
unique_lock<mutex> lock(mMutexMap);
mspMapPoints.erase(pMP);

// TODO: This only erase the pointer.
// Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
unique_lock<mutex> lock(mMutexMap);
mspKeyFrames.erase(pKF);

// TODO: This only erase the pointer.
// Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
unique_lock<mutex> lock(mMutexMap);
mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
unique_lock<mutex> lock(mMutexMap);
mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
unique_lock<mutex> lock(mMutexMap);
return mnBigChangeIdx;
}

vector<KeyFrame *> Map::GetAllKeyFrames()
{
unique_lock<mutex> lock(mMutexMap);
return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
}

vector<MapPoint *> Map::GetAllMapPoints()
{
unique_lock<mutex> lock(mMutexMap);
return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
unique_lock<mutex> lock(mMutexMap);
return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
unique_lock<mutex> lock(mMutexMap);
return mspKeyFrames.size();
}

vector<MapPoint *> Map::GetReferenceMapPoints()
{
unique_lock<mutex> lock(mMutexMap);
return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
unique_lock<mutex> lock(mMutexMap);
return mnMaxKFid;
}

void Map::clear()
{
for (set<MapPoint *>::iterator sit = mspMapPoints.begin(),
                               send = mspMapPoints.end();
     sit != send; sit++)
  delete *sit;

for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(),
                               send = mspKeyFrames.end();
     sit != send; sit++)
  delete *sit;

mspMapPoints.clear();
mspKeyFrames.clear();
mnMaxKFid = 0;
mvpReferenceMapPoints.clear();
mvpKeyFrameOrigins.clear();
}

void Map::ApplyScale(const float s)
{
unique_lock<mutex> lock(mMutexMap);

for (std::set<MapPoint *>::iterator sit = mspMapPoints.begin();
     sit != mspMapPoints.end(); sit++)
{
  MapPoint *pMP = *sit;
  pMP->SetWorldPos(s * pMP->GetWorldPos());
  pMP->UpdateNormalAndDepth();
}
// Pose actualization:revise
for (std::set<KeyFrame *>::iterator sit = mspKeyFrames.begin();
     sit != mspKeyFrames.end(); sit++)
{
  cv::Mat Tcw_ = (*sit)->GetPose();
  cv::Mat Twc_ = (*sit)->GetPoseInverse();

  Twc_.rowRange(0, 3).col(3) *= s;
  Tcw_.rowRange(0, 3).colRange(0, 3) = Twc_.rowRange(0, 3).colRange(0, 3).t();
  Tcw_.rowRange(0, 3).col(3) =
      -Tcw_.rowRange(0, 3).colRange(0, 3) * Twc_.rowRange(0, 3).col(3);
  (*sit)->SetPose(Tcw_);
}
}

void Map::cleanTracked()
{
for (auto &pMP : mspMapPoints)
{
  pMP->assigned = false;
}
}

// OS3
void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}

long unsigned int Map::GetId()
{
    return mnId;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetInertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetInertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetInertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetInertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}



void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    int nMPWithoutObs = 0;
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame*, std::tuple<int,int>> mpObs = pMPi->GetObservations();
        for(map<KeyFrame*, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
        {
            if(it->first->GetMap() != this)
            {
                pMPi->EraseObservation(it->first); //We need to find where the KF is set as Bad but the observation is not removed
            }

        }
    }
    cout << "  Bad MapPoints removed" << endl;

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }

    mvpBackupMapPoints.clear();
    // Backup of set container to vector
    //std::copy(mspMapPoints.begin(), mspMapPoints.end(), std::back_inserter(mvpBackupMapPoints));
    for(MapPoint* pMPi : mspMapPoints)
    {
        //cout << "Pre-save of mappoint " << pMPi->mnId << endl;
        mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames,mspMapPoints);
    }
    cout << "  MapPoints back up done!!" << endl;

    mvpBackupKeyFrames.clear();
    //std::copy(mspKeyFrames.begin(), mspKeyFrames.end(), std::back_inserter(mvpBackupKeyFrames));
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
    }
    cout << "  KeyFrames back up done!!" << endl;

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<unsigned int, GeometricCamera*> &mpCams)
{
    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

    map<long unsigned int,MapPoint*> mpMapPointId;
    for(MapPoint* pMPi : mspMapPoints)
    {
        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    //map<long unsigned int, KeyFrame*> mpKeyFrameId;
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }
    cout << "Number of KF: " << mspKeyFrames.size() << endl;
    cout << "Number of MP: " << mspMapPoints.size() << endl;

    // References reconstruction between different instances
    for(MapPoint* pMPi : mspMapPoints)
    {
        //cout << "Post-Load of mappoint " << pMPi->mnId << endl;
        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }
    cout << "End to rebuild MapPoint references" << endl;

    for(KeyFrame* pKFi : mspKeyFrames)
    {
        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }

    cout << "End to rebuild KeyFrame references" << endl;

    mvpBackupMapPoints.clear();
}

long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel, const cv::Mat t)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    cv::Mat Txw = cv::Mat::eye(4,4,CV_32F);
    R.copyTo(Txw.rowRange(0,3).colRange(0,3));

    cv::Mat Tyx = cv::Mat::eye(4,4,CV_32F);

    cv::Mat Tyw = Tyx*Txw;
    Tyw.rowRange(0,3).col(3) = Tyw.rowRange(0,3).col(3)+t;
    cv::Mat Ryw = Tyw.rowRange(0,3).colRange(0,3);
    cv::Mat tyw = Tyw.rowRange(0,3).col(3);

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        cv::Mat Twc = pKF->GetPoseInverse();
        Twc.rowRange(0,3).col(3)*=s;
        cv::Mat Tyc = Tyw*Twc;
        cv::Mat Tcy = cv::Mat::eye(4,4,CV_32F);
        Tcy.rowRange(0,3).colRange(0,3) = Tyc.rowRange(0,3).colRange(0,3).t();
        Tcy.rowRange(0,3).col(3) = -Tcy.rowRange(0,3).colRange(0,3)*Tyc.rowRange(0,3).col(3);
        pKF->SetPose(Tcy);
        cv::Mat Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(s*Ryw*pMP->GetWorldPos()+tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

} // namespace ORB_SLAM2
