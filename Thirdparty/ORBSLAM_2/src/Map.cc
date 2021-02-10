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


} // namespace ORB_SLAM2
