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

#ifndef MAP_H
#define MAP_H

#include "KeyFrame.h"
#include "MapPoint.h"

#include <mutex>
#include <set>
#include <vector>

#include "GeometricCamera.h"

namespace ORB_SLAM2
{
  class MapPoint;
  class KeyFrame;
  class Frame;
  class Map
  {
  public:
    Map();

    void AddKeyFrame(KeyFrame *pKF);
    void addMapPoint(MapPoint *pMP);
    void eraseMapPoint(MapPoint *pMP);
    void EraseKeyFrame(KeyFrame *pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();
    virtual void ApplyScale(const float s);

    std::vector<KeyFrame *> GetAllKeyFrames();
    std::vector<MapPoint *> GetAllMapPoints();
    std::vector<MapPoint *> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned KeyFramesInMap();

    long unsigned int GetMaxKFid();
    // Set to false all the assigned points
    virtual void cleanTracked();

    virtual void clear();

  public:
    std::vector<KeyFrame *> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads
    // (id conflict)
    std::mutex mMutexPointCreation;

  protected:
    std::set<MapPoint *> mspMapPoints;
    std::set<KeyFrame *> mspKeyFrames;

    std::vector<MapPoint *> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
    
  // OS3
  public:
    std::vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFrame* mpFirstRegionKF;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;
    
  protected:
    long unsigned int mnId;
    
    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;
    
    long unsigned int mnInitKFid;
    long unsigned int mnLastLoopKFid;
    
    // // View of the map in aerial sight (for the AtlasViewer)
    // GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;
    
    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;
    
  };

} // namespace ORB_SLAM2

#endif // MAP_H
