/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
* of Zaragoza)
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

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "DefMapPoint.h"
#include "Frame.h"
#include "FrameDrawer.h"
#include "Initializer.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "System.h"
#include "Viewer.h"
#include <mutex>

#include "ImuTypes.h"
#include "GeometricCamera.h"

namespace defSLAM
{
  class System;
}

namespace ORB_SLAM2
{

  class Viewer;
  class Frame;
  class FrameDrawer;
  class Map;
  class LocalMapping;
  class LoopClosing;
  using defSLAM::System;

  class Tracking
  {
  public:
    Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
             MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB,
             const string &strSettingPath, const int sensor, bool viewerOn);
             
    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);

    // Preprocess the input and call Track(). Extract features and performs stereo
    // matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight,
                            const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD,
                          const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const cv::Mat &imRight,
                               const double &timestamp);

    virtual cv::Mat GrabImageMonocularGT(const cv::Mat &im,
                                         const cv::Mat &imRight,
                                         const double &timestamp,
                                         cv::Mat _mask = cv::Mat());
    virtual cv::Mat GrabImageMonocularCTGT(const cv::Mat &im,
                                           const cv::Mat &imDepth,
                                           const double &timestamp,
                                           cv::Mat _mask = cv::Mat());
    void SetLocalMapper(LocalMapping *pLocalMapper);
    void SetLoopClosing(LoopClosing *pLoopClosing);
    void SetViewer(Viewer *pViewer);
    bool RelocateImageMonocular(const cv::Mat &im, const double &timestamp);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when
    // projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want
    // to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const defSLAM::IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    virtual void UpdatekeyPointsanddist();
    
    int GetMatchesInliers();

  public:
    // Tracking states
    enum eTrackingState
    {
      SYSTEM_NOT_READY = -1,
      NO_IMAGES_YET = 0,
      NOT_INITIALIZED = 1,
      OK = 2,
      // LOST = 3
      RECENTLY_LOST=3,
      LOST=4,
      OK_KLT=5
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame *mCurrentFrame;
    Frame mLastFrame;
    
    cv::Mat mImGray;
    cv::Mat mImRGB;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the
    // execution.
    // Basically we store the reference keyframe for each frame and its relative
    // transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame *> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // OS3
    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only
    // localization
    bool mbOnlyTracking;

    void Reset();
    void ResetActiveMap(bool bLocMap = false); // OS3
    
    double getRegInex();
    double getRegLap();
    double getRegTemp();

    void setRegInex(double);
    void setRegLap(double);
    void setRegTemp(double);

    // OS3
    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization

    vector<MapPoint*> GetLocalMapMPS();

    //TEST--
    bool mbNeedRectify;
    bool mbWriteStats;
    cv::Mat mImRight;

  protected:
    // Main tracking function. It is independent of the input sensor.
    virtual void Track();

    // Map initialization for stereo and RGB-D
    virtual void StereoInitialization();
    virtual bool LocalisationAndMapping();
    virtual bool OnlyLocalisation();
    // Map initialization for monocular
    virtual void MonocularInitialization();
    virtual void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    virtual bool TrackReferenceKeyFrame();
    virtual void UpdateLastFrame();
    virtual bool TrackWithMotionModel();

    virtual bool Relocalization();

    void UpdateLocalMap();
    virtual void UpdateLocalPoints();
    virtual void UpdateLocalKeyFrames();

    virtual bool TrackLocalMap();
    virtual void SearchLocalPoints();

    virtual bool NeedNewKeyFrame();
    virtual void CreateNewKeyFrame();

  protected:
    // OS3
    bool mbMapUpdated;

    // Imu preintegration from last frame
    defSLAM::IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<defSLAM::IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<defSLAM::IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    defSLAM::IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    defSLAM::IMU::Bias mLastBias;
    // /end OS3

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    
    // In case of performing only localization, this flag is true when there are
    // no matches to
    // points in the map. Still tracking will continue if there are enough matches
    // with temporal points.
    // In that case we are doing visual odometry. The system will try to do
    // relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;
    // Other Thread Pointers
    LocalMapping *mpLocalMapper;
    LoopClosing *mpLoopClosing;

    // ORB
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor *mpIniORBextractor;

    // BoW
    ORBVocabulary *mpORBVocabulary;
    KeyFrameDatabase *mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer *mpInitializer;
    bool mbSetInit; // OS3
    
    // Local Map
    KeyFrame *mpReferenceKF;
    std::vector<KeyFrame *> mvpLocalKeyFrames;
    std::vector<MapPoint *> mvpLocalMapPoints;

    // System
    System *mpSystem;

    // Drawers
    Viewer *mpViewer;
    FrameDrawer *mpFrameDrawer;
    MapDrawer *mpMapDrawer;
    bool bStepByStep; // OS3

    // Map
    Map *mpMap;

    // Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    // New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // OS3
    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two
    // keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are
    // scaled.
    float mDepthMapFactor;

    // Current matches in frame
    int mnMatchesInliers;

    // Last Frame, KeyFrame and Relocalisation Info
    KeyFrame *mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    
    // OS3
    double mTimeStampLost;
    double time_recently_lost;
    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;
    bool mbCreatedMap;
    int mnNumDataset;

    // Motion Model
    cv::Mat mVelocity;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint *> mlpTemporalPoints;
    ofstream status;
    double RegLap, RegTemp, RegInex, ReliabilityThreshold;
    ofstream matches, scalefile, MapPointFile;

    bool viewerOn;
    
    bool saveResults;

    std::mutex Regmutex;

    // OS3
    ofstream f_track_stats;
    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    ORB_SLAM3::GeometricCamera* mpCamera, *mpCamera2;

    int initID, lastID;

    cv::Mat mTlr;
  };

} // namespace ORB_SLAM2

#endif // TRACKING_H
