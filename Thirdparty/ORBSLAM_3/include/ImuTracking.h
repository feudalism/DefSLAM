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


#ifndef IMUTRACKING_H
#define IMUTRACKING_H

#include "Tracking.h"
#include "GeometricCamera.h"
#include "ImuFrame.h"
#include "ORBextractor.h"
#include "ImuTypes.h"

// #include<opencv2/core/core.hpp>
// #include<opencv2/features2d/features2d.hpp>
// #include <opencv2/video/tracking.hpp>

// #include"Viewer.h"
// #include"LocalMapping.h"
// #include"LoopClosing.h"
// #include "Initializer.h"
// #include "System.h"
// #include "FrameDrawer.h"
// #include "KeyFrameDatabase.h"
// #include "Map.h"
// #include "MapDrawer.h"
// #include "ORBVocabulary.h"

// #include "GeometricCamera.h"

// #include <mutex>
// #include <unordered_set>

// namespace ORB_SLAM2
// {
    // class Tracking;
    // class FrameDrawer;
    // class KeyFrameDatabase;
    // class Map;
    // class MapDrawer;
// } 

namespace ORB_SLAM3
{
    using ORB_SLAM2::Tracking;
    
    using defSLAM::System;
    
    using ORB_SLAM2::FrameDrawer;
    using ORB_SLAM2::KeyFrameDatabase;
    using ORB_SLAM2::Map;
    using ORB_SLAM2::MapDrawer;
    // using ORB_SLAM2::ORBVocabulary;
    // using ORB_SLAM2::ORBextractor;
    
// class Viewer;
// class FrameDrawer;
// class Atlas;
// class LocalMapping;
// class LoopClosing;
// class System;

class ImuTracking : public Tracking
{  

    public:
        ImuTracking(System *pSys, ORBVocabulary *pVoc,
                    FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                    Map *pMap, KeyFrameDatabase *pKFDB,
                    const string &strSettingPath,
                    const int sensor = defSLAM::System::IMU_MONOCULAR,
                    bool viewerOn = false);

        void GrabImuData(const IMU::Point &imuMeasurement);
        
        cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

        // Parse the config file
        bool ParseCamParamFile(cv::FileStorage &fSettings);
        bool ParseORBParamFile(cv::FileStorage &fSettings);
        bool ParseIMUParamFile(cv::FileStorage &fSettings);

        void Reset(bool bLocMap = false);

        // Tracking states
        enum eTrackingState{
            SYSTEM_NOT_READY=-1,
            NO_IMAGES_YET=0,
            NOT_INITIALIZED=1,
            OK=2,
            LOST=3,
            RECENTLY_LOST=4,
            OK_KLT=5
        };

        eTrackingState mState;
        eTrackingState mLastProcessedState;
        
        bool mbInitWith3KFs;

        // frames with estimated pose
        int mTrackedFr;
        bool mbStep;

        // // Preprocess the input and call Track(). Extract features and performs stereo matching.
        // cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
        // cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
        // cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);
        // // cv::Mat GrabImageImuMonocular(const cv::Mat &im, const double &timestamp);

        // void SetLocalMapper(LocalMapping* pLocalMapper);
        // void SetLoopClosing(LoopClosing* pLoopClosing);
        // void SetViewer(Viewer* pViewer);
        // void SetStepByStep(bool bSet);

        // void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
        // KeyFrame* GetLastKeyFrame()
        // {
            // return mpLastKeyFrame;
        // }

        // void CreateMapInAtlas();
        // std::mutex mMutexTracks;

        // //--
        // void NewDataset();
        // int GetNumberDataset();
        // int GetMatchesInliers();
        
        // cv::Mat mImRight;

        // // Current Frame
        // Frame mLastFrame;
        // void ResetActiveMap(bool bLocMap = false);

        // float mMeanTrack;
        // double t0; // time-stamp of first read frame
        // double t0vis; // time-stamp of first inserted keyframe
        // double t0IMU; // time-stamp of IMU initialization


        // vector<MapPoint*> GetLocalMapMPS();


        // //TEST--
        // bool mbNeedRectify;
        // //cv::Mat M1l, M2l;
        // //cv::Mat M1r, M2r;

        // bool mbWriteStats;

    protected:
        // Queue of IMU measurements between frames
        std::list<IMU::Point> mlQueueImuData;

        // // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
        // std::vector<IMU::Point> mvImuFromLastFrame;
        std::mutex mMutexImuQueue;

        int initID, lastID;
        
        GeometricCamera* mpCamera;
        
        // ORB
        ORBextractor* mpORBextractorLeft;
        ORBextractor* mpIniORBextractor;
        
        // IMU
        int mnFramesToResetIMU;

        // Calibration parameters
        IMU::Calib *mpImuCalib;

        // Preintegration from last frame
        IMU::Preintegrated *mpImuPreintegratedFromLastKF;
        
        bool mbMapUpdated;

        // // Main tracking function. It is independent of the input sensor.
        // void Track();

        // // Map initialization for stereo and RGB-D
        // void StereoInitialization();

        // // Map initialization for monocular
        // void MonocularInitialization();
        // void CreateNewMapPoints();
        // cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);
        // void CreateInitialMapMonocular();

        // void CheckReplacedInLastFrame();
        // bool TrackReferenceKeyFrame();
        // void UpdateLastFrame();
        // bool TrackWithMotionModel();
        // bool PredictStateIMU();

        // bool Relocalization();

        // void UpdateLocalMap();
        // void UpdateLocalPoints();
        // void UpdateLocalKeyFrames();

        // bool TrackLocalMap();
        // bool TrackLocalMap_old();
        // void SearchLocalPoints();

        // bool NeedNewKeyFrame();
        // void CreateNewKeyFrame();

        // // Perform preintegration from last frame
        // void PreintegrateIMU();

        // // Reset IMU biases and compute frame velocity
        // void ResetFrameIMU();
        // void ComputeGyroBias(const vector<Frame*> &vpFs, float &bwx,  float &bwy, float &bwz);
        // void ComputeVelocitiesAccBias(const vector<Frame*> &vpFs, float &bax,  float &bay, float &baz);



        // // Last Bias Estimation (at keyframe creation)
        // IMU::Bias mLastBias;

        // // Initalization (only for monocular)
        // bool mbSetInit;
        
        // //Drawers
        // bool bStepByStep;

        // int mnFirstImuFrameId;
        
        //Last Frame, KeyFrame and Relocalisation Info
        double mTimeStampLost;
        double time_recently_lost;

        unsigned int mnFirstFrameId;
        unsigned int mnInitialFrameId;
        unsigned int mnLastInitFrameId;

        bool mbCreatedMap;


        // //Motion Model
        // cv::Mat mVelocity;

        // //int nMapChangeIndex;

        // int mnNumDataset;

        // ofstream f_track_stats;

        // ofstream f_track_times;
        // double mTime_PreIntIMU;
        // double mTime_PosePred;
        // double mTime_LocalMapTrack;
        // double mTime_NewKF_Dec;

        // int initID, lastID;

        // cv::Mat mTlr;
};

} //namespace ORB_SLAM3

#endif // IMUTRACKING_H
