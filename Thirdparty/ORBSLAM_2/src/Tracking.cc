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

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <GroundTruthFrame.h>

#include "Converter.h"
#include "FrameDrawer.h"
#include "Initializer.h"
#include "Map.h"
#include "ORBmatcher.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>
#include <mutex>
#include <unistd.h>
#include <set_MAC.h>

#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "Verbose.h"

using namespace std;

namespace ORB_SLAM2
{
    using defSLAM::Pinhole;
    using defSLAM::KannalaBrandt8;

  Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
                     MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB,
                     const string &strSettingPath, const int sensor,
                     bool viewerOn)
      : 
        mTrackedFr(0), mbStep(false), mbMapUpdated(false), time_recently_lost(5.0),
        mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr),
        // OS2
        mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false),
        mbVO(false), mpLoopClosing(NULL), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
        mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys),
        mpViewer(NULL), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer),
        mpMap(pMap), mnLastRelocFrameId(0),
        viewerOn(viewerOn)
  {
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if(!b_parse_cam)
        std::cout << "*Error with the camera parameters in the config file*" << std::endl;

    // Load ORB parameters
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if(!b_parse_orb)
        std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
        
    initID = 0; lastID = 0;

    // Load IMU parameters
    bool b_parse_imu = true;
    if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO)
    {
        b_parse_imu = ParseIMUParamFile(fSettings);
        if(!b_parse_imu)
            std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
        mnFramesToResetIMU = mMaxFrames;
    }
    mbInitWith3KFs = false;
    mnNumDataset = 0;

    if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
    {
        std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
        try
        {
            throw -1;
        }
        catch(exception &e)
        {

        }
    }

  }

  void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
  {
    mpLocalMapper = pLocalMapper;
  }

  void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
  {
    mpLoopClosing = pLoopClosing;
  }

  void Tracking::SetViewer(Viewer *pViewer) { mpViewer = pViewer; }

  cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft,
                                    const cv::Mat &imRectRight,
                                    const double &timestamp)
  {
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if (mImGray.channels() == 3)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
      }
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
      }
    }

    mCurrentFrame = new Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft,
                              mpORBextractorRight, mpORBVocabulary, mK, mDistCoef,
                              mbf, mThDepth, imRectLeft);

    Track();

    return mCurrentFrame->mTcw.clone();
  }

  cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD,
                                  const double &timestamp)
  {
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if (mImGray.channels() == 3)
    {
      if (mbRGB)
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
      else
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
      else
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
      imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

    mCurrentFrame = new Frame(mImGray, imDepth, timestamp, mpORBextractorLeft,
                              mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

    Track();

    return mCurrentFrame->mTcw.clone();
  }

  cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im,
                                       const double &timestamp)
  {

    mImGray = im.clone();
    im.copyTo(mImRGB);
    saveResults = false;
    if (mImGray.channels() == 3)
    {
      if (mbRGB)
        cv::cvtColor(im, mImGray, cv::COLOR_RGB2GRAY);
      else
        cv::cvtColor(im, mImGray, cv::COLOR_BGR2GRAY);
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
        cv::cvtColor(im, mImGray, cv::COLOR_RGBA2GRAY);
      else
        cv::cvtColor(im, mImGray, cv::COLOR_BGRA2GRAY);
    }

    // Create frame
    if(mSensor == System::IMU_MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            mCurrentFrame = new Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            mCurrentFrame = new Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }
    else if (mSensor == System::MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
            mCurrentFrame = new Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
            mCurrentFrame = new Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    
    // Set initial time
    if (mState==NO_IMAGES_YET)
        t0=timestamp;

    lastID = mCurrentFrame->mnId;
    Track();

    return mCurrentFrame->mTcw.clone();
  }

  cv::Mat Tracking::GrabImageMonocular(const cv::Mat &imRectLeft,
                                       const cv::Mat &imRectRight,
                                       const double &timestamp)
  {
    mImGray = imRectLeft.clone();
    cv::Mat imGrayRight = imRectRight;
    imRectLeft.copyTo(mImRGB);

    if (mImGray.channels() == 3)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
      }
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
      }
    }
    else
    {
      cv::cvtColor(imRectLeft, mImRGB, cv::COLOR_GRAY2RGB);
    }

    mCurrentFrame = new defSLAM::GroundTruthFrame(
        mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight,
        mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, imRectLeft);

    Track();

    return mCurrentFrame->mTcw.clone();
  }

  cv::Mat Tracking::GrabImageMonocularGT(const cv::Mat &imRectLeft,
                                         const cv::Mat &imRectRight,
                                         const double &timestamp, cv::Mat _mask)
  {
    mImGray = imRectLeft.clone();
    cv::Mat imGrayRight = imRectRight;
    imRectLeft.copyTo(mImRGB);

    if (mImGray.channels() == 3)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
      }
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
      }
    }
    else
    {
      cv::cvtColor(imRectLeft, mImRGB, cv::COLOR_GRAY2RGB);
    }

    mCurrentFrame = new defSLAM::GroundTruthFrame(mImGray, imGrayRight, timestamp, mpORBextractorLeft,
                                                  mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, imRectLeft, _mask);

    Track();

    static std::vector<float> a;

    if ((mState == eTrackingState::OK) && (saveResults))
    {
      float scale =
          static_cast<defSLAM::GroundTruthFrame *>(mCurrentFrame)
              ->Estimate3DScale(
                  mpMap);
      scalefile << mCurrentFrame->mTimeStamp << " " << scale << std::endl;
      static_cast<defSLAM::GroundTruthFrame *>(mCurrentFrame)
          ->Estimate3DError(mpMap, scale);
      mpMapDrawer->UpdatePoints(mCurrentFrame, scale);
    }
    return mCurrentFrame->mTcw.clone();
  }

  cv::Mat Tracking::GrabImageMonocularCTGT(const cv::Mat &imRectLeft,
                                           const cv::Mat &imDepth,
                                           const double &timestamp,
                                           cv::Mat _mask)
  {
    mImGray = imRectLeft.clone();
    imRectLeft.copyTo(mImRGB);

    if (mImGray.channels() == 3)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
      }
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
      }
    }
    else
    {
      cv::cvtColor(imRectLeft, mImRGB, cv::COLOR_GRAY2RGB);
    }

    mCurrentFrame = new defSLAM::GroundTruthFrame(
        mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef,
        mbf, mThDepth, imRectLeft, imDepth, true, _mask);

    this->Track();

    if ((mState == eTrackingState::OK) && (true))
    {
      float scale =
          static_cast<defSLAM::GroundTruthFrame *>(mCurrentFrame)->Estimate3DScale(mpMap);
      scalefile << mCurrentFrame->mTimeStamp << " " << scale << std::endl;
      double error = static_cast<defSLAM::GroundTruthFrame *>(mCurrentFrame)
                         ->Estimate3DError(mpMap, scale);

      if (viewerOn)
      {
        mpMapDrawer->UpdatePoints(mCurrentFrame, scale);
        this->mpFrameDrawer->SetError(error);
      }
    }

    return mCurrentFrame->mTcw.clone();
  }

  bool Tracking::RelocateImageMonocular(const cv::Mat &im,
                                        const double &timestamp)
  {
    mImGray = im;

    if (mImGray.channels() == 3)
    {
      if (mbRGB)
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
      else
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
      else
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }
    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
      mCurrentFrame =
          new Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK,
                    mDistCoef, mbf, mThDepth, im);
    else
      mCurrentFrame =
          new Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK,
                    mDistCoef, mbf, mThDepth, im);

    bool bOK = Relocalization();
    mpFrameDrawer->Update(this);
    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);

    return bOK;
  }

  void Tracking::Track()
  {
    if (mState == NO_IMAGES_YET)
    {
      mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if (mState == NOT_INITIALIZED)
    {
      if (mCurrentFrame->StereoAvailable)
        StereoInitialization();
      else
        MonocularInitialization();

      mpFrameDrawer->Update(this);

      if (mState != OK)
        return;
    }
    else
    {
      // System is initialized. Track Frame.
      bool bOK;

      // Initial camera pose estimation using motion model or relocalization (if
      // tracking is lost)
      if (!mbOnlyTracking)
      {
        bOK = this->LocalisationAndMapping();
      }
      else
      {
        bOK = this->OnlyLocalisation();
      }

      mCurrentFrame->mpReferenceKF = mpReferenceKF;

      // If we have an initial estimation of the camera pose and matching. Track
      // the local map.
      if (!mbOnlyTracking)
      {
        if (bOK)
        {
          bOK = TrackLocalMap();
        }
      }
      else
      {
        // mbVO true means that there are few matches to MapPoints in the map. We
        // cannot retrieve
        // a local map and therefore we do not perform TrackLocalMap(). Once the
        // system relocalizes
        // the camera we will use the local map again.
        if (bOK && !mbVO)
          bOK = TrackLocalMap();
      }

      if (bOK)
      {
        mState = OK;
        this->status << mCurrentFrame->mTimeStamp << " " << 0 << std::endl;
      }
      else
      {
        mState = LOST;
        this->status << mCurrentFrame->mTimeStamp << " " << 1 << std::endl;
      }

      // Update drawer
      mpFrameDrawer->Update(this);

      // If tracking were good, check if we insert a keyframe
      if (bOK)
      {
        // Update motion model
        if (!mLastFrame.mTcw.empty())
        {
          cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
          mLastFrame.GetRotationInverse().copyTo(
              LastTwc.rowRange(0, 3).colRange(0, 3));
          mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
          mVelocity = mCurrentFrame->mTcw * LastTwc;
        }
        else
          mVelocity = cv::Mat();

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);

        // Clean VO matches
        for (int i = 0; i < mCurrentFrame->N; i++)
        {
          MapPoint *pMP = mCurrentFrame->mvpMapPoints[i];
          if (pMP)
            if (pMP->Observations() < 1)
            {
              mCurrentFrame->mvbOutlier[i] = false;
              mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }

        // Delete temporal MapPoints
        for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(),
                                        lend = mlpTemporalPoints.end();
             lit != lend; lit++)
        {
          MapPoint *pMP = *lit;
          delete pMP;
        }
        mlpTemporalPoints.clear();

        // Check if we need to insert a new keyframe
        if ((mCurrentFrame->mnId % 4) < 1)
          CreateNewKeyFrame();

        // We allow points with high innovation (considererd outliers by the Huber
        // Function)
        // pass to the new keyframe, so that bundle adjustment will finally decide
        // if they are outliers or not. We don't want next frame to estimate its
        // position
        // with those points so we discard them in the frame.
        for (int i = 0; i < mCurrentFrame->N; i++)
        {
          if (mCurrentFrame->mvpMapPoints[i] && mCurrentFrame->mvbOutlier[i])
          {
            mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
          }
        }
        //    MapPointFile << std::endl;
      }

      // Reset if the camera get lost soon after initialization
      if (mState == LOST)
      {
        if (mpMap->KeyFramesInMap() <= 5)
        {
          cout << "Track lost soon after initialisation, reseting..." << endl;
          mpSystem->Reset();
          return;
        }
      }

      if (!mCurrentFrame->mpReferenceKF)
        mCurrentFrame->mpReferenceKF = mpReferenceKF;

      mLastFrame = Frame(*mCurrentFrame);
    }
    // Store frame pose information to retrieve the complete camera trajectory
    // afterwards.
    if (!mCurrentFrame->mTcw.empty())
    {
      cv::Mat Tcr =
          mCurrentFrame->mTcw * mCurrentFrame->mpReferenceKF->GetPoseInverse();
      mlRelativeFramePoses.push_back(Tcr);
      mlpReferences.push_back(mpReferenceKF);
      mlFrameTimes.push_back(mCurrentFrame->mTimeStamp);
      mlbLost.push_back(mState == LOST);
    }
    else
    {
      // This can happen if tracking is lost
      mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
      mlpReferences.push_back(mlpReferences.back());
      mlFrameTimes.push_back(mlFrameTimes.back());
      mlbLost.push_back(mState == LOST);
    }

    if (mState == OK)
      this->UpdatekeyPointsanddist();
  }

  bool Tracking::OnlyLocalisation()
  {
    // Localization Mode: Local Mapping is deactivated
    bool bOK;
    if (mState == LOST)
    {
      bOK = Relocalization();
    }
    else
    {
      if (!mbVO)
      {
        // In last frame we tracked enough MapPoints in the map
        if (!mVelocity.empty())
        {
          bOK = TrackWithMotionModel();
        }
        else
        {
          bOK = TrackReferenceKeyFrame();
        }
      }
      else
      {
        // In last frame we tracked mainly "visual odometry" points.

        // We compute two camera poses, one from motion model and one doing
        // relocalization.
        // If relocalization is sucessfull we choose that solution, otherwise we
        // retain
        // the "visual odometry" solution.

        bool bOKMM = false;
        bool bOKReloc = false;
        vector<MapPoint *> vpMPsMM;
        vector<bool> vbOutMM;
        cv::Mat TcwMM;
        if (!mVelocity.empty())
        {
          bOKMM = TrackWithMotionModel();
          vpMPsMM = mCurrentFrame->mvpMapPoints;
          vbOutMM = mCurrentFrame->mvbOutlier;
          TcwMM = mCurrentFrame->mTcw.clone();
        }
        bOKReloc = Relocalization();

        if (bOKMM && !bOKReloc)
        {
          mCurrentFrame->SetPose(TcwMM);
          mCurrentFrame->mvpMapPoints = vpMPsMM;
          mCurrentFrame->mvbOutlier = vbOutMM;

          if (mbVO)
          {
            for (int i = 0; i < mCurrentFrame->N; i++)
            {
              if (mCurrentFrame->mvpMapPoints[i] &&
                  !mCurrentFrame->mvbOutlier[i])
              {
                mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
              }
            }
          }
        }
        else if (bOKReloc)
        {
          mbVO = false;
        }

        bOK = bOKReloc || bOKMM;
      }
    }
    return bOK;
  }

  bool Tracking::LocalisationAndMapping()
  {
    // Local Mapping is activated. This is the normal behaviour, unless
    // you explicitly activate the "only tracking" mode.
    bool bOK;
    if (mState == OK)
    {
      if (mVelocity.empty() || mCurrentFrame->mnId < mnLastRelocFrameId + 2)
      {
        bOK = TrackReferenceKeyFrame();
      }
      else
      {
        bOK = TrackWithMotionModel();
        if (!bOK)
          bOK = TrackReferenceKeyFrame();
      }
    }
    else
    {
      bOK = Relocalization();
    }

    return bOK;
  }

  void Tracking::StereoInitialization()
  {
    if (mCurrentFrame->N > 100)
    {
      // Set Frame pose to the origin
      mCurrentFrame->SetPose(cv::Mat::eye(4, 4, CV_32F));

      // Create KeyFrame
      KeyFrame *pKFini = new KeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);

      // Insert KeyFrame in the map
      mpMap->AddKeyFrame(pKFini);

      // Create MapPoints and asscoiate to KeyFrame
      for (int i = 0; i < mCurrentFrame->N; i++)
      {
        float z = mCurrentFrame->mvDepth[i];
        if (z > 0)
        {
          cv::Mat x3D = mCurrentFrame->UnprojectStereo(i);
          MapPoint *pNewMP = new defSLAM::DefMapPoint(x3D, pKFini, mpMap);
          pNewMP->AddObservation(pKFini, i);
          pKFini->addMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpMap->addMapPoint(pNewMP);

          mCurrentFrame->mvpMapPoints[i] = pNewMP;
        }
      }

      cout << "New map created with " << mpMap->MapPointsInMap() << " points"
           << endl;

      mpLocalMapper->InsertKeyFrame(pKFini);

      mLastFrame = Frame(*mCurrentFrame);
      mnLastKeyFrameId = mCurrentFrame->mnId;
      mpLastKeyFrame = pKFini;

      mvpLocalKeyFrames.push_back(pKFini);
      mvpLocalMapPoints = mpMap->GetAllMapPoints();
      mpReferenceKF = pKFini;
      mCurrentFrame->mpReferenceKF = pKFini;

      mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

      mpMap->mvpKeyFrameOrigins.push_back(pKFini);

      mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);

      mState = OK;
    }
  }

  void Tracking::MonocularInitialization()
  {

    if (!mpInitializer)
    {
      // Set Reference Frame
      if (mCurrentFrame->mvKeys.size() > 100)
      {
        mInitialFrame = Frame(*mCurrentFrame);
        mInitialFrame.mpORBextractorLeft = mCurrentFrame->mpORBextractorLeft;
        mInitialFrame.mpORBvocabulary = mCurrentFrame->mpORBvocabulary;

        mLastFrame = Frame(*mCurrentFrame);
        mvbPrevMatched.resize(mCurrentFrame->mvKeysUn.size());
        for (size_t i = 0; i < mCurrentFrame->mvKeysUn.size(); i++)
          mvbPrevMatched[i] = mCurrentFrame->mvKeysUn[i].pt;

        if (mpInitializer)
          delete mpInitializer;

        mpInitializer = new Initializer(*mCurrentFrame, 0.5, 250);

        fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

        return;
      }
    }
    else
    {
      // Try to initialize
      if ((int)mCurrentFrame->mvKeys.size() <= 30)
      {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer *>(NULL);
        fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
        return;
      }

      // Find correspondences
      ORBmatcher matcher(0.9, true);
      int nmatches = matcher.SearchForInitialization(
          mInitialFrame, *mCurrentFrame, mvbPrevMatched, mvIniMatches, 150);

      // Check if there are enough correspondences
      if (nmatches < 20)
      {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer *>(NULL);
        return;
      }

      cv::Mat Rcw;                 // Current Camera Rotation
      cv::Mat tcw;                 // Current Camera Translation
      vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

      if (mpInitializer->Initialize(*mCurrentFrame, mvIniMatches, Rcw, tcw,
                                    mvIniP3D, vbTriangulated))
      {
        for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
        {
          if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
          {
            mvIniMatches[i] = -1;
            nmatches--;
          }
        }

        // Set Frame Poses
        mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
        cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
        Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
        tcw.copyTo(Tcw.rowRange(0, 3).col(3));
        mCurrentFrame->SetPose(Tcw);

        CreateInitialMapMonocular();
      }
    }
  }

  void Tracking::CreateInitialMapMonocular()
  {
    // Create KeyFrames
    KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
    KeyFrame *pKFcur = new KeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for (size_t i = 0; i < mvIniMatches.size(); i++)
    {
      if (mvIniMatches[i] < 0)
        continue;

      // Create MapPoint.
      cv::Mat worldPos(mvIniP3D[i]);

      MapPoint *pMP = new defSLAM::DefMapPoint(worldPos, pKFcur, mpMap);

      pKFini->addMapPoint(pMP, i);
      pKFcur->addMapPoint(pMP, mvIniMatches[i]);

      pMP->AddObservation(pKFini, i);
      pMP->AddObservation(pKFcur, mvIniMatches[i]);

      pMP->ComputeDistinctiveDescriptors();
      pMP->UpdateNormalAndDepth();

      // Fill Current Frame structure
      mCurrentFrame->mvpMapPoints[mvIniMatches[i]] = pMP;
      mCurrentFrame->mvbOutlier[mvIniMatches[i]] = false;

      // Add to Map
      mpMap->addMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points"
         << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 30)
    {
      cout << "Wrong initialization, reseting..." << endl;
      Reset();
      return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
      if (vpAllMapPoints[iMP])
      {
        MapPoint *pMP = vpAllMapPoints[iMP];
        pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
      }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame->SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame->mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame->mpReferenceKF = pKFcur;

    mLastFrame = Frame(*mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;
  }

  void Tracking::CheckReplacedInLastFrame()
  {
    for (int i = 0; i < mLastFrame.N; i++)
    {
      MapPoint *pMP = mLastFrame.mvpMapPoints[i];

      if (pMP)
      {
        MapPoint *pRep = pMP->GetReplaced();
        if (pRep)
        {
          mLastFrame.mvpMapPoints[i] = pRep;
        }
      }
    }
  }

  bool Tracking::TrackReferenceKeyFrame()
  {
    // Compute Bag of Words vector
    mCurrentFrame->ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<MapPoint *> vpMapPointMatches;

    int nmatches =
        matcher.SearchByBoW(mpReferenceKF, *mCurrentFrame, vpMapPointMatches);

    if (nmatches < 15)
      return false;

    mCurrentFrame->mvpMapPoints = vpMapPointMatches;
    mCurrentFrame->SetPose(mLastFrame.mTcw);

    Optimizer::poseOptimization(mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (mCurrentFrame->mvbOutlier[i])
        {
          MapPoint *pMP = mCurrentFrame->mvpMapPoints[i];

          mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
          mCurrentFrame->mvbOutlier[i] = false;
          pMP->mbTrackInView = false;
          pMP->mnLastFrameSeen = mCurrentFrame->mnId;
          nmatches--;
        }
        else if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
          nmatchesMap++;
      }
    }

    return nmatchesMap >= 30;
  }

  void Tracking::UpdateLastFrame()
  {
    // Update pose according to reference keyframe
    KeyFrame *pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR ||
        !mbOnlyTracking)
      return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float, int>> vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for (int i = 0; i < mLastFrame.N; i++)
    {
      float z = mLastFrame.mvDepth[i];
      if (z > 0)
      {
        vDepthIdx.push_back(make_pair(z, i));
      }
    }

    if (vDepthIdx.empty())
      return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++)
    {
      int i = vDepthIdx[j].second;

      bool bCreateNew = false;

      MapPoint *pMP = mLastFrame.mvpMapPoints[i];
      if (!pMP)
        bCreateNew = true;
      else if (pMP->Observations() < 1)
      {
        bCreateNew = true;
      }

      if (bCreateNew)
      {
        cv::Mat x3D = mLastFrame.UnprojectStereo(i);
        MapPoint *pNewMP = new defSLAM::DefMapPoint(x3D, mpMap, &mLastFrame, i);

        mLastFrame.mvpMapPoints[i] = pNewMP;

        mlpTemporalPoints.push_back(pNewMP);
        nPoints++;
      }
      else
      {
        nPoints++;
      }

      if (vDepthIdx[j].first > mThDepth && nPoints > 100)
        break;
    }
  }

  bool Tracking::TrackWithMotionModel()
  {
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame->SetPose(mVelocity * mLastFrame.mTcw);

    fill(mCurrentFrame->mvpMapPoints.begin(), mCurrentFrame->mvpMapPoints.end(),
         static_cast<MapPoint *>(NULL));

    // Project points seen in previous frame
    int th;
    if (mSensor != System::STEREO)
      th = 15;
    else
      th = 7;
    int nmatches = matcher.SearchByProjection(*mCurrentFrame, mLastFrame, th,
                                              mSensor == System::MONOCULAR);

    // If few matches, uses a wider window search
    if (nmatches < 60)
    {
      fill(mCurrentFrame->mvpMapPoints.begin(), mCurrentFrame->mvpMapPoints.end(),
           static_cast<MapPoint *>(nullptr));
      nmatches = matcher.SearchByProjection(*mCurrentFrame, mLastFrame, 2 * th,
                                            mSensor == System::MONOCULAR);
    }

    if (nmatches < 60)
      return false;

    // Optimize frame pose with all matches
    Optimizer::poseOptimization(mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (mCurrentFrame->mvbOutlier[i])
        {
          MapPoint *pMP = mCurrentFrame->mvpMapPoints[i];

          mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
          mCurrentFrame->mvbOutlier[i] = false;
          pMP->mbTrackInView = false;
          pMP->mnLastFrameSeen = mCurrentFrame->mnId;
          nmatches--;
        }
        else if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
          nmatchesMap++;
      }
    }

    if (mbOnlyTracking)
    {
      mbVO = nmatchesMap < 10;
      return nmatches > 20;
    }

    return nmatchesMap >= 10;
  }

  bool Tracking::TrackLocalMap()
  {
    // We have an estimation of the camera pose and some map points tracked in the
    // frame.
    // We retrieve the local map and try to find matches to points in the local
    // map.

    UpdateLocalMap();
    SearchLocalPoints();

    // Optimize Pose
    Optimizer::poseOptimization(mCurrentFrame);
    mnMatchesInliers = 0;
    int mnMatchesOutliers(0);

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (!mCurrentFrame->mvbOutlier[i])
        {
          mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
          if (!mbOnlyTracking)
          {
            if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
            {
              mnMatchesInliers++;
            }
          }
          else
            mnMatchesInliers++;
        }
        else
        {
          mnMatchesOutliers++;
        }
      }
    }
    auto points = mpMap->GetReferenceMapPoints();
    auto numberLocalMapPoints(0);
    for (auto pMP : points)
    {
      if (pMP)
      {
        if (pMP->isBad())
          continue;
        if (mCurrentFrame->isInFrustum(pMP, 0.0))
        {
          numberLocalMapPoints++;
        }
      }
    }
    // Optimize Pose
    int observedFrame(0);
    int mI(0);
    int mO(0);
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (mCurrentFrame->mvpMapPoints[i]->isBad())
          continue;
        observedFrame++;
        if (!mCurrentFrame->mvbOutlier[i])
        {
          mI++;
        }
        else
        {
          mO++;
        }
      }
    }

    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(5)
        << uint(mCurrentFrame->mTimeStamp);
    std::cout << out.str() << " " << mI << " " << mO << " "
              << numberLocalMapPoints << std::endl;
    this->matches << out.str() << " " << mI << " " << mO << " "
                  << numberLocalMapPoints << std::endl;
    // Clear points tracked
    this->mpMap->cleanTracked();
    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames &&
        mnMatchesInliers < 50)
      return false;

    if (mnMatchesInliers < 30)
      return false;
    else
      return true;
  }

  bool Tracking::NeedNewKeyFrame()
  {
    if (mbOnlyTracking)
      return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
      return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last
    // relocalisation
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames &&
        nKFs > mMaxFrames)
      return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
      nMinObs = 2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be
    // potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;
    if (mSensor != System::MONOCULAR)
    {
      for (int i = 0; i < mCurrentFrame->N; i++)
      {
        if (mCurrentFrame->mvDepth[i] > 0 &&
            mCurrentFrame->mvDepth[i] < mThDepth)
        {
          if (mCurrentFrame->mvpMapPoints[i] && !mCurrentFrame->mvbOutlier[i])
            nTrackedClose++;
          else
            nNonTrackedClose++;
        }
      }
    }

    bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
      thRefRatio = 0.4f;

    if (mSensor == System::MONOCULAR)
      thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe
    // insertion
    const bool c1a = mCurrentFrame->mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame->mnId >= mnLastKeyFrameId + mMinFrames &&
                      bLocalMappingIdle);
    // Condition 1c: tracking is weak
    const bool c1c =
        mSensor != System::MONOCULAR &&
        (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of
    // visual odometry compared to map matches.
    const bool c2 =
        ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) &&
         mnMatchesInliers > 15);

    if ((c1a || c1b || c1c) && c2)
    {
      // If the mapping accepts keyframes, insert keyframe.
      // Otherwise send a signal to interrupt BA
      if (bLocalMappingIdle)
      {
        return true;
      }
      else
      {
        mpLocalMapper->InterruptBA();
        if (mSensor != System::MONOCULAR)
        {
          if (mpLocalMapper->KeyframesInQueue() < 3)
            return true;
          else
            return false;
        }
        else
          return false;
      }
    }
    else
      return false;
  }

  void Tracking::CreateNewKeyFrame()
  {
    if (!mpLocalMapper->SetNotStop(true))
      return;

    KeyFrame *pKF = new KeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame->mpReferenceKF = pKF;
    mpLocalMapper->InsertKeyFrame(pKF);
    mpLocalMapper->SetNotStop(false);
    mnLastKeyFrameId = mCurrentFrame->mnId;
    mpLastKeyFrame = pKF;
  }

  void Tracking::SearchLocalPoints()
  {
    // Do not search map points already matched
    for (vector<MapPoint *>::iterator vit = mCurrentFrame->mvpMapPoints.begin(),
                                      vend = mCurrentFrame->mvpMapPoints.end();
         vit != vend; vit++)
    {
      MapPoint *pMP = *vit;
      if (pMP)
      {
        if (pMP->isBad())
        {
          *vit = static_cast<MapPoint *>(nullptr);
        }
        else
        {
          pMP->assigned = true;
          pMP->IncreaseVisible();
          pMP->mnLastFrameSeen = mCurrentFrame->mnId;
          pMP->mbTrackInView = false;
        }
      }
    }
    std::vector<MapPoint *> idxs;
    for (int i(0); i < mCurrentFrame->N; i++)
    {
      MapPoint *pMP = mCurrentFrame->mvpMapPoints[i];
      if (pMP)
      {
        idxs.push_back(pMP);
      }
    }
    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(),
                                      vend = mvpLocalMapPoints.end();
         vit != vend; vit++)
    {
      MapPoint *pMP = *vit;
      if (pMP)
      {
        idxs.push_back(pMP);
      }
      if (pMP->mnLastFrameSeen == mCurrentFrame->mnId)
        continue;
      if (pMP->isBad())
        continue;
      // Project (this fills MapPoint variables for matching)
      if (mCurrentFrame->isInFrustum(pMP, 0.5))
      {
        pMP->IncreaseVisible();
        nToMatch++;
      }
    }
    if (nToMatch > 0)
    {
      ORBmatcher matcher(0.8, false);
      int th = 3;

      // If the camera has been relocalised recently, perform a coarser search
      if (mCurrentFrame->mnId < mnLastRelocFrameId + 2)
        th = 5;
      matcher.SearchByProjection(*mCurrentFrame, mvpLocalMapPoints, th);
    }
  }

  void Tracking::UpdateLocalMap()
  {
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
  }

  void Tracking::UpdateLocalPoints()
  {
    mvpLocalMapPoints.clear();

    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                            itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF; itKF++)
    {
      KeyFrame *pKF = *itKF;
      const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();
      for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(),
                                              itEndMP = vpMPs.end();
           itMP != itEndMP; itMP++)
      {
        MapPoint *pMP = *itMP;
        if (!pMP)
          continue;
        if (pMP->mnTrackReferenceForFrame == mCurrentFrame->mnId)
          continue;
        if (!pMP->isBad())
        {
          mvpLocalMapPoints.push_back(pMP);
          pMP->mnTrackReferenceForFrame = mCurrentFrame->mnId;
        }
      }
    }
  }

  void Tracking::UpdateLocalKeyFrames()
  {
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame *, int> keyframeCounter;
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        MapPoint *pMP = mCurrentFrame->mvpMapPoints[i];
        if (!pMP->isBad())
        {
          const map<KeyFrame *, std::tuple<int, int>> observations = pMP->GetObservations();
          for (map<KeyFrame *, std::tuple<int, int>>::const_iterator it = observations.begin(),
                                                       itend = observations.end();
               it != itend; it++)
            keyframeCounter[it->first]++;
        }
        else
        {
          mCurrentFrame->mvpMapPoints[i] = NULL;
        }
      }
    }

    if (keyframeCounter.empty())
      return;

    int max = 0;
    KeyFrame *pKFmax = static_cast<KeyFrame *>(nullptr);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also
    // check which keyframe shares most points
    for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(),
                                              itEnd = keyframeCounter.end();
         it != itEnd; it++)
    {
      KeyFrame *pKF = it->first;

      if (pKF->isBad())
        continue;

      if (it->second > max)
      {
        max = it->second;
        pKFmax = pKF;
      }

      mvpLocalKeyFrames.push_back(it->first);
      pKF->mnTrackReferenceForFrame = mCurrentFrame->mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to
    // already-included keyframes
    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                            itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF; itKF++)
    {
      // Limit the number of keyframes
      if (mvpLocalKeyFrames.size() > 80)
        break;

      KeyFrame *pKF = *itKF;

      const vector<KeyFrame *> vNeighs =
          mpMap->GetAllKeyFrames(); // pKF->GetBestCovisibilityKeyFrames(100);

      for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(),
                                              itEndNeighKF = vNeighs.end();
           itNeighKF != itEndNeighKF; itNeighKF++)
      {
        KeyFrame *pNeighKF = *itNeighKF;
        if (!pNeighKF->isBad())
        {
          if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame->mnId)
          {
            mvpLocalKeyFrames.push_back(pNeighKF);
            pNeighKF->mnTrackReferenceForFrame = mCurrentFrame->mnId;
            break;
          }
        }
      }

      const set<KeyFrame *> spChilds = pKF->GetChilds();
      for (set<KeyFrame *>::const_iterator sit = spChilds.begin(),
                                           send = spChilds.end();
           sit != send; sit++)
      {
        KeyFrame *pChildKF = *sit;
        if (!pChildKF->isBad())
        {
          if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame->mnId)
          {
            mvpLocalKeyFrames.push_back(pChildKF);
            pChildKF->mnTrackReferenceForFrame = mCurrentFrame->mnId;
            break;
          }
        }
      }

      KeyFrame *pParent = pKF->GetParent();
      if (pParent)
      {
        if (pParent->mnTrackReferenceForFrame != mCurrentFrame->mnId)
        {
          mvpLocalKeyFrames.push_back(pParent);
          pParent->mnTrackReferenceForFrame = mCurrentFrame->mnId;
          break;
        }
      }
    }

    if (pKFmax)
    {
      mpReferenceKF = pKFmax;
      mCurrentFrame->mpReferenceKF = mpReferenceKF;
    }
  }

  bool Tracking::Relocalization()
  {
    // Compute Bag of Words Vector
    mCurrentFrame->ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for
    // relocalisation
    vector<KeyFrame *> vpCandidateKFs =
        mpKeyFrameDB->DetectRelocalizationCandidates(mCurrentFrame);

    if (vpCandidateKFs.empty())
      return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<PnPsolver *> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint *>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++)
    {
      KeyFrame *pKF = vpCandidateKFs[i];
      if (pKF->isBad())
        vbDiscarded[i] = true;
      else
      {
        int nmatches =
            matcher.SearchByBoW(pKF, *mCurrentFrame, vvpMapPointMatches[i]);
        if (nmatches < 15)
        {
          vbDiscarded[i] = true;
          continue;
        }
        else
        {
          PnPsolver *pSolver =
              new PnPsolver(*mCurrentFrame, vvpMapPointMatches[i]);
          pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
          vpPnPsolvers[i] = pSolver;
          nCandidates++;
        }
      }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch)
    {
      for (int i = 0; i < nKFs; i++)
      {
        if (vbDiscarded[i])
          continue;

        // Perform 5 Ransac Iterations
        vector<bool> vbInliers;
        int nInliers;
        bool bNoMore;

        PnPsolver *pSolver = vpPnPsolvers[i];
        cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

        // If Ransac reachs max. iterations discard keyframe
        if (bNoMore)
        {
          vbDiscarded[i] = true;
          nCandidates--;
        }

        // If a Camera Pose is computed, optimize
        if (!Tcw.empty())
        {
          Tcw.copyTo(mCurrentFrame->mTcw);

          set<MapPoint *> sFound;

          const int np = vbInliers.size();

          for (int j = 0; j < np; j++)
          {
            if (vbInliers[j])
            {
              mCurrentFrame->mvpMapPoints[j] = vvpMapPointMatches[i][j];
              sFound.insert(vvpMapPointMatches[i][j]);
            }
            else
              mCurrentFrame->mvpMapPoints[j] = NULL;
          }

          int nGood = Optimizer::poseOptimization(mCurrentFrame);

          if (nGood < 10)
            continue;

          for (int io = 0; io < mCurrentFrame->N; io++)
            if (mCurrentFrame->mvbOutlier[io])
              mCurrentFrame->mvpMapPoints[io] = static_cast<MapPoint *>(nullptr);

          // If few inliers, search by projection in a coarse window and optimize
          // again
          if (nGood < 50)
          {
            int nadditional = matcher2.SearchByProjection(
                *mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

            if (nadditional + nGood >= 50)
            {
              nGood = Optimizer::poseOptimization(mCurrentFrame);

              // If many inliers but still not enough, search by projection again
              // in a narrower window
              // the camera has been already optimized with many points
              if (nGood > 30 && nGood < 50)
              {
                sFound.clear();
                for (int ip = 0; ip < mCurrentFrame->N; ip++)
                  if (mCurrentFrame->mvpMapPoints[ip])
                    sFound.insert(mCurrentFrame->mvpMapPoints[ip]);
                nadditional = matcher2.SearchByProjection(
                    *mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                // Final optimization
                if (nGood + nadditional >= 50)
                {
                  nGood = Optimizer::poseOptimization(mCurrentFrame);

                  for (int io = 0; io < mCurrentFrame->N; io++)
                    if (mCurrentFrame->mvbOutlier[io])
                      mCurrentFrame->mvpMapPoints[io] = NULL;
                }
              }
            }
          }

          // If the pose is supported by enough inliers stop ransacs and continue
          if (nGood >= 50)
          {
            bMatch = true;
            break;
          }
        }
      }
    }

    if (!bMatch)
    {
      return false;
    }
    else
    {
      mnLastRelocFrameId = mCurrentFrame->mnId;
      return true;
    }
  }

  void Tracking::Reset()
  {

    cout << "System Reseting" << endl;
    if (mpViewer)
    {
      mpViewer->RequestStop();
      while (!mpViewer->isStopped())
        usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    if (mpLoopClosing)
      mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if (mpInitializer)
    {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer *>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if (mpViewer)
      mpViewer->Release();
  }

  void Tracking::ChangeCalibration(const string &strSettingPath)
  {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0)
    {
      DistCoef.resize(5);
      DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
  }

  void Tracking::InformOnlyTracking(const bool &flag) { mbOnlyTracking = flag; }

  void Tracking::UpdatekeyPointsanddist()
  {
    std::vector<std::pair<std::pair<uint, uint>, double>> keyanddist;
    uint N = this->mCurrentFrame->mvKeys.size();

    for (uint i = 0; i < N; i++)
    {
      MapPoint *pMP = this->mCurrentFrame->mvpMapPoints[i];
      if (pMP)
      {
        if (!this->mCurrentFrame->mvbOutlier[i])
        {
          cv::Mat wposw = pMP->GetWorldPos();
          uint x = this->mCurrentFrame->mvKeys[i].pt.x;
          uint y = this->mCurrentFrame->mvKeys[i].pt.y;
          cv::Mat row = cv::Mat::ones(1, 1, CV_32F);
          wposw.push_back(row);
          cv::Mat wposc = this->mCurrentFrame->mTcw * wposw;
          std::pair<uint, uint> kp = std::pair<uint, uint>(x, y);
          /*    keyanddist.push_back(std::pair<std::pair<uint,uint>,double>(kp,
                                                                            sqrt(wposc.at<float>(0)* wposc.at<float>(0)+ wposc.at<float>(1)* wposc.at<float>(1)+ wposc.at<float>(2)* wposc.at<float>(2))));
            */
          keyanddist
              .push_back(std::pair<std::pair<uint, uint>, double>(
                  kp, wposc.at<float>(2)));
        }
      }
    }
  }

  double Tracking::getRegInex()
  {
    std::unique_lock<std::mutex> m(Regmutex);
    return RegInex;
  }

  double Tracking::getRegLap()
  {
    std::unique_lock<std::mutex> m(Regmutex);
    return RegLap;
  }

  double Tracking::getRegTemp()
  {
    std::unique_lock<std::mutex> m(Regmutex);
    return RegTemp;
  }

  void Tracking::setRegInex(double newreginex)
  {
    std::unique_lock<std::mutex> m(Regmutex);
    RegInex = newreginex;
  }

  void Tracking::setRegLap(double newreglap)
  {
    std::unique_lock<std::mutex> m(Regmutex);
    RegLap = newreglap;
  }

  void Tracking::setRegTemp(double newregTemp)
  {
    std::unique_lock<std::mutex> m(Regmutex);
    RegTemp = newregTemp;
  }
  
    int Tracking::GetMatchesInliers()
    {
        return mnMatchesInliers;
    }


void Tracking::UpdateFrameIMU(const float s, const defSLAM::IMU::Bias &b, KeyFrame* pCurrentKeyFrame)
{
    Map * pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        while(pKF->isBad())
        {
            pKF = pKF->GetParent();
        }

        if(pKF->GetMap() == pMap)
        {
            (*lit).rowRange(0,3).col(3)=(*lit).rowRange(0,3).col(3)*s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame->SetNewBias(mLastBias);

    cv::Mat Gz = (cv::Mat_<float>(3,1) << 0, 0, -defSLAM::IMU::GRAVITY_VALUE);

    cv::Mat twb1;
    cv::Mat Rwb1;
    cv::Mat Vwb1;
    float t12;

    while(!mCurrentFrame->imuIsPreintegrated())
    {
        usleep(500);
    }


    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame->mpImuPreintegrated)
    {
        twb1 = mCurrentFrame->mpLastKeyFrame->GetImuPosition();
        Rwb1 = mCurrentFrame->mpLastKeyFrame->GetImuRotation();
        Vwb1 = mCurrentFrame->mpLastKeyFrame->GetVelocity();
        t12 = mCurrentFrame->mpImuPreintegrated->dT;

        mCurrentFrame->SetImuPoseVelocity(Rwb1*mCurrentFrame->mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame->mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mCurrentFrame->mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame->mnId;
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];
    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        vector<float> vCamCalib{fx,fy,cx,cy};

        mpCamera = new Pinhole(vCamCalib);

        // mpAtlasmpAtlas->AddCamera(mpCamera);

        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5)
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;

    }
    else if(sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        if(!b_miss_params)
        {
            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);

            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO){
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            if(!node.empty())
            {
                mTlr = node.mat();
                if(mTlr.rows != 3 || mTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params)
            {
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << mTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params)
        {
            return false;
        }

        // mpAtlas->AddCamera(mpCamera);
        // mpAtlas->AddCamera(mpCamera2);
    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if(!node.empty() && node.isReal())
        {
            mbf = node.real();
        }
        else
        {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO)
    {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty()  && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf*mThDepth/fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }
        else
        {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }


    }

    if(mSensor==System::RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal())
        {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    if(b_miss_params)
    {
        return false;
    }

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        // mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST); // defSLAM
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::Mat Tbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty())
    {
        Tbc = node.mat();
        if(Tbc.rows != 4 || Tbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }

    cout << endl;

    cout << "Left camera to Imu Transform (Tbc): " << endl << Tbc << endl;

    float freq, Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt())
    {
        freq = node.operator int();
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(freq);
    cout << endl;
    cout << "IMU frequency: " << freq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    mpImuCalib = new defSLAM::IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new defSLAM::IMU::Preintegrated(defSLAM::IMU::Bias(),*mpImuCalib);


    return true;
}

void Tracking::GrabImuData(const defSLAM::IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

} // namespace ORB_SLAM2
