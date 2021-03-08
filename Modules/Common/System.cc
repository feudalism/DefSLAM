/**
* This file is part of DefSLAM.
*
* Copyright (C) 2017-2020 Jose Lamarca Peiro <jlamarca at unizar dot es>, J.M.M. Montiel (University
*of Zaragoza) && Shaifali Parashar, Adrien Bartoli (Université Clermont Auvergne)
*
* DefSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DefSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DefSLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "System.h"
#include "Converter.h"
#include "set_MAC.h"
#ifdef ORBSLAM
#include "FrameDrawer.h"
#include "LocalMapping.h"
#include "Map.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "Viewer.h"
#else
#include "DefFrameDrawer.h"
#include "DefLocalMapping.h"
#include "DefMap.h"
#include "DefMapDrawer.h"
#include "DefTracking.h"
#include "DefViewer.h"
#endif
#include <iomanip>
#include <pangolin/pangolin.h>
#include <thread>
#include <unistd.h>

namespace defSLAM
{
  using ORB_SLAM2::Converter;
  
  System::System(const string &strVocFile, const string &strSettingsFile,
                 const bool bUseViewer)
      : mSensor(MONOCULAR), mpLoopCloser(NULL), mpViewer(static_cast<Viewer *>(nullptr)),
        mbReset(false), mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)
  {
    // Output welcome message
#ifndef ORBSLAM
    cout << endl
         << "DefSLAM 2019-2020 José Lamarca, University of Zaragoza." << endl
         << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
         << "This is free software, and you are welcome to redistribute it"
         << endl
         << endl;
#else
    cout << endl
         << "Modification for test of ORB-SLAM2 Copyright (C) 2014-2016 " << endl
         << "Raul Mur - Artal, University of Zaragoza." << endl
         << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
         << "This is free software, and you are welcome to redistribute it" << endl
         << "under certain conditions. See LICENSE.txt." << endl
         << endl;

#endif
    cout << "Input sensor was set to: ";

    if (mSensor != MONOCULAR)
    {
      cout << "Error" << endl;
      exit(-1);
    }
    // Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
      cerr << "Failed to open settings file at: " << strSettingsFile << endl;
      exit(-1);
    }

    // Load ORB Vocabulary
    cout << endl
         << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad)
    {
      cerr << "Wrong path to vocabulary. " << endl;
      cerr << "Falied to open at: " << strVocFile << endl;
      exit(-1);
    }
    cout << "Vocabulary loaded!" << endl
         << endl;

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

// Create the Map
#ifndef ORBSLAM
    mpMap = new DefMap();
#else
    mpMap = new Map();
#endif

    if (bUseViewer)
    {
// Create Drawers. These are used by the Viewer
#ifndef ORBSLAM
      mpFrameDrawer = new DefFrameDrawer(mpMap);
      mpMapDrawer = new DefMapDrawer(mpMap, strSettingsFile);
#else
      mpFrameDrawer = new FrameDrawer(mpMap);
      mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
#endif
    }
// Initialize the Tracking thread
//(it will live in the main thread of execution, the one that called this
// constructor)
#ifdef ORBSLAM
    mpTracker =
        new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap,
                     mpKeyFrameDatabase, strSettingsFile, mSensor, bUseViewer);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mpMapDrawer, mSensor == MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

#else
    mpTracker = new DefTracking(this, mpVocabulary, mpFrameDrawer,
                                mpMapDrawer, mpMap, mpKeyFrameDatabase,
                                strSettingsFile, mSensor, bUseViewer);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new defSLAM::DefLocalMapping(
        mpMap, strSettingsFile);
#ifdef PARALLEL
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
#endif
#endif

// Initialize the Loop Closing thread and launch
#ifndef ORBSLAM
    mpLoopCloser =
        nullptr; // new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary,
    //      mSensor != MONOCULAR);
    mptLoopClosing =
        nullptr; // new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
#else

    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary,
                                   mSensor != MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
#endif

    // Initialize the Viewer thread and launch
    if (bUseViewer)
    {
#ifndef ORBSLAM
      mpViewer = new DefViewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,
                               strSettingsFile);
#else
      mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,
                            strSettingsFile);
#endif
      mptViewer = new std::thread(&Viewer::Run, mpViewer);
      mpTracker->SetViewer(mpViewer);
    }

    // Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpLocalMapper->SetTracker(mpTracker);

#ifdef ORBSLAM
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
#else
    mpTracker->SetLoopClosing(nullptr);
    mpLocalMapper->SetLoopCloser(nullptr);
#endif
  }

  cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp,
                                 const cv::Mat _mask)
  {
    cv::Mat Mask;
    if (_mask.empty())
    {
      Mask = cv::Mat(im.rows, im.cols, CV_8UC1, cv::Scalar(255));
    }
    else
    {
      Mask = _mask.clone();
    }

    if (mSensor != MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
              "Monocular."
           << endl;
      exit(-1);
    }

    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();
#ifdef PARALLEL
        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(1000);
        }
#endif
        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
      }
    }
#ifndef ORBSLAM
    // static_cast<DefLocalMapping *>(mpLocalMapper)->DoNotSaveResults();
#endif
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);
    if (mpViewer)
      mpViewer->Updatetimestamp(timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
    return Tcw;
  }

  void System::Restart(uint localzone, uint propagationzone)
  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);
    static_cast<DefMap *>(mpMap)->GetTemplate()->restart();
    std::vector<MapPoint *> mPoints = mpMap->GetAllMapPoints();

    for (std::vector<MapPoint *>::iterator pMP = mPoints.begin();
         pMP != mPoints.end(); pMP++)
    {
      if (static_cast<DefMapPoint *>(*pMP)->getFacet())
        static_cast<DefMapPoint *>(*pMP)->RecalculatePosition();
    }
  }

  cv::Mat System::TrackMonocularGT(const cv::Mat &im, const cv::Mat &imRight,
                                   const double &timestamp, const cv::Mat _mask)
  {
    if (mSensor != MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
              "Monocular."
           << endl;
      exit(-1);
    }
    cv::Mat Mask;
    if (_mask.empty())
    {
      Mask = cv::Mat(im.rows, im.cols, CV_8UC1, cv::Scalar(255));
    }
    else
    {
      Mask = _mask.clone();
    }
    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();
#ifdef PARALLEL

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(2000);
        }
#endif
        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
      }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocularGT(im, imRight, timestamp, Mask);
#ifndef ORBSLAM
#ifndef PARALLEL
    if (mpTracker->mState == Tracking::eTrackingState::OK)
      static_cast<DefLocalMapping *>(mpLocalMapper)->insideTheLoop();
#endif
    if (mpViewer)
    {
      mpViewer->Updatetimestamp(timestamp);
      while (!mpViewer->go())
        usleep(3000);
    }
#endif

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
    return Tcw;
  }

  cv::Mat System::TrackMonocularCTGT(const cv::Mat &im, const cv::Mat &CTdepth,
                                     const double &timestamp,
                                     const cv::Mat _mask)
  {
    if (mSensor != MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
              "Monocular."
           << endl;
      exit(-1);
    }
    cv::Mat Mask;
    if (_mask.empty())
    {
      Mask = cv::Mat(im.rows, im.cols, CV_8UC1, cv::Scalar(255));
    }
    else
    {
      Mask = _mask.clone();
    }
    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();
#ifdef PARALLEL

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(2000);
        }
#endif
        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
      }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocularCTGT(im, CTdepth, timestamp, Mask);
#ifndef ORBSLAM
#ifndef PARALLEL
    if (mpTracker->mState == Tracking::eTrackingState::OK)
      static_cast<DefLocalMapping *>(mpLocalMapper)->insideTheLoop();
#endif
    if (mpViewer)
    {
      mpViewer->Updatetimestamp(timestamp);
      while (!mpViewer->go())
        usleep(3000);
    }
#endif
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
    return Tcw;
  }

  void System::ActivateLocalizationMode()
  {
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
  }

  void System::DeactivateLocalizationMode()
  {
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
  }

  bool System::MapChanged()
  {
    static int n = 0;
    int curn = mpMap->GetLastBigChangeIdx();
    if (n < curn)
    {
      n = curn;
      return true;
    }
    else
      return false;
  }

  void System::Reset()
  {
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
  }

  void System::Shutdown()
  {
    mpLocalMapper->RequestFinish();
    if (mpLoopCloser)
      mpLoopCloser->RequestFinish();
    if (mpViewer)
    {
      mpViewer->RequestFinish();
      while (!mpViewer->isFinished())
        usleep(5000);
    }
    mptViewer->join();
    delete mptViewer;

    // Wait until all thread have effectively stopped
    if (mpLoopCloser)
      while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
             mpLoopCloser->isRunningGBA())
      {
        usleep(5000);
      }

    if (mpViewer)
#ifdef ORBSLAM
      pangolin::BindToContext("ORBSLAM2: Map Viewer");
#else
      pangolin::BindToContext("DefSLAM: Map Viewer");
#endif
  }

  int System::GetTrackingState()
  {
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
  }
  
  std::string System::getDataAsString()
  {
    cv::Mat Tcw = mpTracker->mCurrentFrame->mTcw.clone();
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> q = Converter::toQuaternion(Rwc);
    
    std::stringstream stream;
    stream <<
      // frame times
      std::setprecision(6) << mpTracker->mCurrentFrame->mTimeStamp << " "
      // pose
      <<  std::setprecision(9) << twc.at<float>(0)
                              << " " << twc.at<float>(1)
                              << " " << twc.at<float>(2) << " "
                              << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
      << endl;
    std::string data = stream.str();
      
    return data;
  }
  
  void System::SaveTrajectory(ofstream &f)
  {
    cv::Mat Tcw = mpTracker->mCurrentFrame->mTcw.clone();
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> q = Converter::toQuaternion(Rwc);

    f <<
      // frame times
      setprecision(6) << mpTracker->mCurrentFrame->mTimeStamp << " "
      // pose
      <<  setprecision(9) << twc.at<float>(0)
                          << " " << twc.at<float>(1)
                          << " " << twc.at<float>(2) << " "
                          << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
      << endl;
  }
      
  void System::updateTrajectory(const double &x, const double &y, const double &z, 
                    const double &qx, const double &qy, const double &qz, const double &qw)
  {
      std::cout << "Force updating trajectory!" << std::endl;
      
      // update currentFrame->Tcw
      Eigen::Matrix3d Rwc2 = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      cv::Mat Rwc = Converter::toCvMat(Rwc2);
      cv::Mat Rcw = Rwc.t();
      
      cv::Mat twc = cv::Mat(3, 1, CV_32F);
      twc.at<float>(0) = x;
      twc.at<float>(1) = y;
      twc.at<float>(2) = z;
      cv::Mat tcw = -Rcw*twc;
      
      cv::Mat Tcw = mpTracker->mCurrentFrame->mTcw;
      tcw.copyTo(Tcw.rowRange(0, 3).col(3));
      Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
      
      mpTracker->mCurrentFrame->SetPose(Tcw);

      // update velocity for motion model
      Frame lastFrame = static_cast<DefTracking *>(mpTracker)->getLastFrame();
        if (!lastFrame.mTcw.empty())
        {
          cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
          lastFrame.GetRotationInverse().copyTo(
              LastTwc.rowRange(0, 3).colRange(0, 3));
          lastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
          static_cast <DefTracking *>(mpTracker)->setVelocity(Tcw * LastTwc);
        }
        else
          static_cast <DefTracking *>(mpTracker)->setVelocity(cv::Mat());

      // update relative frame poses
      cv::Mat Tcr = Tcw * mpTracker->mCurrentFrame->mpReferenceKF->GetPoseInverse();
      mpTracker->mlRelativeFramePoses.pop_back();
      mpTracker->mlRelativeFramePoses.push_back(Tcr);

      // set to LOST
      mpTracker->mState = Tracking::eTrackingState::LOST;
      mpTracker->mlbLost.pop_back();
      mpTracker->mlbLost.push_back(true);
  }

} // namespace defSLAM
