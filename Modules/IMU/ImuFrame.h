#ifndef IMUFRAME_H
#define IMUFRAME_H

#include "Frame.h"

#include<vector>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "ImuTypes.h"
#include "ORBVocabulary.h"

#include <mutex>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM3{  
  using ORB_SLAM2::Frame;
  using ORB_SLAM2::ORBextractor;
  using ORB_SLAM2::ORBVocabulary;
  
  using IMU::Calib;
  using IMU::Bias;
  
  class ImuFrame : public Frame {
    public:
      ImuFrame();
        
      // Constructor for Monocular cameras.
      ImuFrame(const cv::Mat &imGray, const double &timeStamp,
        ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera,
        cv::Mat &distCoef, const float &bf, const float &thDepth,
        Frame* pPrevF = static_cast<Frame*>(NULL),
        const Calib &ImuCalib = Calib());
        
    void SetNewBias(const Bias &b);
  };
}

#endif