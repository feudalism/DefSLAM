#include "ImuFrame.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"

#include <thread>
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3
{
  ImuFrame::ImuFrame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc,
                GeometricCamera* pCamera,
                cv::Mat &distCoef, const float &bf, const float &thDepth,
                Frame* pPrevF, const Calib &ImuCalib)
       : Frame(imGray, timeStamp, extractor, voc, pCamera,
                distCoef, bf, thDepth, pPrevF, ImuCalib)
  {
    ImGray = imGray.clone();
  }
}