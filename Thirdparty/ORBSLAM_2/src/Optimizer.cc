/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include <Eigen/StdVector>

#include "Converter.h"
#include "Verbose.h"
#include "G2oTypes.h"
#include "OptimizableTypes.h"

#include <mutex>

namespace ORB_SLAM2
{
  using ORB_SLAM3::Verbose;
  using ORB_SLAM3::VertexPose;
  using ORB_SLAM3::VertexVelocity;
  using ORB_SLAM3::VertexGyroBias;
  using ORB_SLAM3::VertexAccBias;
  using ORB_SLAM3::VertexScale;
  using ORB_SLAM3::VertexGDir;
  
  using ORB_SLAM3::EdgeInertial;
  using ORB_SLAM3::EdgeGyroRW;
  using ORB_SLAM3::EdgeAccRW;
  using ORB_SLAM3::EdgeMono;
  using ORB_SLAM3::EdgeStereo;
  using ORB_SLAM3::EdgePriorAcc;
  using ORB_SLAM3::EdgePriorGyro;
  using ORB_SLAM3::EdgeInertialGS;
  using ORB_SLAM3::EdgeGyroRW;

  void Optimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
  {
    vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
  }

  void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                   int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
  {
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
      optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      KeyFrame *pKF = vpKFs[i];
      if (pKF->isBad())
        continue;
      g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
      vSE3->setId(pKF->mnId);
      vSE3->setFixed(pKF->mnId == 0);
      optimizer.addVertex(vSE3);
      if (pKF->mnId > maxKFid)
        maxKFid = pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for (size_t i = 0; i < vpMP.size(); i++)
    {
      MapPoint *pMP = vpMP[i];
      if (pMP->isBad())
        continue;
      g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
      vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
      const int id = pMP->mnId + maxKFid + 1;
      vPoint->setId(id);
      vPoint->setMarginalized(true);
      optimizer.addVertex(vPoint);

      const map<KeyFrame *, std::tuple<int, int>> observations = pMP->GetObservations();

      int nEdges = 0;
      //SET EDGES
      for (map<KeyFrame *, std::tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
      {

        KeyFrame *pKF = mit->first;
        if (pKF->isBad() || pKF->mnId > maxKFid)
          continue;

        nEdges++;

        const int leftIndex = get<0>(mit->second);
        const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

        if(leftIndex != -1 && pKF->mvuRight[get<0>(mit->second)]<0)
        {
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
          e->setMeasurement(obs);
          const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          if (bRobust)
          {
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);
          }

          e->fx = pKF->fx;
          e->fy = pKF->fy;
          e->cx = pKF->cx;
          e->cy = pKF->cy;

          optimizer.addEdge(e);
        }
        else // stereo?
        {
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
          e->setMeasurement(obs);
          const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          if (bRobust)
          {
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber3D);
          }

          e->fx = pKF->fx;
          e->fy = pKF->fy;
          e->cx = pKF->cx;
          e->cy = pKF->cy;
          e->bf = pKF->mbf;

          optimizer.addEdge(e);
        }
      }

      if (nEdges == 0)
      {
        optimizer.removeVertex(vPoint);
        vbNotIncludedMP[i] = true;
      }
      else
      {
        vbNotIncludedMP[i] = false;
      }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      KeyFrame *pKF = vpKFs[i];
      if (pKF->isBad())
        continue;
      g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
      g2o::SE3Quat SE3quat = vSE3->estimate();
      if (nLoopKF == 0)
      {
        pKF->SetPose(Converter::toCvMat(SE3quat));
      }
      else
      {
        pKF->mTcwGBA.create(4, 4, CV_32F);
        Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
        pKF->mnBAGlobalForKF = nLoopKF;
      }
    }

    //Points
    for (size_t i = 0; i < vpMP.size(); i++)
    {
      if (vbNotIncludedMP[i])
        continue;

      MapPoint *pMP = vpMP[i];

      if (pMP->isBad())
        continue;
      g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

      if (nLoopKF == 0)
      {
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
      }
      else
      {
        pMP->mPosGBA.create(3, 1, CV_32F);
        Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
        pMP->mnBAGlobalForKF = nLoopKF;
      }
    }
  }

  int Optimizer::poseOptimization(Frame *pFrame)
  {
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    // Set Frame vertex
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);

    {
      unique_lock<mutex> lock(MapPoint::mGlobalMutex);

      for (int i = 0; i < N; i++)
      {
        MapPoint *pMP = pFrame->mvpMapPoints[i];
        if (pMP)
        {
          // Monocular observation
          if (pFrame->mvuRight[i] < 0)
          {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->fx = pFrame->fx;
            e->fy = pFrame->fy;
            e->cx = pFrame->cx;
            e->cy = pFrame->cy;
            cv::Mat Xw = pMP->GetWorldPos();
            e->Xw[0] = Xw.at<float>(0);
            e->Xw[1] = Xw.at<float>(1);
            e->Xw[2] = Xw.at<float>(2);

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          }
          else // Stereo observation
          {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            //SET EDGE
            Eigen::Matrix<double, 3, 1> obs;
            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            const float &kp_ur = pFrame->mvuRight[i];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaStereo);

            e->fx = pFrame->fx;
            e->fy = pFrame->fy;
            e->cx = pFrame->cx;
            e->cy = pFrame->cy;
            e->bf = pFrame->mbf;
            cv::Mat Xw = pMP->GetWorldPos();
            e->Xw[0] = Xw.at<float>(0);
            e->Xw[1] = Xw.at<float>(1);
            e->Xw[2] = Xw.at<float>(2);

            optimizer.addEdge(e);

            vpEdgesStereo.push_back(e);
            vnIndexEdgeStereo.push_back(i);
          }
        }
      }
    }

    if (nInitialCorrespondences < 3)
      return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    const int its[4] = {10, 10, 10, 10};

    int nBad = 0;
    for (size_t it = 0; it < 4; it++)
    {

      vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
      optimizer.initializeOptimization(0);
      optimizer.optimize(its[it]);

      nBad = 0;
      for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
      {
        g2o::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

        const size_t idx = vnIndexEdgeMono[i];

        if (pFrame->mvbOutlier[idx])
        {
          e->computeError();
        }

        const float chi2 = e->chi2();

        if (chi2 > chi2Mono[it])
        {
          pFrame->mvbOutlier[idx] = true;
          e->setLevel(1);
          nBad++;
        }
        else
        {
          pFrame->mvbOutlier[idx] = false;
          e->setLevel(0);
        }

        if (it == 2)
          e->setRobustKernel(0);
      }

      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
      {
        g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

        const size_t idx = vnIndexEdgeStereo[i];

        if (pFrame->mvbOutlier[idx])
        {
          e->computeError();
        }

        const float chi2 = e->chi2();

        if (chi2 > chi2Stereo[it])
        {
          pFrame->mvbOutlier[idx] = true;
          e->setLevel(1);
          nBad++;
        }
        else
        {
          e->setLevel(0);
          pFrame->mvbOutlier[idx] = false;
        }

        if (it == 2)
          e->setRobustKernel(0);
      }

      if (optimizer.edges().size() < 10)
        break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences - nBad;
  }

  void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap)
  {
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame *> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
      KeyFrame *pKFi = vNeighKFs[i];
      pKFi->mnBALocalForKF = pKF->mnId;
      if (!pKFi->isBad())
        lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
      vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
      for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
      {
        MapPoint *pMP = *vit;
        if (pMP)
          if (!pMP->isBad())
            if (pMP->mnBALocalForKF != pKF->mnId)
            {
              lLocalMapPoints.push_back(pMP);
              pMP->mnBALocalForKF = pKF->mnId;
            }
      }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
      map<KeyFrame *, std::tuple<int, int>> observations = (*lit)->GetObservations();
      for (map<KeyFrame *, std::tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
      {
        KeyFrame *pKFi = mit->first;

        if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
        {
          pKFi->mnBAFixedForKF = pKF->mnId;
          if (!pKFi->isBad())
            lFixedCameras.push_back(pKFi);
        }
      }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
      optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
      KeyFrame *pKFi = *lit;
      g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
      vSE3->setId(pKFi->mnId);
      vSE3->setFixed(pKFi->mnId == 0);
      optimizer.addVertex(vSE3);
      if (pKFi->mnId > maxKFid)
        maxKFid = pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
      KeyFrame *pKFi = *lit;
      g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
      vSE3->setId(pKFi->mnId);
      vSE3->setFixed(true);
      optimizer.addVertex(vSE3);
      if (pKFi->mnId > maxKFid)
        maxKFid = pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
      MapPoint *pMP = *lit;
      g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
      vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
      int id = pMP->mnId + maxKFid + 1;
      vPoint->setId(id);
      vPoint->setMarginalized(true);
      optimizer.addVertex(vPoint);

      const map<KeyFrame *, std::tuple<int, int>> observations = pMP->GetObservations();

      //Set edges
      for (map<KeyFrame *, std::tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
      {
        KeyFrame *pKFi = mit->first;

        if (!pKFi->isBad())
        {
          const int leftIndex = get<0>(mit->second);
          const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];

          // Monocular observation
          if (pKFi->mvuRight[get<0>(mit->second)] < 0)
          {
            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);
            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            e->fx = pKFi->fx;
            e->fy = pKFi->fy;
            e->cx = pKFi->cx;
            e->cy = pKFi->cy;

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);
          }
          else // Stereo observation
          {
            Eigen::Matrix<double, 3, 1> obs;
            const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);
            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberStereo);

            e->fx = pKFi->fx;
            e->fy = pKFi->fy;
            e->cx = pKFi->cx;
            e->cy = pKFi->cy;
            e->bf = pKFi->mbf;

            optimizer.addEdge(e);
            vpEdgesStereo.push_back(e);
            vpEdgeKFStereo.push_back(pKFi);
            vpMapPointEdgeStereo.push_back(pMP);
          }
        }
      }
    }

    if (pbStopFlag)
      if (*pbStopFlag)
        return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
      if (*pbStopFlag)
        bDoMore = false;

    if (bDoMore)
    {

      // Check inlier observations
      for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
      {
        g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
          continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
          e->setLevel(1);
        }

        e->setRobustKernel(0);
      }

      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
      {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
          continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
          e->setLevel(1);
        }

        e->setRobustKernel(0);
      }

      // Optimize again without the outliers

      optimizer.initializeOptimization(0);
      optimizer.optimize(10);
    }

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
      g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
      MapPoint *pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad())
        continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive())
      {
        KeyFrame *pKFi = vpEdgeKFMono[i];
        vToErase.push_back(make_pair(pKFi, pMP));
      }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
      g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
      MapPoint *pMP = vpMapPointEdgeStereo[i];

      if (pMP->isBad())
        continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive())
      {
        KeyFrame *pKFi = vpEdgeKFStereo[i];
        vToErase.push_back(make_pair(pKFi, pMP));
      }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty())
    {
      for (size_t i = 0; i < vToErase.size(); i++)
      {
        KeyFrame *pKFi = vToErase[i].first;
        MapPoint *pMPi = vToErase[i].second;
        pKFi->EraseMapPointMatch(pMPi);
        pMPi->EraseObservation(pKFi);
      }
    }

    // Recover optimized data

    //Keyframes
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
      KeyFrame *pKF = *lit;
      g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
      g2o::SE3Quat SE3quat = vSE3->estimate();
      pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
      MapPoint *pMP = *lit;
      g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
      pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
      pMP->UpdateNormalAndDepth();
    }
  }

  void Optimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                         const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                         const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                         const map<KeyFrame *, set<KeyFrame *>> &LoopConnections, const bool &bFixScale)
  {
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
    vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
    {
      KeyFrame *pKF = vpKFs[i];
      if (pKF->isBad())
        continue;
      g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

      const int nIDi = pKF->mnId;

      LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

      if (it != CorrectedSim3.end())
      {
        vScw[nIDi] = it->second;
        VSim3->setEstimate(it->second);
      }
      else
      {
        Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
        Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
        g2o::Sim3 Siw(Rcw, tcw, 1.0);
        vScw[nIDi] = Siw;
        VSim3->setEstimate(Siw);
      }

      if (pKF == pLoopKF)
        VSim3->setFixed(true);

      VSim3->setId(nIDi);
      VSim3->setMarginalized(false);
      VSim3->_fix_scale = bFixScale;

      optimizer.addVertex(VSim3);

      vpVertices[nIDi] = VSim3;
    }

    set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

    const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

    // Set Loop edges
    for (map<KeyFrame *, set<KeyFrame *>>::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++)
    {
      KeyFrame *pKF = mit->first;
      const long unsigned int nIDi = pKF->mnId;
      const set<KeyFrame *> &spConnections = mit->second;
      const g2o::Sim3 Siw = vScw[nIDi];
      const g2o::Sim3 Swi = Siw.inverse();

      for (set<KeyFrame *>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++)
      {
        const long unsigned int nIDj = (*sit)->mnId;
        if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat)
          continue;

        const g2o::Sim3 Sjw = vScw[nIDj];
        const g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3 *e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;

        optimizer.addEdge(e);

        sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
      }
    }

    // Set normal edges
    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
    {
      KeyFrame *pKF = vpKFs[i];

      const int nIDi = pKF->mnId;

      g2o::Sim3 Swi;

      LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

      if (iti != NonCorrectedSim3.end())
        Swi = (iti->second).inverse();
      else
        Swi = vScw[nIDi].inverse();

      KeyFrame *pParentKF = pKF->GetParent();

      // Spanning tree edge
      if (pParentKF)
      {
        int nIDj = pParentKF->mnId;

        g2o::Sim3 Sjw;

        LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

        if (itj != NonCorrectedSim3.end())
          Sjw = itj->second;
        else
          Sjw = vScw[nIDj];

        g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3 *e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;
        optimizer.addEdge(e);
      }

      // Loop edges
      const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
      for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++)
      {
        KeyFrame *pLKF = *sit;
        if (pLKF->mnId < pKF->mnId)
        {
          g2o::Sim3 Slw;

          LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

          if (itl != NonCorrectedSim3.end())
            Slw = itl->second;
          else
            Slw = vScw[pLKF->mnId];

          g2o::Sim3 Sli = Slw * Swi;
          g2o::EdgeSim3 *el = new g2o::EdgeSim3();
          el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
          el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
          el->setMeasurement(Sli);
          el->information() = matLambda;
          optimizer.addEdge(el);
        }
      }

      // Covisibility graph edges
      const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
      for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++)
      {
        KeyFrame *pKFn = *vit;
        if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
        {
          if (!pKFn->isBad() && pKFn->mnId < pKF->mnId)
          {
            if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId))))
              continue;

            g2o::Sim3 Snw;

            LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

            if (itn != NonCorrectedSim3.end())
              Snw = itn->second;
            else
              Snw = vScw[pKFn->mnId];

            g2o::Sim3 Sni = Snw * Swi;

            g2o::EdgeSim3 *en = new g2o::EdgeSim3();
            en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
            en->setMeasurement(Sni);
            en->information() = matLambda;
            optimizer.addEdge(en);
          }
        }
      }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      KeyFrame *pKFi = vpKFs[i];

      const int nIDi = pKFi->mnId;

      g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
      g2o::Sim3 CorrectedSiw = VSim3->estimate();
      vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
      Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
      Eigen::Vector3d eigt = CorrectedSiw.translation();
      double s = CorrectedSiw.scale();

      eigt *= (1. / s); //[R t/s;0 1]

      cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

      pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
    {
      MapPoint *pMP = vpMPs[i];

      if (pMP->isBad())
        continue;

      int nIDr;
      if (pMP->mnCorrectedByKF == pCurKF->mnId)
      {
        nIDr = pMP->mnCorrectedReference;
      }
      else
      {
        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
        nIDr = pRefKF->mnId;
      }

      g2o::Sim3 Srw = vScw[nIDr];
      g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

      cv::Mat P3Dw = pMP->GetWorldPos();
      Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
      Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

      cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
      pMP->SetWorldPos(cvCorrectedP3Dw);

      pMP->UpdateNormalAndDepth();
    }
  }

  int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
  {
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale = bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0, 2);
    vSim3->_principle_point1[1] = K1.at<float>(1, 2);
    vSim3->_focal_length1[0] = K1.at<float>(0, 0);
    vSim3->_focal_length1[1] = K1.at<float>(1, 1);
    vSim3->_principle_point2[0] = K2.at<float>(0, 2);
    vSim3->_principle_point2[1] = K2.at<float>(1, 2);
    vSim3->_focal_length2[0] = K2.at<float>(0, 0);
    vSim3->_focal_length2[1] = K2.at<float>(1, 1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2 * N);
    vpEdges12.reserve(2 * N);
    vpEdges21.reserve(2 * N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for (int i = 0; i < N; i++)
    {
      if (!vpMatches1[i])
        continue;

      MapPoint *pMP1 = vpMapPoints1[i];
      MapPoint *pMP2 = vpMatches1[i];

      const int id1 = 2 * i + 1;
      const int id2 = 2 * (i + 1);

      const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));

      if (pMP1 && pMP2)
      {
        if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0)
        {
          g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
          cv::Mat P3D1w = pMP1->GetWorldPos();
          cv::Mat P3D1c = R1w * P3D1w + t1w;
          vPoint1->setEstimate(Converter::toVector3d(P3D1c));
          vPoint1->setId(id1);
          vPoint1->setFixed(true);
          optimizer.addVertex(vPoint1);

          g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
          cv::Mat P3D2w = pMP2->GetWorldPos();
          cv::Mat P3D2c = R2w * P3D2w + t2w;
          vPoint2->setEstimate(Converter::toVector3d(P3D2c));
          vPoint2->setId(id2);
          vPoint2->setFixed(true);
          optimizer.addVertex(vPoint2);
        }
        else
          continue;
      }
      else
        continue;

      nCorrespondences++;

      // Set edge x1 = S12*X2
      Eigen::Matrix<double, 2, 1> obs1;
      const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
      obs1 << kpUn1.pt.x, kpUn1.pt.y;

      g2o::EdgeSim3ProjectXYZ *e12 = new g2o::EdgeSim3ProjectXYZ();
      e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
      e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
      e12->setMeasurement(obs1);
      const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
      e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

      g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
      e12->setRobustKernel(rk1);
      rk1->setDelta(deltaHuber);
      optimizer.addEdge(e12);

      // Set edge x2 = S21*X1
      Eigen::Matrix<double, 2, 1> obs2;
      const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
      obs2 << kpUn2.pt.x, kpUn2.pt.y;

      g2o::EdgeInverseSim3ProjectXYZ *e21 = new g2o::EdgeInverseSim3ProjectXYZ();

      e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
      e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
      e21->setMeasurement(obs2);
      float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
      e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

      g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
      e21->setRobustKernel(rk2);
      rk2->setDelta(deltaHuber);
      optimizer.addEdge(e21);

      vpEdges12.push_back(e12);
      vpEdges21.push_back(e21);
      vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad = 0;
    for (size_t i = 0; i < vpEdges12.size(); i++)
    {
      g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
      g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
      if (!e12 || !e21)
        continue;

      if (e12->chi2() > th2 || e21->chi2() > th2)
      {
        size_t idx = vnIndexEdge[i];
        vpMatches1[idx] = static_cast<MapPoint *>(NULL);
        optimizer.removeEdge(e12);
        optimizer.removeEdge(e21);
        vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ *>(NULL);
        vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ *>(NULL);
        nBad++;
      }
    }

    int nMoreIterations;
    if (nBad > 0)
      nMoreIterations = 10;
    else
      nMoreIterations = 5;

    if (nCorrespondences - nBad < 10)
      return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for (size_t i = 0; i < vpEdges12.size(); i++)
    {
      g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
      g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
      if (!e12 || !e21)
        continue;

      if (e12->chi2() > th2 || e21->chi2() > th2)
      {
        size_t idx = vnIndexEdge[i];
        vpMatches1[idx] = static_cast<MapPoint *>(NULL);
      }
      else
        nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
    g2oS12 = vSim3_recov->estimate();

    return nIn;
  }

// OS3
void Optimizer::LocalInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, bool bLarge, bool bRecInit)
{
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Map* pCurrentMap = pKF->GetMap();

    int maxOpt=10;
    int opt_it=10;
    if(bLarge)
    {
        maxOpt=25;
        opt_it=4;
    }
    const int Nd = std::min((int)pCurrentMap->KeyFramesInMap()-2,maxOpt);
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFrame*> vpOptimizableKFs;
    const vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
    list<KeyFrame*> lpOptVisKFs;

    vpOptimizableKFs.reserve(Nd);
    vpOptimizableKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    for(int i=1; i<Nd; i++)
    {
        if(vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
        }
        else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by temporal optimizable keyframes
    list<MapPoint*> lLocalMapPoints;
    for(int i=0; i<N; i++)
    {
        vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    list<KeyFrame*> lFixedKeyFrames;
    if(vpOptimizableKFs.back()->mPrevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF=pKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->mnBALocalForKF=0;
        vpOptimizableKFs.back()->mnBAFixedForKF=pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Optimizable visual KFs
    const int maxCovKF = 0;
    for(int i=0, iend=vpNeighsKFs.size(); i<iend; i++)
    {
        if(lpOptVisKFs.size() >= maxCovKF)
            break;

        KeyFrame* pKFi = vpNeighsKFs[i];
        if(pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
            continue;
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
        {
            lpOptVisKFs.push_back(pKFi);

            vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
            for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
            {
                MapPoint* pMP = *vit;
                if(pMP)
                    if(!pMP->isBad())
                        if(pMP->mnBALocalForKF!=pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF=pKF->mnId;
                        }
            }
        }
    }

    // Fixed KFs which are not covisible optimizable
    const int maxFixKF = 200;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if(lFixedKeyFrames.size()>=maxFixKF)
            break;
    }

    bool bNonFixed = (lFixedKeyFrames.size() == 0);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    if(bLarge)
    {
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e-2); // to avoid iterating for finding optimal lambda
        optimizer.setAlgorithm(solver);
    }
    else
    {
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e0);
        optimizer.setAlgorithm(solver);
    }


    // Set Local temporal KeyFrame vertices
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Local visual KeyFrame vertices
    for(list<KeyFrame*>::iterator it=lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it!=itEnd; it++)
    {
        KeyFrame* pKFi = *it;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if(pKFi->bImu) // This should be done only for keyframe just before temporal window
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
    }

    // Create intertial constraints
    vector<EdgeInertial*> vei(N,(EdgeInertial*)NULL);
    vector<EdgeGyroRW*> vegr(N,(EdgeGyroRW*)NULL);
    vector<EdgeAccRW*> vear(N,(EdgeAccRW*)NULL);

    for(int i=0;i<N;i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        if(!pKFi->mPrevKF)
        {
            cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
            continue;
        }
        if(pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
        {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
            g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
            g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);

            if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
            {
                cerr << "Error " << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

            vei[i]->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            if(i==N-1 || bRecInit)
            {
                // All inertial residuals are included without robust cost function, but not that one linking the
                // last optimizable keyframe inside of the local window and the first fixed keyframe out. The
                // information matrix for this measurement is also downweighted. This is done to avoid accumulating
                // error due to fixing variables.
                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                if(i==N-1)
                    vei[i]->setInformation(vei[i]->information()*1e-2);
                rki->setDelta(sqrt(16.92));
            }
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0,VG1);
            vegr[i]->setVertex(1,VG2);
            cv::Mat cvInfoG = pKFi->mpImuPreintegrated->C.rowRange(9,12).colRange(9,12).inv(cv::DECOMP_SVD);
            Eigen::Matrix3d InfoG;

            for(int r=0;r<3;r++)
                for(int c=0;c<3;c++)
                    InfoG(r,c)=cvInfoG.at<float>(r,c);
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            // cout << "b";
            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0,VA1);
            vear[i]->setVertex(1,VA2);
            cv::Mat cvInfoA = pKFi->mpImuPreintegrated->C.rowRange(12,15).colRange(12,15).inv(cv::DECOMP_SVD);
            Eigen::Matrix3d InfoA;
            for(int r=0;r<3;r++)
                for(int c=0;c<3;c++)
                    InfoA(r,c)=cvInfoA.at<float>(r,c);
            vear[i]->setInformation(InfoA);           

            optimizer.addEdge(vear[i]);
        }
        else
            cout << "ERROR building inertial edge" << endl;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (N+lFixedKeyFrames.size())*lLocalMapPoints.size();

    // Mono
    vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // Stereo
    vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);



    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

    const unsigned long iniMPid = maxKFid*5;

    map<int,int> mVisEdges;
    for(int i=0;i<N;i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];
        mVisEdges[pKFi->mnId] = 0;
    }
    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        mVisEdges[(*lit)->mnId] = 0;
    }

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));

        unsigned long id = pMP->mnId+iniMPid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        // Create visual constraints
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
                continue;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                const int leftIndex = get<0>(mit->second);

                cv::KeyPoint kpUn;

                // Monocular left observation
                if(leftIndex != -1 && pKFi->mvuRight[leftIndex]<0)
                {
                    mVisEdges[pKFi->mnId]++;

                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                // Stereo-observation
                else if(leftIndex != -1)// Stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    mVisEdges[pKFi->mnId]++;

                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }

                // Monocular right observation
                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;
                        mVisEdges[pKFi->mnId]++;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        EdgeMono* e = new EdgeMono(1);

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                }
            }
        }
    }

    //cout << "Total map points: " << lLocalMapPoints.size() << endl;
    for(map<int,int>::iterator mit=mVisEdges.begin(), mend=mVisEdges.end(); mit!=mend; mit++)
    {
        assert(mit->second>=3);
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float err = optimizer.activeRobustChi2();
    optimizer.optimize(opt_it); // Originally to 2
    float err_end = optimizer.activeRobustChi2();
    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);


    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        EdgeMono* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];
        bool bClose = pMP->mTrackDepth<10.f;

        if(pMP->isBad())
            continue;

        if((e->chi2()>chi2Mono2 && !bClose) || (e->chi2()>1.5f*chi2Mono2 && bClose) || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }


    // Stereo
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>chi2Stereo2)
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex and erase outliers
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);


    // TODO: Some convergence problems have been detected here
    //cout << "err0 = " << err << endl;
    //cout << "err_end = " << err_end << endl;
    if((2*err < err_end || isnan(err) || isnan(err_end)) && !bLarge) //bGN)
    {
        cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
        return;
    }



    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }



    // Display main statistcis of optimization
    Verbose::PrintMess("LIBA KFs: " + to_string(N), Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("LIBA bNonFixed?: " + to_string(bNonFixed), Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("LIBA KFs visual outliers: " + to_string(vToErase.size()), Verbose::VERBOSITY_DEBUG);

    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
        (*lit)->mnBAFixedForKF = 0;

    // Recover optimized data
    // Local temporal Keyframes
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF=0;

        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            pKFi->SetVelocity(Converter::toCvMat(VV->estimate()));
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            ORB_SLAM3::Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(defSLAM::IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]));

        }
    }

    // Local visual KeyFrame
    for(list<KeyFrame*>::iterator it=lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it!=itEnd; it++)
    {
        KeyFrame* pKFi = *it;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF=0;
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();

    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    /*double t_const = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();
    double t_opt = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    double t_rec = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    /*std::cout << " Construction time: " << t_const << std::endl;
    std::cout << " Optimization time: " << t_opt << std::endl;
    std::cout << " Recovery time: " << t_rec << std::endl;
    std::cout << " Total time: " << t_const+t_opt+t_rec << std::endl;
    std::cout << " Optimization iterations: " << opt_it << std::endl;*/

}


void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel, bool bGauss, float priorG, float priorA)
{
    Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
    int its = 200; // Check number of iterations
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    if (priorG!=0.f)
        solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);


    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+(pKFi->mnId)+1);
        if (bFixedVel)
            VV->setFixed(true);
        else
            VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid*2+2);
    if (bFixedVel)
        VG->setFixed(true);
    else
        VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid*2+3);
    if (bFixedVel)
        VA->setFixed(true);
    else
        VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    EdgePriorAcc* epa = new EdgePriorAcc(cv::Mat::zeros(3,1,CV_32F));
    epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro* epg = new EdgePriorGyro(cv::Mat::zeros(3,1,CV_32F));
    epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Rwg);
    VGDir->setId(maxKFid*2+4);
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(scale);
    VS->setId(maxKFid*2+5);
    VS->setFixed(!bMono); // Fixed for stereo case
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS*> vpei;
    vpei.reserve(vpKFs.size());
    vector<pair<KeyFrame*,KeyFrame*> > vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());
    std::cout << "build optimization graph" << std::endl;

    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;
            if(!pKFi->mpImuPreintegrated)
                std::cout << "Not preintegrated measurement" << std::endl;

            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid*2+2);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid*2+3);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid*2+4);
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid*2+5);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG << ", "<< VA << ", " << VP2 << ", " << VV2 <<  ", "<< VGDir << ", "<< VS <<endl;

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            vpei.push_back(ei);

            vppUsedKF.push_back(make_pair(pKFi->mPrevKF,pKFi));
            optimizer.addEdge(ei);

        }
    }

    // Compute error for different scales
    std::set<g2o::HyperGraph::Edge*> setEdges = optimizer.edges();

    std::cout << "start optimization" << std::endl;
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);

    std::cout << "end optimization" << std::endl;

    scale = VS->estimate();

    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid*2+2));
    VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid*2+3));
    ORB_SLAM3::Vector6d vb;
    vb << VG->estimate(), VA->estimate();
    bg << VG->estimate();
    ba << VA->estimate();
    scale = VS->estimate();


    defSLAM::IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);
    Rwg = VGDir->estimate().Rwg;

    cv::Mat cvbg = Converter::toCvMat(bg);

    //Keyframes velocities and biases
    std::cout << "update Keyframes velocities and biases" << std::endl;

    const int N = vpKFs.size();
    for(size_t i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;

        VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+(pKFi->mnId)+1));
        Eigen::Vector3d Vw = VV->estimate(); // Velocity is scaled after
        pKFi->SetVelocity(Converter::toCvMat(Vw));

        if (cv::norm(pKFi->GetGyroBias()-cvbg)>0.01)
        {
            pKFi->SetNewBias(b);
            if (pKFi->mpImuPreintegrated)
                pKFi->mpImuPreintegrated->Reintegrate();
        }
        else
            pKFi->SetNewBias(b);


    }
}


void Optimizer::InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG, float priorA)
{
    int its = 200; // Check number of iterations
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);


    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+(pKFi->mnId)+1);
        VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid*2+2);
    VG->setFixed(false);
    optimizer.addVertex(VG);

    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid*2+3);
    VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    EdgePriorAcc* epa = new EdgePriorAcc(cv::Mat::zeros(3,1,CV_32F));
    epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro* epg = new EdgePriorGyro(cv::Mat::zeros(3,1,CV_32F));
    epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
    VGDir->setId(maxKFid*2+4);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(1.0);
    VS->setId(maxKFid*2+5);
    VS->setFixed(true); // Fixed since scale is obtained from already well initialized map
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS*> vpei;
    vpei.reserve(vpKFs.size());
    vector<pair<KeyFrame*,KeyFrame*> > vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());

    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;

            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid*2+2);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid*2+3);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid*2+4);
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid*2+5);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG << ", "<< VA << ", " << VP2 << ", " << VV2 <<  ", "<< VGDir << ", "<< VS <<endl;

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            vpei.push_back(ei);

            vppUsedKF.push_back(make_pair(pKFi->mPrevKF,pKFi));
            optimizer.addEdge(ei);

        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);


    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid*2+2));
    VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid*2+3));
    ORB_SLAM3::Vector6d vb;
    vb << VG->estimate(), VA->estimate();
    bg << VG->estimate();
    ba << VA->estimate();

    defSLAM::IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);

    cv::Mat cvbg = Converter::toCvMat(bg);

    //Keyframes velocities and biases
    const int N = vpKFs.size();
    for(size_t i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;

        VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+(pKFi->mnId)+1));
        Eigen::Vector3d Vw = VV->estimate();
        pKFi->SetVelocity(Converter::toCvMat(Vw));

        if (cv::norm(pKFi->GetGyroBias()-cvbg)>0.01)
        {
            pKFi->SetNewBias(b);
            if (pKFi->mpImuPreintegrated)
                pKFi->mpImuPreintegrated->Reintegrate();
        }
        else
            pKFi->SetNewBias(b);
    }
}

void Optimizer::InertialOptimization(vector<KeyFrame*> vpKFs, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG, float priorA)
{
    int its = 200; // Check number of iterations
    long unsigned int maxKFid = vpKFs[0]->GetMap()->GetMaxKFid();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);


    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        //if(pKFi->mnId>maxKFid)
        //    continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+(pKFi->mnId)+1);
        VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid*2+2);
    VG->setFixed(false);
    optimizer.addVertex(VG);

    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid*2+3);
    VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    EdgePriorAcc* epa = new EdgePriorAcc(cv::Mat::zeros(3,1,CV_32F));
    epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro* epg = new EdgePriorGyro(cv::Mat::zeros(3,1,CV_32F));
    epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
    VGDir->setId(maxKFid*2+4);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(1.0);
    VS->setId(maxKFid*2+5);
    VS->setFixed(true); // Fixed since scale is obtained from already well initialized map
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS*> vpei;
    vpei.reserve(vpKFs.size());
    vector<pair<KeyFrame*,KeyFrame*> > vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());

    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;

            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid*2+2);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid*2+3);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid*2+4);
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid*2+5);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG << ", "<< VA << ", " << VP2 << ", " << VV2 <<  ", "<< VGDir << ", "<< VS <<endl;

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            vpei.push_back(ei);

            vppUsedKF.push_back(make_pair(pKFi->mPrevKF,pKFi));
            optimizer.addEdge(ei);

        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);


    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid*2+2));
    VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid*2+3));
    ORB_SLAM3::Vector6d vb;
    vb << VG->estimate(), VA->estimate();
    bg << VG->estimate();
    ba << VA->estimate();

    defSLAM::IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);

    cv::Mat cvbg = Converter::toCvMat(bg);

    //Keyframes velocities and biases
    const int N = vpKFs.size();
    for(size_t i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;

        VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+(pKFi->mnId)+1));
        Eigen::Vector3d Vw = VV->estimate();
        pKFi->SetVelocity(Converter::toCvMat(Vw));

        if (cv::norm(pKFi->GetGyroBias()-cvbg)>0.01)
        {
            pKFi->SetNewBias(b);
            if (pKFi->mpImuPreintegrated)
                pKFi->mpImuPreintegrated->Reintegrate();
        }
        else
            pKFi->SetNewBias(b);
    }
}


void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale)
{
    int its = 10;
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (all variables are fixed)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+1+(pKFi->mnId));
        VV->setFixed(true);
        optimizer.addVertex(VV);

        // Vertex of fixed biases
        VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
        VG->setId(2*(maxKFid+1)+(pKFi->mnId));
        VG->setFixed(true);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(vpKFs.front());
        VA->setId(3*(maxKFid+1)+(pKFi->mnId));
        VA->setFixed(true);
        optimizer.addVertex(VA);
    }

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Rwg);
    VGDir->setId(4*(maxKFid+1));
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(scale);
    VS->setId(4*(maxKFid+1)+1);
    VS->setFixed(false);
    optimizer.addVertex(VS);

    // Graph edges
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;

            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex((maxKFid+1)+pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex((maxKFid+1)+pKFi->mnId);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(2*(maxKFid+1)+pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(3*(maxKFid+1)+pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(4*(maxKFid+1));
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(4*(maxKFid+1)+1);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                Verbose::PrintMess("Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) + ", " + to_string(VG->id()) + ", " + to_string(VA->id()) + ", " + to_string(VP2->id()) + ", " + to_string(VV2->id()) +  ", " + to_string(VGDir->id()) + ", " + to_string(VS->id()), Verbose::VERBOSITY_NORMAL);

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            optimizer.addEdge(ei);
        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);

    // Recover optimized data
    scale = VS->estimate();
    Rwg = VGDir->estimate().Rwg;
}

void Optimizer::FullInertialBA(Map *pMap, int its, const bool bFixLocal, const long unsigned int nLoopId, bool *pbStopFlag, bool bInit, float priorG, float priorA, Eigen::VectorXd *vSingVal, bool *bHess)
{
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int nNonFixed = 0;

    // Set KeyFrame vertices
    KeyFrame* pIncKF;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        pIncKF=pKFi;
        bool bFixed = false;
        if(bFixLocal)
        {
            bFixed = (pKFi->mnBALocalForKF>=(maxKFid-1)) || (pKFi->mnBAFixedForKF>=(maxKFid-1));
            if(!bFixed)
                nNonFixed++;
            VP->setFixed(bFixed);
        }
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(bFixed);
            optimizer.addVertex(VV);
            if (!bInit)
            {
                VertexGyroBias* VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid+3*(pKFi->mnId)+2);
                VG->setFixed(bFixed);
                optimizer.addVertex(VG);
                VertexAccBias* VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid+3*(pKFi->mnId)+3);
                VA->setFixed(bFixed);
                optimizer.addVertex(VA);
            }
        }
    }

    if (bInit)
    {
        VertexGyroBias* VG = new VertexGyroBias(pIncKF);
        VG->setId(4*maxKFid+2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pIncKF);
        VA->setId(4*maxKFid+3);
        VA->setFixed(false);
        optimizer.addVertex(VA);
    }

    if(bFixLocal)
    {
        if(nNonFixed<3)
            return;
    }

    // IMU links
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(!pKFi->mPrevKF)
        {
            Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!", Verbose::VERBOSITY_NORMAL);
            continue;
        }

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;
            if(pKFi->bImu && pKFi->mPrevKF->bImu)
            {
                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);

                g2o::HyperGraph::Vertex* VG1;
                g2o::HyperGraph::Vertex* VA1;
                g2o::HyperGraph::Vertex* VG2;
                g2o::HyperGraph::Vertex* VA2;
                if (!bInit)
                {
                    VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
                    VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
                    VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
                    VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);
                }
                else
                {
                    VG1 = optimizer.vertex(4*maxKFid+2);
                    VA1 = optimizer.vertex(4*maxKFid+3);
                }

                g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);

                if (!bInit)
                {
                    if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
                    {
                        cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;

                        continue;
                    }
                }
                else
                {
                    if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2)
                    {
                        cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<endl;

                        continue;
                    }
                }

                EdgeInertial* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
                ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
                ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
                ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
                ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
                ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
                ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rki);
                rki->setDelta(sqrt(16.92));

                optimizer.addEdge(ei);

                if (!bInit)
                {
                    EdgeGyroRW* egr= new EdgeGyroRW();
                    egr->setVertex(0,VG1);
                    egr->setVertex(1,VG2);
                    cv::Mat cvInfoG = pKFi->mpImuPreintegrated->C.rowRange(9,12).colRange(9,12).inv(cv::DECOMP_SVD);
                    Eigen::Matrix3d InfoG;
                    for(int r=0;r<3;r++)
                        for(int c=0;c<3;c++)
                            InfoG(r,c)=cvInfoG.at<float>(r,c);
                    egr->setInformation(InfoG);
                    egr->computeError();
                    optimizer.addEdge(egr);

                    EdgeAccRW* ear = new EdgeAccRW();
                    ear->setVertex(0,VA1);
                    ear->setVertex(1,VA2);
                    cv::Mat cvInfoA = pKFi->mpImuPreintegrated->C.rowRange(12,15).colRange(12,15).inv(cv::DECOMP_SVD);
                    Eigen::Matrix3d InfoA;
                    for(int r=0;r<3;r++)
                        for(int c=0;c<3;c++)
                            InfoA(r,c)=cvInfoA.at<float>(r,c);
                    ear->setInformation(InfoA);
                    ear->computeError();
                    optimizer.addEdge(ear);
                }
            }
            else
            {
                cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu" << endl;
            }
        }
    }

    if (bInit)
    {
        g2o::HyperGraph::Vertex* VG = optimizer.vertex(4*maxKFid+2);
        g2o::HyperGraph::Vertex* VA = optimizer.vertex(4*maxKFid+3);

        // Add prior to comon biases
        EdgePriorAcc* epa = new EdgePriorAcc(cv::Mat::zeros(3,1,CV_32F));
        epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
        double infoPriorA = priorA; //
        epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);

        EdgePriorGyro* epg = new EdgePriorGyro(cv::Mat::zeros(3,1,CV_32F));
        epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
        double infoPriorG = priorG; //
        epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);
    }

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    const unsigned long iniMPid = maxKFid*5;

    vector<bool> vbNotIncludedMP(vpMPs.size(),false);

    for(size_t i=0; i<vpMPs.size(); i++)
    {
        MapPoint* pMP = vpMPs[i];
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        unsigned long id = pMP->mnId+iniMPid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();


        bool bAllFixed = true;

        //Set edges
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnId>maxKFid)
                continue;

            if(!pKFi->isBad())
            {
                const int leftIndex = get<0>(mit->second);
                cv::KeyPoint kpUn;

                if(leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)]<0) // Monocular observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    if(bAllFixed)
                        if(!VP->fixed())
                            bAllFixed=false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                }
                else if(leftIndex != -1 && pKFi->mvuRight[leftIndex] >= 0) // stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    if(bAllFixed)
                        if(!VP->fixed())
                            bAllFixed=false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                }

                if(pKFi->mpCamera2){ // Monocular right observation
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 && rightIndex < pKFi->mvKeysRight.size()){
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double,2,1> obs;
                        kpUn = pKFi->mvKeysRight[rightIndex];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMono *e = new EdgeMono(1);

                        g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                        if(bAllFixed)
                            if(!VP->fixed())
                                bAllFixed=false;

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                    }
                }
            }
        }

        if(bAllFixed)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;


    optimizer.initializeOptimization();
    optimizer.optimize(its);


    // Recover optimized data
    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        if(nLoopId==0)
        {
            cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
            pKFi->SetPose(Tcw);
        }
        else
        {
            pKFi->mTcwGBA = cv::Mat::eye(4,4,CV_32F);
            Converter::toCvMat(VP->estimate().Rcw[0]).copyTo(pKFi->mTcwGBA.rowRange(0,3).colRange(0,3));
            Converter::toCvMat(VP->estimate().tcw[0]).copyTo(pKFi->mTcwGBA.rowRange(0,3).col(3));
            pKFi->mnBAGlobalForKF = nLoopId;

        }
        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            if(nLoopId==0)
            {
                pKFi->SetVelocity(Converter::toCvMat(VV->estimate()));
            }
            else
            {
                pKFi->mVwbGBA = Converter::toCvMat(VV->estimate());
            }

            VertexGyroBias* VG;
            VertexAccBias* VA;
            if (!bInit)
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            }
            else
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(4*maxKFid+2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(4*maxKFid+3));
            }

            ORB_SLAM3::Vector6d vb;
            vb << VG->estimate(), VA->estimate();
            defSLAM::IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);
            if(nLoopId==0)
            {
                pKFi->SetNewBias(b);
            }
            else
            {
                pKFi->mBiasGBA = b;
            }
        }
    }

    //Points
    for(size_t i=0; i<vpMPs.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMPs[i];
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));

        if(nLoopId==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopId;
        }

    }

    pMap->IncreaseChangeIndex();
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, vector<KeyFrame*> &vpNonEnoughOptKFs)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    Map* pCurrentMap = pKF->GetMap();

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            lLocalKeyFrames.push_back(pKFi);
    }
    for(KeyFrame* pKFi : vpNonEnoughOptKFs)
    {
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap && pKFi->mnBALocalForKF != pKF->mnId)
        {
            pKFi->mnBALocalForKF = pKF->mnId;
            lLocalKeyFrames.push_back(pKFi);
        }
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    set<MapPoint*> sNumObsMP;
    int num_fixedKF;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if(pKFi->mnId==pCurrentMap->GetInitKFid())
        {
            num_fixedKF = 1;
        }
        vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                {

                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }

                }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId )
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
    num_fixedKF = lFixedCameras.size() + num_fixedKF;
    if(num_fixedKF < 2)
    {
        //Verbose::PrintMess("LM-LBA: New Fixed KFs had been set", Verbose::VERBOSITY_NORMAL);
        //TODO We set 2 KFs to fixed to avoid a degree of freedom in scale
        list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin();
        int lowerId = pKF->mnId;
        KeyFrame* pLowerKf;
        int secondLowerId = pKF->mnId;
        KeyFrame* pSecondLowerKF;

        for(; lit != lLocalKeyFrames.end(); lit++)
        {
            KeyFrame* pKFi = *lit;
            if(pKFi == pKF || pKFi->mnId == pCurrentMap->GetInitKFid())
            {
                continue;
            }

            if(pKFi->mnId < lowerId)
            {
                lowerId = pKFi->mnId;
                pLowerKf = pKFi;
            }
            else if(pKFi->mnId < secondLowerId)
            {
                secondLowerId = pKFi->mnId;
                pSecondLowerKF = pKFi;
            }
        }
        lFixedCameras.push_back(pLowerKf);
        lLocalKeyFrames.remove(pLowerKf);
        num_fixedKF++;
        if(num_fixedKF < 2)
        {
            lFixedCameras.push_back(pSecondLowerKF);
            lLocalKeyFrames.remove(pSecondLowerKF);
            num_fixedKF++;
        }
    }

    if(num_fixedKF == 0)
    {
        Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
        //return;
    }
    //Verbose::PrintMess("LM-LBA: There are " + to_string(lLocalKeyFrames.size()) + " KFs and " + to_string(lLocalMapPoints.size()) + " MPs to optimize. " + to_string(num_fixedKF) + " KFs are fixed", Verbose::VERBOSITY_DEBUG);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    if (pCurrentMap->IsInertial())
        solver->setUserLambdaInit(100.0); // TODO uncomment
    //cout << "LM-LBA: lambda init: " << solver->userLambdaInit() << endl;

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==pCurrentMap->GetInitKFid());
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    Verbose::PrintMess("LM-LBA: opt/fixed KFs: " + to_string(lLocalKeyFrames.size()) + "/" + to_string(lFixedCameras.size()), Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("LM-LBA: local MPs: " + to_string(lLocalMapPoints.size()), Verbose::VERBOSITY_DEBUG);

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    int nPoints = 0;

    int nKFs = lLocalKeyFrames.size()+lFixedCameras.size(), nEdges = 0;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        nPoints++;

        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                const int cam0Index = get<0>(mit->second);

                // Monocular observation of Camera 0
                if(cam0Index != -1 && pKFi->mvuRight[cam0Index]<0)
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[cam0Index];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->pCamera = pKFi->mpCamera;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    nEdges++;
                }
                else if(cam0Index != -1 && pKFi->mvuRight[cam0Index]>=0)// Stereo observation (with rectified images)
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[cam0Index];
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[cam0Index];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    nEdges++;
                }

                // Monocular observation of Camera 0
                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->mTrl = Converter::toSE3Quat(pKFi->mTrl);

                        e->pCamera = pKFi->mpCamera2;

                        optimizer.addEdge(e);
                        vpEdgesBody.push_back(e);
                        vpEdgeKFBody.push_back(pKFi);
                        vpMapPointEdgeBody.push_back(pMP);

                        nEdges++;
                    }
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
        {
            return;
        }

    optimizer.initializeOptimization();

    //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int numPerform_it = optimizer.optimize(5);
    //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();



    //std::cout << "LBA time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    //std::cout << "Keyframes: " << nKFs << " --- MapPoints: " << nPoints << " --- Edges: " << nEdges << endl;

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

        // Check inlier observations
        int nMonoBadObs = 0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                //e->setLevel(1);
                nMonoBadObs++;
            }

            //e->setRobustKernel(0);
        }

        int nBodyBadObs = 0;
        for(size_t i=0, iend=vpEdgesBody.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
            MapPoint* pMP = vpMapPointEdgeBody[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                //e->setLevel(1);
                nBodyBadObs++;
            }

            //e->setRobustKernel(0);
        }

        int nStereoBadObs = 0;
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                //e->setLevel(1);
                nStereoBadObs++;
            }

            //e->setRobustKernel(0);
        }
        Verbose::PrintMess("LM-LBA: First optimization has " + to_string(nMonoBadObs) + " monocular and " + to_string(nStereoBadObs) + " stereo bad observations", Verbose::VERBOSITY_DEBUG);

        // Optimize again without the outliers
        //Verbose::PrintMess("LM-LBA: second optimization", Verbose::VERBOSITY_DEBUG);
        //optimizer.initializeOptimization(0);
        //numPerform_it = optimizer.optimize(10);
        numPerform_it += optimizer.optimize(5);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesBody.size()+vpEdgesStereo.size());

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesBody.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
        MapPoint* pMP = vpMapPointEdgeBody[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFBody[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    Verbose::PrintMess("LM-LBA: outlier observations: " + to_string(vToErase.size()), Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("LM-LBA: total of observations: " + to_string(vpMapPointEdgeMono.size()+vpMapPointEdgeStereo.size()), Verbose::VERBOSITY_DEBUG);
    bool bRedrawError = false;
    bool bWriteStats = false;

    // Get Map Mutex
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {

        //cout << "LM-LBA: There are " << vToErase.size() << " observations whose will be deleted from the map" << endl;
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data
    //Keyframes
    vpNonEnoughOptKFs.clear();
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        cv::Mat Tiw = Converter::toCvMat(SE3quat);
        cv::Mat Tco_cn = pKFi->GetPose() * Tiw.inv();
        cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
        double dist = cv::norm(trasl);
        pKFi->SetPose(Converter::toCvMat(SE3quat));

        pKFi->mnNumberOfOpt += numPerform_it;
        //cout << "LM-LBA: KF " << pKFi->mnId << " had performed " <<  pKFi->mnNumberOfOpt << " iterations" << endl;
        if(pKFi->mnNumberOfOpt < 10)
        {
            vpNonEnoughOptKFs.push_back(pKFi);
        }
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    pCurrentMap->IncreaseChangeIndex();
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF)
{    
    //cout << "LBA" << endl;
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    Map* pCurrentMap = pKF->GetMap();

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    num_fixedKF = 0;
    list<MapPoint*> lLocalMapPoints;
    set<MapPoint*> sNumObsMP;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if(pKFi->mnId==pMap->GetInitKFid())
        {
            num_fixedKF = 1;
        }
        vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                {

                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
                }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId )
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
    num_fixedKF = lFixedCameras.size() + num_fixedKF;
    if(num_fixedKF < 2)
    {
        //Verbose::PrintMess("LM-LBA: New Fixed KFs had been set", Verbose::VERBOSITY_NORMAL);
        //TODO We set 2 KFs to fixed to avoid a degree of freedom in scale
        list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin();
        int lowerId = pKF->mnId;
        KeyFrame* pLowerKf;
        int secondLowerId = pKF->mnId;
        KeyFrame* pSecondLowerKF;

        for(; lit != lLocalKeyFrames.end(); lit++)
        {
            KeyFrame* pKFi = *lit;
            if(pKFi == pKF || pKFi->mnId == pMap->GetInitKFid())
            {
                continue;
            }

            if(pKFi->mnId < lowerId)
            {
                lowerId = pKFi->mnId;
                pLowerKf = pKFi;
            }
            else if(pKFi->mnId < secondLowerId)
            {
                secondLowerId = pKFi->mnId;
                pSecondLowerKF = pKFi;
            }
        }
        lFixedCameras.push_back(pLowerKf);
        lLocalKeyFrames.remove(pLowerKf);
        num_fixedKF++;
        if(num_fixedKF < 2)
        {
            lFixedCameras.push_back(pSecondLowerKF);
            lLocalKeyFrames.remove(pSecondLowerKF);
            num_fixedKF++;
        }
    }

    if(num_fixedKF == 0)
    {
        Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
        //return;
    }
    //Verbose::PrintMess("LM-LBA: There are " + to_string(lLocalKeyFrames.size()) + " KFs and " + to_string(lLocalMapPoints.size()) + " MPs to optimize. " + to_string(num_fixedKF) + " KFs are fixed", Verbose::VERBOSITY_DEBUG);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    if (pMap->IsInertial())
        solver->setUserLambdaInit(100.0); // TODO uncomment
    //cout << "LM-LBA: lambda init: " << solver->userLambdaInit() << endl;

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==pMap->GetInitKFid());
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }
    //Verbose::PrintMess("LM-LBA: KFs to optimize added", Verbose::VERBOSITY_DEBUG);

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    int nPoints = 0;

    int nKFs = lLocalKeyFrames.size()+lFixedCameras.size(), nEdges = 0;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        nPoints++;

        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                const int leftIndex = get<0>(mit->second);

                // Monocular observation
                if(leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)]<0)
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->pCamera = pKFi->mpCamera;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    nEdges++;
                }
                else if(leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)]>=0)// Stereo observation
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    nEdges++;
                }

                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->mTrl = Converter::toSE3Quat(pKFi->mTrl);

                        e->pCamera = pKFi->mpCamera2;

                        optimizer.addEdge(e);
                        vpEdgesBody.push_back(e);
                        vpEdgeKFBody.push_back(pKFi);
                        vpMapPointEdgeBody.push_back(pMP);

                        nEdges++;
                    }
                }
            }
        }
    }

    //Verbose::PrintMess("LM-LBA: total observations: " + to_string(vpMapPointEdgeMono.size()+vpMapPointEdgeStereo.size()), Verbose::VERBOSITY_DEBUG);

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    optimizer.optimize(5);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    //std::cout << "LBA time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    //std::cout << "Keyframes: " << nKFs << " --- MapPoints: " << nPoints << " --- Edges: " << nEdges << endl;

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

        // Check inlier observations
        int nMonoBadObs = 0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                // e->setLevel(1); // MODIFICATION
                nMonoBadObs++;
            }

            //e->setRobustKernel(0);
        }

        int nBodyBadObs = 0;
        for(size_t i=0, iend=vpEdgesBody.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
            MapPoint* pMP = vpMapPointEdgeBody[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                //e->setLevel(1);
                nBodyBadObs++;
            }

            //e->setRobustKernel(0);
        }

        int nStereoBadObs = 0;
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                //TODO e->setLevel(1);
                nStereoBadObs++;
            }

            //TODO e->setRobustKernel(0);
        }
        //Verbose::PrintMess("LM-LBA: First optimization has " + to_string(nMonoBadObs) + " monocular and " + to_string(nStereoBadObs) + " stereo bad observations", Verbose::VERBOSITY_DEBUG);

        // Optimize again without the outliers
        //Verbose::PrintMess("LM-LBA: second optimization", Verbose::VERBOSITY_DEBUG);
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesBody.size()+vpEdgesStereo.size());

    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesBody.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
        MapPoint* pMP = vpMapPointEdgeBody[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFBody[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    //Verbose::PrintMess("LM-LBA: outlier observations: " + to_string(vToErase.size()), Verbose::VERBOSITY_DEBUG);
    bool bRedrawError = false;
    if(vToErase.size() >= (vpMapPointEdgeMono.size()+vpMapPointEdgeStereo.size()) * 0.5)
    {
        Verbose::PrintMess("LM-LBA: ERROR IN THE OPTIMIZATION, MOST OF THE POINTS HAS BECOME OUTLIERS", Verbose::VERBOSITY_NORMAL);

        return;
        bRedrawError = true;
        string folder_name = "test_LBA";
        string name = "_PreLM_LBA";
        name = "_PreLM_LBA_Fixed";

    }


    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        map<KeyFrame*, int> mspInitialConnectedKFs;
        map<KeyFrame*, int> mspInitialObservationKFs;
        if(bRedrawError)
        {
            for(KeyFrame* pKFi : lLocalKeyFrames)
            {

                mspInitialConnectedKFs[pKFi] = pKFi->GetConnectedKeyFrames().size();
                mspInitialObservationKFs[pKFi] = pKFi->GetNumberMPs();
            }
        }

        //cout << "LM-LBA: There are " << vToErase.size() << " observations whose will be deleted from the map" << endl;
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }

        map<KeyFrame*, int> mspFinalConnectedKFs;
        map<KeyFrame*, int> mspFinalObservationKFs;
        if(bRedrawError)
        {
            ofstream f_lba;
            f_lba.open("test_LBA/LBA_failure_KF" + to_string(pKF->mnId) +".txt");
            f_lba << "# KF id, Initial Num CovKFs, Final Num CovKFs, Initial Num MPs, Fimal Num MPs" << endl;
            f_lba << fixed;

            for(KeyFrame* pKFi : lLocalKeyFrames)
            {
                pKFi->UpdateConnections();
                int finalNumberCovKFs = pKFi->GetConnectedKeyFrames().size();
                int finalNumberMPs = pKFi->GetNumberMPs();
                f_lba << pKFi->mnId << ", " << mspInitialConnectedKFs[pKFi] << ", " << finalNumberCovKFs << ", " << mspInitialObservationKFs[pKFi] << ", " << finalNumberMPs << endl;

                mspFinalConnectedKFs[pKFi] = finalNumberCovKFs;
                mspFinalObservationKFs[pKFi] = finalNumberMPs;
            }

            f_lba.close();
        }
    }

    // Recover optimized data
    //Keyframes
    bool bShowStats = false;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        cv::Mat Tiw = Converter::toCvMat(SE3quat);
        cv::Mat Tco_cn = pKFi->GetPose() * Tiw.inv();
        cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
        double dist = cv::norm(trasl);
        pKFi->SetPose(Converter::toCvMat(SE3quat));

        if(dist > 1.0)
        {
            bShowStats = true;
            Verbose::PrintMess("LM-LBA: Too much distance in KF " + to_string(pKFi->mnId) +  ", " + to_string(dist) +  " meters. Current KF " + to_string(pKF->mnId), Verbose::VERBOSITY_DEBUG);
            Verbose::PrintMess("LM-LBA: Number of connections between the KFs " + to_string(pKF->GetWeight((pKFi))), Verbose::VERBOSITY_DEBUG);

            int numMonoMP = 0, numBadMonoMP = 0;
            int numStereoMP = 0, numBadStereoMP = 0;
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
            {
                if(vpEdgeKFMono[i] != pKFi)
                    continue;
                ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
                MapPoint* pMP = vpMapPointEdgeMono[i];

                if(pMP->isBad())
                    continue;

                if(e->chi2()>5.991 || !e->isDepthPositive())
                {
                    numBadMonoMP++;
                }
                else
                {
                    numMonoMP++;
                }
            }

            for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
            {
                if(vpEdgeKFStereo[i] != pKFi)
                    continue;
                g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
                MapPoint* pMP = vpMapPointEdgeStereo[i];

                if(pMP->isBad())
                    continue;

                if(e->chi2()>7.815 || !e->isDepthPositive())
                {
                    numBadStereoMP++;
                }
                else
                {
                    numStereoMP++;
                }
            }
            Verbose::PrintMess("LM-LBA: Good observations in mono " + to_string(numMonoMP) +  " and stereo " + to_string(numStereoMP), Verbose::VERBOSITY_DEBUG);
            Verbose::PrintMess("LM-LBA: Bad observations in mono " + to_string(numBadMonoMP) +  " and stereo " + to_string(numBadStereoMP), Verbose::VERBOSITY_DEBUG);
        }
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    if(bRedrawError)
    {
        string folder_name = "test_LBA";
        string name = "_PostLM_LBA";
        //pMap->printReprojectionError(lLocalKeyFrames, pKF, name, folder_name);
        name = "_PostLM_LBA_Fixed";
        //pMap->printReprojectionError(lFixedCameras, pKF, name, folder_name);
    }

    // TODO Check this changeindex
    pMap->IncreaseChangeIndex();
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag)
{
    bool bShowImages = false;

    vector<MapPoint*> vpMPs;

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;
    set<KeyFrame*> spKeyFrameBA;

    Map* pCurrentMap = pMainKF->GetMap();

    //set<MapPoint*> sNumObsMP;

    // Set fixed KeyFrame vertices
    for(KeyFrame* pKFi : vpFixedKF)
    {
        if(pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map", Verbose::VERBOSITY_NORMAL);
            continue;
        }

        pKFi->mnBALocalForMerge = pMainKF->mnId;

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;

        set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
        for(MapPoint* pMPi : spViewMPs)
        {
            if(pMPi)
                if(!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)

                    if(pMPi->mnBALocalForMerge!=pMainKF->mnId)
                    {
                        vpMPs.push_back(pMPi);
                        pMPi->mnBALocalForMerge=pMainKF->mnId;
                    }
                    /*if(sNumObsMP.find(pMPi) == sNumObsMP.end())
                    {
                        sNumObsMP.insert(pMPi);
                    }
                    else
                    {
                        if(pMPi->mnBALocalForMerge!=pMainKF->mnId)
                        {
                            vpMPs.push_back(pMPi);
                            pMPi->mnBALocalForMerge=pMainKF->mnId;
                        }
                    }*/
        }

        spKeyFrameBA.insert(pKFi);
    }

    //cout << "End to load Fixed KFs" << endl;

    // Set non fixed Keyframe vertices
    set<KeyFrame*> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
    for(KeyFrame* pKFi : vpAdjustKF)
    {
        if(pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
            continue;

        pKFi->mnBALocalForKF = pMainKF->mnId;

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;

        set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
        for(MapPoint* pMPi : spViewMPs)
        {
            if(pMPi)
            {
                if(!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)
                {
                    /*if(sNumObsMP.find(pMPi) == sNumObsMP.end())
                    {
                        sNumObsMP.insert(pMPi);
                    }*/
                    if(pMPi->mnBALocalForMerge != pMainKF->mnId)
                    {
                        vpMPs.push_back(pMPi);
                        pMPi->mnBALocalForMerge = pMainKF->mnId;
                    }
                }
            }
        }

        spKeyFrameBA.insert(pKFi);
    }

    //Verbose::PrintMess("LBA: There are " + to_string(vpMPs.size()) + " MPs to optimize", Verbose::VERBOSITY_NORMAL);

    //cout << "End to load KFs for position adjust" << endl;

    const int nExpectedSize = (vpAdjustKF.size()+vpFixedKF.size())*vpMPs.size();

    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    map<KeyFrame*, int> mpObsKFs;
    map<KeyFrame*, int> mpObsFinalKFs;
    map<MapPoint*, int> mpObsMPs;
    for(unsigned int i=0; i < vpMPs.size(); ++i)
    {
        MapPoint* pMPi = vpMPs[i];
        if(pMPi->isBad())
            continue;

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMPi->GetWorldPos()));
        const int id = pMPi->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);


        const map<KeyFrame*,tuple<int,int>> observations = pMPi->GetObservations();
        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            //cout << "--KF view init" << endl;

            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid || pKF->mnBALocalForMerge != pMainKF->mnId || !pKF->GetMapPoint(get<0>(mit->second)))
                continue;

            //cout << "-- KF view exists" << endl;
            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[get<0>(mit->second)];
            //cout << "-- KeyPoint loads" << endl;

            if(pKF->mvuRight[get<0>(mit->second)]<0) //Monocular
            {
                mpObsMPs[pMPi]++;
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();


                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
                //cout << "-- Sigma loads" << endl;

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                e->pCamera = pKF->mpCamera;
                //cout << "-- Calibration loads" << endl;

                optimizer.addEdge(e);
                //cout << "-- Edge added" << endl;

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMPi);
                //cout << "-- Added to vector" << endl;

                mpObsKFs[pKF]++;
            }
            else // RGBD or Stereo
            {
                mpObsMPs[pMPi]+=2;
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber3D);

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMPi);

                mpObsKFs[pKF]++;
            }
            //cout << "-- End to load point" << endl;
        }
    }
    //Verbose::PrintMess("LBA: number total of edged -> " + to_string(vpEdgeKFMono.size() + vpEdgeKFStereo.size()), Verbose::VERBOSITY_NORMAL);

    map<int, int> mStatsObs;
    for(map<MapPoint*, int>::iterator it = mpObsMPs.begin(); it != mpObsMPs.end(); ++it)
    {
        MapPoint* pMPi = it->first;
        int numObs = it->second;

        mStatsObs[numObs]++;
        /*if(numObs < 5)
        {
            cout << "LBA: MP " << pMPi->mnId << " has " << numObs << " observations" << endl;
        }*/
    }

    /*for(map<int, int>::iterator it = mStatsObs.begin(); it != mStatsObs.end(); ++it)
    {
        cout << "LBA: There are " << it->second << " MPs with " << it->first << " observations" << endl;
    }*/

    //cout << "End to load MPs" << endl;

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    //cout << "End the first optimization" << endl;

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    map<unsigned long int, int> mWrongObsKF;
    if(bDoMore)
    {

        // Check inlier observations
        int badMonoMP = 0, badStereoMP = 0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badMonoMP++;
            }

            e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badStereoMP++;
            }

            e->setRobustKernel(0);
        }
        Verbose::PrintMess("LBA: First optimization, there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP) + " sterero bad edges", Verbose::VERBOSITY_DEBUG);

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    //cout << "End the second optimization (without outliers)" << endl;
    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());
    set<MapPoint*> spErasedMPs;
    set<KeyFrame*> spErasedKFs;

    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
            mWrongObsKF[pKFi->mnId]++;
            badMonoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
            mWrongObsKF[pKFi->mnId]++;
            badStereoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }
    Verbose::PrintMess("LBA: Second optimization, there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP) + " sterero bad edges", Verbose::VERBOSITY_DEBUG);

    // Get Map Mutex
    unique_lock<mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        map<KeyFrame*, int> mpMPs_in_KF;
        for(KeyFrame* pKFi : spErasedKFs)
        {
            int num_MPs = pKFi->GetMapPoints().size();
            mpMPs_in_KF[pKFi] = num_MPs;
        }

        Verbose::PrintMess("LBA: There are " + to_string(vToErase.size()) + " observations whose will be deleted from the map", Verbose::VERBOSITY_DEBUG);
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }

        Verbose::PrintMess("LBA: " + to_string(spErasedMPs.size()) + " MPs had deleted observations", Verbose::VERBOSITY_DEBUG);
        Verbose::PrintMess("LBA: Current map is " + to_string(pMainKF->GetMap()->GetId()), Verbose::VERBOSITY_DEBUG);
        int numErasedMP = 0;
        for(MapPoint* pMPi : spErasedMPs)
        {
            if(pMPi->isBad())
            {
                Verbose::PrintMess("LBA: MP " + to_string(pMPi->mnId) + " has lost almost all the observations, its origin map is " + to_string(pMPi->mnOriginMapId), Verbose::VERBOSITY_DEBUG);
                numErasedMP++;
            }
        }
        Verbose::PrintMess("LBA: " + to_string(numErasedMP) + " MPs had deleted from the map", Verbose::VERBOSITY_DEBUG);

        for(KeyFrame* pKFi : spErasedKFs)
        {
            int num_MPs = pKFi->GetMapPoints().size();
            int num_init_MPs = mpMPs_in_KF[pKFi];
            Verbose::PrintMess("LBA: Initially KF " + to_string(pKFi->mnId) + " had " + to_string(num_init_MPs) + ", at the end has " + to_string(num_MPs), Verbose::VERBOSITY_DEBUG);
        }
    }
    for(unsigned int i=0; i < vpMPs.size(); ++i)
    {
        MapPoint* pMPi = vpMPs[i];
        if(pMPi->isBad())
            continue;

        const map<KeyFrame*,tuple<int,int>> observations = pMPi->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            //cout << "--KF view init" << endl;

            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid || pKF->mnBALocalForKF != pMainKF->mnId || !pKF->GetMapPoint(get<0>(mit->second)))
                continue;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[get<0>(mit->second)];
            //cout << "-- KeyPoint loads" << endl;

            if(pKF->mvuRight[get<0>(mit->second)]<0) //Monocular
            {
                mpObsFinalKFs[pKF]++;
            }
            else // RGBD or Stereo
            {

                mpObsFinalKFs[pKF]++;
            }
            //cout << "-- End to load point" << endl;
        }
    }

    //cout << "End to erase observations" << endl;

    // Recover optimized data

    //Keyframes
    for(KeyFrame* pKFi : vpAdjustKF)
    {
        if(pKFi->isBad())
            continue;

        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        cv::Mat Tiw = Converter::toCvMat(SE3quat);
        cv::Mat Tco_cn = pKFi->GetPose() * Tiw.inv();
        cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
        double dist = cv::norm(trasl);

        int numMonoBadPoints = 0, numMonoOptPoints = 0;
        int numStereoBadPoints = 0, numStereoOptPoints = 0;
        vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;
        vector<MapPoint*> vpMonoMPsBad, vpStereoMPsBad;

        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];
            KeyFrame* pKFedge = vpEdgeKFMono[i];

            if(pKFi != pKFedge)
            {
                continue;
            }

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                numMonoBadPoints++;
                vpMonoMPsBad.push_back(pMP);

            }
            else
            {
                numMonoOptPoints++;
                vpMonoMPsOpt.push_back(pMP);
            }

        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];
            KeyFrame* pKFedge = vpEdgeKFMono[i];

            if(pKFi != pKFedge)
            {
                continue;
            }

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                numStereoBadPoints++;
                vpStereoMPsBad.push_back(pMP);
            }
            else
            {
                numStereoOptPoints++;
                vpStereoMPsOpt.push_back(pMP);
            }
        }


        if(numMonoOptPoints + numStereoOptPoints < 50)
        {
            Verbose::PrintMess("LBA ERROR: KF " + to_string(pKFi->mnId) + " has only " + to_string(numMonoOptPoints) + " monocular and " + to_string(numStereoOptPoints) + " stereo points", Verbose::VERBOSITY_DEBUG);
        }
        if(dist > 1.0)
        {
            if(bShowImages)
            {
                string strNameFile = pKFi->mNameFile;
                // #ifdef OPENCV4
                cv::Mat imLeft = cv::imread(strNameFile, cv::IMREAD_UNCHANGED);
                cv::cvtColor(imLeft, imLeft, cv::COLOR_GRAY2BGR);
                // #else
                // cv::Mat imLeft = cv::imread(strNameFile, CV_LOAD_IMAGE_UNCHANGED);
                // cv::cvtColor(imLeft, imLeft, CV_GRAY2BGR);
                // #endif

                int numPointsMono = 0, numPointsStereo = 0;
                int numPointsMonoBad = 0, numPointsStereoBad = 0;
                for(int i=0; i<vpMonoMPsOpt.size(); ++i)
                {
                    if(!vpMonoMPsOpt[i] || vpMonoMPsOpt[i]->isBad())
                    {
                        continue;
                    }
                    int index = get<0>(vpMonoMPsOpt[i]->GetIndexInKeyFrame(pKFi));
                    if(index < 0)
                    {
                        //cout << "LBA ERROR: KF has a monocular observation which is not recognized by the MP" << endl;
                        //cout << "LBA: KF " << pKFi->mnId << " and MP " << vpMonoMPsOpt[i]->mnId << " with index " << endl;
                        continue;
                    }

                    //string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                    cv::circle(imLeft, pKFi->mvKeys[index].pt, 2, cv::Scalar(255, 0, 0));
                    //cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                    numPointsMono++;
                 }

                 for(int i=0; i<vpStereoMPsOpt.size(); ++i)
                 {
                     if(!vpStereoMPsOpt[i] || vpStereoMPsOpt[i]->isBad())
                     {
                         continue;
                     }
                     int index = get<0>(vpStereoMPsOpt[i]->GetIndexInKeyFrame(pKFi));
                     if(index < 0)
                     {
                         //cout << "LBA: KF has a stereo observation which is not recognized by the MP" << endl;
                         //cout << "LBA: KF " << pKFi->mnId << " and MP " << vpStereoMPsOpt[i]->mnId << endl;
                         continue;
                      }

                      //string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                      cv::circle(imLeft, pKFi->mvKeys[index].pt, 2, cv::Scalar(0, 255, 0));
                      //cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                      numPointsStereo++;
                 }

                 for(int i=0; i<vpMonoMPsBad.size(); ++i)
                 {
                     if(!vpMonoMPsBad[i] || vpMonoMPsBad[i]->isBad())
                     {
                         continue;
                     }
                     int index = get<0>(vpMonoMPsBad[i]->GetIndexInKeyFrame(pKFi));
                     if(index < 0)
                     {
                         //cout << "LBA ERROR: KF has a monocular observation which is not recognized by the MP" << endl;
                         //cout << "LBA: KF " << pKFi->mnId << " and MP " << vpMonoMPsOpt[i]->mnId << " with index " << endl;
                         continue;
                     }

                    //string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                    cv::circle(imLeft, pKFi->mvKeys[index].pt, 2, cv::Scalar(0, 0, 255));
                    //cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                    numPointsMonoBad++;
                }
                for(int i=0; i<vpStereoMPsBad.size(); ++i)
                {
                    if(!vpStereoMPsBad[i] || vpStereoMPsBad[i]->isBad())
                    {
                        continue;
                    }
                    int index = get<0>(vpStereoMPsBad[i]->GetIndexInKeyFrame(pKFi));
                    if(index < 0)
                    {
                        //cout << "LBA: KF has a stereo observation which is not recognized by the MP" << endl;
                        //cout << "LBA: KF " << pKFi->mnId << " and MP " << vpStereoMPsOpt[i]->mnId << endl;
                        continue;
                    }

                    //string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                    cv::circle(imLeft, pKFi->mvKeys[index].pt, 2, cv::Scalar(0, 0, 255));
                    //cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                    numPointsStereoBad++;
                }

                string namefile = "./test_LBA/LBA_KF" + to_string(pKFi->mnId) + "_" + to_string(numPointsMono + numPointsStereo) +"_D" + to_string(dist) +".png";
                cv::imwrite(namefile, imLeft);

                Verbose::PrintMess("--LBA in KF " + to_string(pKFi->mnId), Verbose::VERBOSITY_DEBUG);
                Verbose::PrintMess("--Distance: " + to_string(dist) + " meters", Verbose::VERBOSITY_DEBUG);
                Verbose::PrintMess("--Number of observations: " + to_string(numMonoOptPoints) + " in mono and " + to_string(numStereoOptPoints) + " in stereo", Verbose::VERBOSITY_DEBUG);
                Verbose::PrintMess("--Number of discarded observations: " + to_string(numMonoBadPoints) + " in mono and " + to_string(numStereoBadPoints) + " in stereo", Verbose::VERBOSITY_DEBUG);
                Verbose::PrintMess("--To much distance correction in LBA: It has " + to_string(mpObsKFs[pKFi]) + " observated MPs", Verbose::VERBOSITY_DEBUG);
                Verbose::PrintMess("--To much distance correction in LBA: It has " + to_string(mpObsFinalKFs[pKFi]) + " deleted observations", Verbose::VERBOSITY_DEBUG);
                Verbose::PrintMess("--------", Verbose::VERBOSITY_DEBUG);
            }
        }
        pKFi->SetPose(Tiw);

    }
    //cout << "End to update the KeyFrames" << endl;

    //Points
    for(MapPoint* pMPi : vpMPs)
    {
        if(pMPi->isBad())
            continue;

        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMPi->mnId+maxKFid+1));
        pMPi->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMPi->UpdateNormalAndDepth();

    }
    //cout << "End to update MapPoint" << endl;
}


} // namespace ORB_SLAM2
