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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"

#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace ORB_SLAM2
{

    class LoopClosing;

    class Optimizer
    {
     public:
        void BundleAdjustment(const std::vector<KeyFrame *> &vpKF,
                              const std::vector<MapPoint *> &vpMP, int nIterations = 5,
                              bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                              const bool bRobust = true);
        static void GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5,
                                    bool *pbStopFlag = NULL,
                                    const unsigned long nLoopKF = 0,
                                    const bool bRobust = true);
        void static FullInertialBA(Map *pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);

        void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, vector<KeyFrame*> &vpNonEnoughOptKFs);
        void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF);
        // Local BA in welding area when two maps are merged
        void static LocalBundleAdjustment(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag);


        void LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap);

        int poseOptimization(Frame *pFrame);

        // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
        void static OptimizeEssentialGraph(
            Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
            const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
            const LoopClosing::KeyFrameAndPose &CorrectedSim3,
            const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
            const bool &bFixScale);

        // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
        static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2,
                         std::vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12,
                         const float th2, const bool bFixScale);
                         
         // inertial systems
        void static LocalInertialBA(KeyFrame* pKF, bool *pbStopFlag, Map *pMap,
                bool bLarge = false, bool bRecInit = false);
                
        // Inertial pose-graph
        void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel=false, bool bGauss=false, float priorG = 1e2, float priorA = 1e6);
        void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
        void static InertialOptimization(vector<KeyFrame*> vpKFs, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
        void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);


    }; // namespace Optimizer

} // namespace ORB_SLAM2

#endif // OPTIMIZER_H
