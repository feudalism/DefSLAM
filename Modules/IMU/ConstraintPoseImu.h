#ifndef CONSTRAINT_POSE_IMU_H
#define CONSTRAINT_POSE_IMU_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "ImuTypes.h"
#include "Converter.h"

namespace ORB_SLAM3
{

using ORB_SLAM2::Converter;
    
typedef Eigen::Matrix<double, 15, 15> Matrix15d;
    
class ConstraintPoseImu
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintPoseImu(const Eigen::Matrix3d &Rwb_, const Eigen::Vector3d &twb_, const Eigen::Vector3d &vwb_,
                       const Eigen::Vector3d &bg_, const Eigen::Vector3d &ba_, const Matrix15d &H_):
                       Rwb(Rwb_), twb(twb_), vwb(vwb_), bg(bg_), ba(ba_), H(H_)
    {
        H = (H+H)/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,15,15> > es(H);
        Eigen::Matrix<double,15,1> eigs = es.eigenvalues();
        for(int i=0;i<15;i++)
            if(eigs[i]<1e-12)
                eigs[i]=0;
        H = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    }
    
    ConstraintPoseImu(const cv::Mat &Rwb_, const cv::Mat &twb_, const cv::Mat &vwb_,
                       const defSLAM::IMU::Bias &b, const cv::Mat &H_)
    {
        Rwb = Converter::toMatrix3d(Rwb_);
        twb = Converter::toVector3d(twb_);
        vwb = Converter::toVector3d(vwb_);
        bg << b.bwx, b.bwy, b.bwz;
        ba << b.bax, b.bay, b.baz;
        for(int i=0;i<15;i++)
            for(int j=0;j<15;j++)
                H(i,j)=H_.at<float>(i,j);
        H = (H+H)/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,15,15> > es(H);
        Eigen::Matrix<double,15,1> eigs = es.eigenvalues();
        for(int i=0;i<15;i++)
            if(eigs[i]<1e-12)
                eigs[i]=0;
        H = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    }

    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d vwb;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
    Matrix15d H;
};

} // namespace ORB_SLAM3

#endif // CONSTRAINT_POSE_IMU_H