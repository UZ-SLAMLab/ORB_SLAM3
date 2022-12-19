/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"

#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <Frame.h>
#include <KeyFrame.h>

#include "Converter.h"
#include <math.h>

namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class GeometricCamera;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 15, 1> Vector15d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 15, 15> Matrix15d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w);

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R);

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v);
Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v);
Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z);

Eigen::Matrix3d Skew(const Eigen::Vector3d &w);
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z);

template <typename T = double>
Eigen::Matrix<T, 3, 3> NormalizeRotation(const Eigen::Matrix<T, 3, 3> &R)
{
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

// 相关节点中使用，存放的是imu与cam的内外参
class ImuCamPose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuCamPose() {}
    ImuCamPose(KeyFrame *pKF);
    ImuCamPose(Frame *pF);
    ImuCamPose(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame *pKF);

    void SetParam(const std::vector<Eigen::Matrix3d> &_Rcw, const std::vector<Eigen::Vector3d> &_tcw, const std::vector<Eigen::Matrix3d> &_Rbc,
                    const std::vector<Eigen::Vector3d> &_tbc, const double &_bf);

    void Update(const double *pu);                                                   // update in the imu reference
    void UpdateW(const double *pu);                                                  // update in the world reference
    Eigen::Vector2d Project(const Eigen::Vector3d &Xw, int cam_idx = 0) const;       // Mono
    Eigen::Vector3d ProjectStereo(const Eigen::Vector3d &Xw, int cam_idx = 0) const; // Stereo
    bool isDepthPositive(const Eigen::Vector3d &Xw, int cam_idx = 0) const;

public:
    // For IMU
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;

    // For set of cameras
    std::vector<Eigen::Matrix3d> Rcw;
    std::vector<Eigen::Vector3d> tcw;
    std::vector<Eigen::Matrix3d> Rcb, Rbc;
    std::vector<Eigen::Vector3d> tcb, tbc;
    double bf;
    std::vector<GeometricCamera *> pCamera;

    // For posegraph 4DoF
    Eigen::Matrix3d Rwb0;
    Eigen::Matrix3d DR;

    int its; // 记录更新次数
};

// 逆深度点，后面节点虽然有用到，但是声明的节点并没有使用到，暂时不看
class InvDepthPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    InvDepthPoint() {}
    InvDepthPoint(double _rho, double _u, double _v, KeyFrame *pHostKF);

    void Update(const double *pu);

    double rho;
    double u, v; // they are not variables, observation in the host frame

    double fx, fy, cx, cy, bf; // from host frame

    int its;
};

// Optimizable parameters are IMU pose
// 优化中关于位姿的节点，6自由度
class VertexPose : public g2o::BaseVertex<6, ImuCamPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() {}
    VertexPose(KeyFrame *pKF)
    {
        setEstimate(ImuCamPose(pKF));
    }
    VertexPose(Frame *pF)
    {
        setEstimate(ImuCamPose(pF));
    }

    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    // 重置函数,设定被优化变量的原始值
    virtual void setToOriginImpl()
    {
    }

    virtual void oplusImpl(const double *update_)
    {
        // https://github.com/RainerKuemmerle/g2o/blob/master/doc/README_IF_IT_WAS_WORKING_AND_IT_DOES_NOT.txt
        // 官方讲解cache
        // 需要在oplusImpl与setEstimate函数中添加
        _estimate.Update(update_);
        updateCache();
    }
};

// 优化中关于位姿的节点，4自由度  3个平移加一个航偏
class VertexPose4DoF : public g2o::BaseVertex<4, ImuCamPose>
{
    // Translation and yaw are the only optimizable variables
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose4DoF() {}
    VertexPose4DoF(KeyFrame *pKF)
    {
        setEstimate(ImuCamPose(pKF));
    }
    VertexPose4DoF(Frame *pF)
    {
        setEstimate(ImuCamPose(pF));
    }
    VertexPose4DoF(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame *pKF)
    {

        setEstimate(ImuCamPose(_Rwc, _twc, pKF));
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
    }

    // 强制让旋转的前两维的更新量为0
    virtual void oplusImpl(const double *update_)
    {
        double update6DoF[6];
        update6DoF[0] = 0;
        update6DoF[1] = 0;
        update6DoF[2] = update_[0];
        update6DoF[3] = update_[1];
        update6DoF[4] = update_[2];
        update6DoF[5] = update_[3];
        _estimate.UpdateW(update6DoF);
        updateCache();
    }
};

/** 
 * @brief 速度节点
 */
class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity() {}
    VertexVelocity(KeyFrame *pKF);
    VertexVelocity(Frame *pF);

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
    }

    virtual void oplusImpl(const double *update_)
    {
        Eigen::Vector3d uv;
        uv << update_[0], update_[1], update_[2];
        setEstimate(estimate() + uv);
    }
};

/** 
 * @brief 陀螺仪偏置节点
 */
class VertexGyroBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyroBias() {}
    VertexGyroBias(KeyFrame *pKF);
    VertexGyroBias(Frame *pF);

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
    }

    virtual void oplusImpl(const double *update_)
    {
        Eigen::Vector3d ubg;
        ubg << update_[0], update_[1], update_[2];
        setEstimate(estimate() + ubg);
    }
};

/** 
 * @brief 加速度计偏置节点
 */
class VertexAccBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAccBias() {}
    VertexAccBias(KeyFrame *pKF);
    VertexAccBias(Frame *pF);

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
    }

    virtual void oplusImpl(const double *update_)
    {
        Eigen::Vector3d uba;
        uba << update_[0], update_[1], update_[2];
        setEstimate(estimate() + uba);
    }
};

// Gravity direction vertex
// 重力方向
class GDirection
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GDirection() {}
    GDirection(Eigen::Matrix3d pRwg) : Rwg(pRwg) {}

    void Update(const double *pu)
    {
        // 强行优化不可观的数据，会导致不收敛
        Rwg = Rwg * ExpSO3(pu[0], pu[1], 0.0);
    }

    Eigen::Matrix3d Rwg, Rgw;

    int its;
};

/** 
 * @brief 重力方向节点
 */
class VertexGDir : public g2o::BaseVertex<2, GDirection>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGDir() {}
    VertexGDir(Eigen::Matrix3d pRwg)
    {
        setEstimate(GDirection(pRwg));
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
    }

    virtual void oplusImpl(const double *update_)
    {
        _estimate.Update(update_);
        updateCache();
    }
};

// scale vertex
/** 
 * @brief 尺度节点
 */
class VertexScale : public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexScale()
    {
        setEstimate(1.0);
    }
    VertexScale(double ps)
    {
        setEstimate(ps);
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
        setEstimate(1.0);
    }

    virtual void oplusImpl(const double *update_)
    {
        setEstimate(estimate() * exp(*update_));
    }
};

// Inverse depth point (just one parameter, inverse depth at the host frame)
/** 
 * @brief 没用
 */
class VertexInvDepth : public g2o::BaseVertex<1, InvDepthPoint>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexInvDepth() {}
    VertexInvDepth(double invDepth, double u, double v, KeyFrame *pHostKF)
    {
        setEstimate(InvDepthPoint(invDepth, u, v, pHostKF));
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    virtual void setToOriginImpl()
    {
    }

    virtual void oplusImpl(const double *update_)
    {
        _estimate.Update(update_);
        updateCache();
    }
};

/** 
 * @brief 单目视觉重投影的边
 * 这里或许会有疑问，OptimizableTypes.h 里面不是定义了视觉重投影的边么？
 * 原因是这样，那里面定义的边也用，不过只是在纯视觉时，没有imu情况下，因为用已经定义好的节点就好
 * 但是加入imu时，优化要有残差的边与重投影的边同时存在，且两个边可能连接同一个位姿节点，所以需要重新弄一个包含imu位姿的节点
 * 因此，边也需要重新写，并且在imu优化时使用这个边
 */
// 误差为2维， 类型为Eigen::Vector2d， 节点1类型为g2o::VertexSBAPointXYZ，节点二类型为VertexPose
class EdgeMono : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMono(int cam_idx_ = 0) : cam_idx(cam_idx_)
    {
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    // 计算重投影误差
    void computeError()
    {
        const g2o::VertexSBAPointXYZ *VPoint = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[1]);
        const Eigen::Vector2d obs(_measurement);
        _error = obs - VPose->estimate().Project(VPoint->estimate(), cam_idx);
    }

    virtual void linearizeOplus();

    bool isDepthPositive()
    {
        const g2o::VertexSBAPointXYZ *VPoint = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[1]);
        return VPose->estimate().isDepthPositive(VPoint->estimate(), cam_idx);
    }

    Eigen::Matrix<double, 2, 9> GetJacobian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 2, 9> J;
        J.block<2, 3>(0, 0) = _jacobianOplusXi;
        J.block<2, 6>(0, 3) = _jacobianOplusXj;
        return J;
    }

    // 由2*2的像素点信息矩阵变成了9*9的关于旋转平移与三维点坐标的信息矩阵
    // 可以理解为像素的不确定性给旋转平移跟三维点带来了多大的不确定性
    Eigen::Matrix<double, 9, 9> GetHessian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 2, 9> J;
        J.block<2, 3>(0, 0) = _jacobianOplusXi;
        J.block<2, 6>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }

public:
    const int cam_idx;
};

/** 
 * @brief 单目纯位姿一元边
 */
class EdgeMonoOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoOnlyPose(const Eigen::Vector3f &Xw_, int cam_idx_ = 0) : Xw(Xw_.cast<double>()),
                                                                        cam_idx(cam_idx_) {}

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);
        const Eigen::Vector2d obs(_measurement);
        _error = obs - VPose->estimate().Project(Xw, cam_idx);
    }

    virtual void linearizeOplus();

    bool isDepthPositive()
    {
        const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);
        return VPose->estimate().isDepthPositive(Xw, cam_idx);
    }

    Eigen::Matrix<double, 6, 6> GetHessian()
    {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

public:
    const Eigen::Vector3d Xw;
    const int cam_idx;
};

/** 
 * @brief 双目位姿三维点二元边
 */
class EdgeStereo : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 代码中都是0
    EdgeStereo(int cam_idx_ = 0) : cam_idx(cam_idx_) {}

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const g2o::VertexSBAPointXYZ *VPoint = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[1]);
        const Eigen::Vector3d obs(_measurement);
        _error = obs - VPose->estimate().ProjectStereo(VPoint->estimate(), cam_idx);
    }

    virtual void linearizeOplus();

    Eigen::Matrix<double, 3, 9> GetJacobian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 9> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 6>(0, 3) = _jacobianOplusXj;
        return J;
    }

    Eigen::Matrix<double, 9, 9> GetHessian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 9> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 6>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }

public:
    const int cam_idx;
};

/** 
 * @brief 双目纯位姿一元边
 */
class EdgeStereoOnlyPose : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereoOnlyPose(const Eigen::Vector3f &Xw_, int cam_idx_ = 0) : Xw(Xw_.cast<double>()), cam_idx(cam_idx_) {}

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);
        const Eigen::Vector3d obs(_measurement);
        _error = obs - VPose->estimate().ProjectStereo(Xw, cam_idx);
    }

    virtual void linearizeOplus();

    Eigen::Matrix<double, 6, 6> GetHessian()
    {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

public:
    const Eigen::Vector3d Xw; // 3D point coordinates
    const int cam_idx;
};

/** 
 * @brief 惯性边（误差为残差）
 */
class EdgeInertial : public g2o::BaseMultiEdge<9, Vector9d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeInertial(IMU::Preintegrated *pInt);

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError();
    virtual void linearizeOplus();

    // 关于pose1与2 的旋转平移速度，以及之间的偏置的信息矩阵
    Eigen::Matrix<double, 24, 24> GetHessian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[0];
        J.block<9, 3>(0, 6) = _jacobianOplus[1];
        J.block<9, 3>(0, 9) = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

    // 没用
    Eigen::Matrix<double, 18, 18> GetHessianNoPose1()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 18> J;
        J.block<9, 3>(0, 0) = _jacobianOplus[1];
        J.block<9, 3>(0, 3) = _jacobianOplus[2];
        J.block<9, 3>(0, 6) = _jacobianOplus[3];
        J.block<9, 6>(0, 9) = _jacobianOplus[4];
        J.block<9, 3>(0, 15) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

    // 关于pose2 的旋转平移信息矩阵
    Eigen::Matrix<double, 9, 9> GetHessian2()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 9> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[4];
        J.block<9, 3>(0, 6) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

    // 预积分中对应的状态对偏置的雅可比
    const Eigen::Matrix3d JRg, JVg, JPg;
    const Eigen::Matrix3d JVa, JPa;

    IMU::Preintegrated *mpInt;  // 预积分
    const double dt;  // 预积分时间
    Eigen::Vector3d g;  // 0, 0, -IMU::GRAVITY_VALUE
};

// Edge inertial whre gravity is included as optimizable variable and it is not supposed to be pointing in -z axis, as well as scale
/** 
 * @brief 初始化惯性边（误差为残差）
 */
class EdgeInertialGS : public g2o::BaseMultiEdge<9, Vector9d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // EdgeInertialGS(IMU::Preintegrated* pInt);
    EdgeInertialGS(IMU::Preintegrated *pInt);

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError();
    virtual void linearizeOplus();

    const Eigen::Matrix3d JRg, JVg, JPg;
    const Eigen::Matrix3d JVa, JPa;
    IMU::Preintegrated *mpInt;
    const double dt;
    Eigen::Vector3d g, gI;

    // 关于pose1与2 的旋转平移速度，以及之间的偏置，还有重力方向与尺度的信息矩阵
    Eigen::Matrix<double, 27, 27> GetHessian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 27> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[0];
        J.block<9, 3>(0, 6) = _jacobianOplus[1];
        J.block<9, 3>(0, 9) = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        J.block<9, 2>(0, 24) = _jacobianOplus[6];
        J.block<9, 1>(0, 26) = _jacobianOplus[7];
        return J.transpose() * information() * J;
    }

    // 与上面摆放不同
    Eigen::Matrix<double, 27, 27> GetHessian2()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 27> J;
        J.block<9, 3>(0, 0) = _jacobianOplus[2];
        J.block<9, 3>(0, 3) = _jacobianOplus[3];
        J.block<9, 2>(0, 6) = _jacobianOplus[6];
        J.block<9, 1>(0, 8) = _jacobianOplus[7];
        J.block<9, 3>(0, 9) = _jacobianOplus[1];
        J.block<9, 3>(0, 12) = _jacobianOplus[5];
        J.block<9, 6>(0, 15) = _jacobianOplus[0];
        J.block<9, 6>(0, 21) = _jacobianOplus[4];
        return J.transpose() * information() * J;
    }

    // 关于偏置，重力方向与尺度的信息矩阵
    Eigen::Matrix<double, 9, 9> GetHessian3()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 9> J;
        J.block<9, 3>(0, 0) = _jacobianOplus[2];
        J.block<9, 3>(0, 3) = _jacobianOplus[3];
        J.block<9, 2>(0, 6) = _jacobianOplus[6];
        J.block<9, 1>(0, 8) = _jacobianOplus[7];
        return J.transpose() * information() * J;
    }

    // 下面的没有用到，其实也是获取状态的信息矩阵
    Eigen::Matrix<double, 1, 1> GetHessianScale()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 1> J = _jacobianOplus[7];
        return J.transpose() * information() * J;
    }

    Eigen::Matrix<double, 3, 3> GetHessianBiasGyro()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 3> J = _jacobianOplus[2];
        return J.transpose() * information() * J;
    }

    Eigen::Matrix<double, 3, 3> GetHessianBiasAcc()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 3> J = _jacobianOplus[3];
        return J.transpose() * information() * J;
    }

    Eigen::Matrix<double, 2, 2> GetHessianGDir()
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 2> J = _jacobianOplus[6];
        return J.transpose() * information() * J;
    }
};

/** 
 * @brief 陀螺仪偏置的二元边，除了残差及重投影误差外的第三个边，控制偏置变化
 */
class EdgeGyroRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexGyroBias, VertexGyroBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGyroRW() {}

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const VertexGyroBias *VG1 = static_cast<const VertexGyroBias *>(_vertices[0]);
        const VertexGyroBias *VG2 = static_cast<const VertexGyroBias *>(_vertices[1]);
        _error = VG2->estimate() - VG1->estimate();
    }

    virtual void linearizeOplus()
    {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double, 6, 6> GetHessian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }

    Eigen::Matrix3d GetHessian2()
    {
        linearizeOplus();
        return _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
    }
};

/** 
 * @brief 加速度计偏置的二元边，除了残差及重投影误差外的第三个边，控制偏置变化
 */
class EdgeAccRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexAccBias, VertexAccBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAccRW() {}

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const VertexAccBias *VA1 = static_cast<const VertexAccBias *>(_vertices[0]);
        const VertexAccBias *VA2 = static_cast<const VertexAccBias *>(_vertices[1]);
        _error = VA2->estimate() - VA1->estimate();
    }

    virtual void linearizeOplus()
    {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double, 6, 6> GetHessian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }

    Eigen::Matrix3d GetHessian2()
    {
        linearizeOplus();
        return _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
    }
};

/** 
 * @brief 先验类
 */
class ConstraintPoseImu
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintPoseImu(
        const Eigen::Matrix3d &Rwb_, const Eigen::Vector3d &twb_, const Eigen::Vector3d &vwb_,
        const Eigen::Vector3d &bg_, const Eigen::Vector3d &ba_, const Matrix15d &H_)
        : Rwb(Rwb_), twb(twb_), vwb(vwb_), bg(bg_), ba(ba_), H(H_)
    {
        H = (H + H) / 2;  // 对称化
        // 令特征值小的位置变为0
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(H);
        Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 15; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0;
        H = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    }

    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d vwb;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
    Matrix15d H;
};

/** 
 * @brief 先验边，前端优化单帧用到
 */
class EdgePriorPoseImu : public g2o::BaseMultiEdge<15, Vector15d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorPoseImu(ConstraintPoseImu *c);

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError();
    virtual void linearizeOplus();

    Eigen::Matrix<double, 15, 15> GetHessian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 15, 15> J;
        J.block<15, 6>(0, 0) = _jacobianOplus[0];
        J.block<15, 3>(0, 6) = _jacobianOplus[1];
        J.block<15, 3>(0, 9) = _jacobianOplus[2];
        J.block<15, 3>(0, 12) = _jacobianOplus[3];
        return J.transpose() * information() * J;
    }

    Eigen::Matrix<double, 9, 9> GetHessianNoPose()
    {
        linearizeOplus();
        Eigen::Matrix<double, 15, 9> J;
        J.block<15, 3>(0, 0) = _jacobianOplus[1];
        J.block<15, 3>(0, 3) = _jacobianOplus[2];
        J.block<15, 3>(0, 6) = _jacobianOplus[3];
        return J.transpose() * information() * J;
    }
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb, vwb;
    Eigen::Vector3d bg, ba;
};

// Priors for biases
/** 
 * @brief 根据给定值的加速度计先验边，目的是把优化量维持在先验附近
 */
class EdgePriorAcc : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexAccBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorAcc(const Eigen::Vector3f &bprior_) : bprior(bprior_.cast<double>()) {}

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const VertexAccBias *VA = static_cast<const VertexAccBias *>(_vertices[0]);
        _error = bprior - VA->estimate();
    }
    virtual void linearizeOplus();

    Eigen::Matrix<double, 3, 3> GetHessian()
    {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

    const Eigen::Vector3d bprior;
};

/** 
 * @brief 根据给定值的陀螺仪先验边，目的是把优化量维持在先验附近
 */
class EdgePriorGyro : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexGyroBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorGyro(const Eigen::Vector3f &bprior_) : bprior(bprior_.cast<double>()) {}

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const VertexGyroBias *VG = static_cast<const VertexGyroBias *>(_vertices[0]);
        _error = bprior - VG->estimate();
    }
    virtual void linearizeOplus();

    Eigen::Matrix<double, 3, 3> GetHessian()
    {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

    const Eigen::Vector3d bprior;
};

/** 
 * @brief 4DOF的二元边，误差为给定的旋转平移改变量 与 两个输入节点之间的旋转平移改变量的偏差
 */
class Edge4DoF : public g2o::BaseBinaryEdge<6, Vector6d, VertexPose4DoF, VertexPose4DoF>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Edge4DoF(const Eigen::Matrix4d &deltaT)
    {
        dTij = deltaT;
        dRij = deltaT.block<3, 3>(0, 0);
        dtij = deltaT.block<3, 1>(0, 3);
    }

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError()
    {
        const VertexPose4DoF *VPi = static_cast<const VertexPose4DoF *>(_vertices[0]);
        const VertexPose4DoF *VPj = static_cast<const VertexPose4DoF *>(_vertices[1]);
        _error << LogSO3(VPi->estimate().Rcw[0] * VPj->estimate().Rcw[0].transpose() * dRij.transpose()),
            VPi->estimate().Rcw[0] * (-VPj->estimate().Rcw[0].transpose() * VPj->estimate().tcw[0]) + VPi->estimate().tcw[0] - dtij;
    }

    // virtual void linearizeOplus(); // numerical implementation

    Eigen::Matrix4d dTij;
    Eigen::Matrix3d dRij;
    Eigen::Vector3d dtij;
};

} // namespace ORB_SLAM2

#endif // G2OTYPES_H
