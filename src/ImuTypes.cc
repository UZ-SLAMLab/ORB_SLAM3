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

#include "ImuTypes.h"
#include "Converter.h"

#include "GeometricTools.h"

#include<iostream>

namespace ORB_SLAM3
{

namespace IMU
{

const float eps = 1e-4;

Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R){
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z)
{
    Eigen::Matrix3f I;
    I.setIdentity();
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);
    if(d<eps) {
        return I;
    }
    else {
        return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v)
{
    return RightJacobianSO3(v(0),v(1),v(2));
}

Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z)
{
    Eigen::Matrix3f I;
    I.setIdentity();
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);

    if(d<eps) {
        return I;
    }
    else {
        return I + W/2 + W*W*(1.0f/d2 - (1.0f+cos(d))/(2.0f*d*sin(d)));
    }
}

Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v)
{
    return InverseRightJacobianSO3(v(0),v(1),v(2));
}

IntegratedRotation::IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time) {
    const float x = (angVel(0)-imuBias.bwx)*time;
    const float y = (angVel(1)-imuBias.bwy)*time;
    const float z = (angVel(2)-imuBias.bwz)*time;

    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);

    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);
    if(d<eps)
    {
        deltaR = Eigen::Matrix3f::Identity() + W;
        rightJ = Eigen::Matrix3f::Identity();
    }
    else
    {
        deltaR = Eigen::Matrix3f::Identity() + W*sin(d)/d + W*W*(1.0f-cos(d))/d2;
        rightJ = Eigen::Matrix3f::Identity() - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Preintegrated::Preintegrated(const Bias &b_, const Calib &calib)
{
    Nga = calib.Cov;
    NgaWalk = calib.CovWalk;
    Initialize(b_);
}

// Copy constructor
Preintegrated::Preintegrated(Preintegrated* pImuPre): dT(pImuPre->dT),C(pImuPre->C), Info(pImuPre->Info),
     Nga(pImuPre->Nga), NgaWalk(pImuPre->NgaWalk), b(pImuPre->b), dR(pImuPre->dR), dV(pImuPre->dV),
    dP(pImuPre->dP), JRg(pImuPre->JRg), JVg(pImuPre->JVg), JVa(pImuPre->JVa), JPg(pImuPre->JPg), JPa(pImuPre->JPa),
    avgA(pImuPre->avgA), avgW(pImuPre->avgW), bu(pImuPre->bu), db(pImuPre->db), mvMeasurements(pImuPre->mvMeasurements)
{

}

void Preintegrated::CopyFrom(Preintegrated* pImuPre)
{
    dT = pImuPre->dT;
    C = pImuPre->C;
    Info = pImuPre->Info;
    Nga = pImuPre->Nga;
    NgaWalk = pImuPre->NgaWalk;
    b.CopyFrom(pImuPre->b);
    dR = pImuPre->dR;
    dV = pImuPre->dV;
    dP = pImuPre->dP;
    JRg = pImuPre->JRg;
    JVg = pImuPre->JVg;
    JVa = pImuPre->JVa;
    JPg = pImuPre->JPg;
    JPa = pImuPre->JPa;
    avgA = pImuPre->avgA;
    avgW = pImuPre->avgW;
    bu.CopyFrom(pImuPre->bu);
    db = pImuPre->db;
    mvMeasurements = pImuPre->mvMeasurements;
}


void Preintegrated::Initialize(const Bias &b_)
{
    dR.setIdentity();
    dV.setZero();
    dP.setZero();
    JRg.setZero();
    JVg.setZero();
    JVa.setZero();
    JPg.setZero();
    JPa.setZero();
    C.setZero();
    Info.setZero();
    db.setZero();
    b=b_;
    bu=b_;
    avgA.setZero();
    avgW.setZero();
    dT=0.0f;
    mvMeasurements.clear();
}

void Preintegrated::Reintegrate()
{
    std::unique_lock<std::mutex> lock(mMutex);
    const std::vector<integrable> aux = mvMeasurements;
    Initialize(bu);
    for(size_t i=0;i<aux.size();i++)
        IntegrateNewMeasurement(aux[i].a,aux[i].w,aux[i].t);
}

void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt)
{
    mvMeasurements.push_back(integrable(acceleration,angVel,dt));

    // Position is updated firstly, as it depends on previously computed velocity and rotation.
    // Velocity is updated secondly, as it depends on previously computed rotation.
    // Rotation is the last to be updated.

    //Matrices to compute covariance
    Eigen::Matrix<float,9,9> A;
    A.setIdentity();
    Eigen::Matrix<float,9,6> B;
    B.setZero();

    Eigen::Vector3f acc, accW;
    acc << acceleration(0)-b.bax, acceleration(1)-b.bay, acceleration(2)-b.baz;
    accW << angVel(0)-b.bwx, angVel(1)-b.bwy, angVel(2)-b.bwz;

    avgA = (dT*avgA + dR*acc*dt)/(dT+dt);
    avgW = (dT*avgW + accW*dt)/(dT+dt);

    // Update delta position dP and velocity dV (rely on no-updated delta rotation)
    dP = dP + dV*dt + 0.5f*dR*acc*dt*dt;
    dV = dV + dR*acc*dt;

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    Eigen::Matrix<float,3,3> Wacc = Sophus::SO3f::hat(acc);

    A.block<3,3>(3,0) = -dR*dt*Wacc;
    A.block<3,3>(6,0) = -0.5f*dR*dt*dt*Wacc;
    A.block<3,3>(6,3) = Eigen::DiagonalMatrix<float,3>(dt, dt, dt);
    B.block<3,3>(3,3) = dR*dt;
    B.block<3,3>(6,3) = 0.5f*dR*dt*dt;


    // Update position and velocity jacobians wrt bias correction
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JVg = JVg - dR*dt*Wacc*JRg;

    // Update delta rotation
    IntegratedRotation dRi(angVel,b,dt);
    dR = NormalizeRotation(dR*dRi.deltaR);

    // Compute rotation parts of matrices A and B
    A.block<3,3>(0,0) = dRi.deltaR.transpose();
    B.block<3,3>(0,0) = dRi.rightJ*dt;

    // Update covariance
    C.block<9,9>(0,0) = A * C.block<9,9>(0,0) * A.transpose() + B*Nga*B.transpose();
    C.block<6,6>(9,9) += NgaWalk;

    // Update rotation jacobian wrt bias correction
    JRg = dRi.deltaR.transpose()*JRg - dRi.rightJ*dt;

    // Total integrated time
    dT += dt;
}

void Preintegrated::MergePrevious(Preintegrated* pPrev)
{
    if (pPrev==this)
        return;

    std::unique_lock<std::mutex> lock1(mMutex);
    std::unique_lock<std::mutex> lock2(pPrev->mMutex);
    Bias bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    const std::vector<integrable > aux1 = pPrev->mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements;

    Initialize(bav);
    for(size_t i=0;i<aux1.size();i++)
        IntegrateNewMeasurement(aux1[i].a,aux1[i].w,aux1[i].t);
    for(size_t i=0;i<aux2.size();i++)
        IntegrateNewMeasurement(aux2[i].a,aux2[i].w,aux2[i].t);

}

void Preintegrated::SetNewBias(const Bias &bu_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    bu = bu_;

    db(0) = bu_.bwx-b.bwx;
    db(1) = bu_.bwy-b.bwy;
    db(2) = bu_.bwz-b.bwz;
    db(3) = bu_.bax-b.bax;
    db(4) = bu_.bay-b.bay;
    db(5) = bu_.baz-b.baz;
}

IMU::Bias Preintegrated::GetDeltaBias(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    return IMU::Bias(b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz,b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
}


Eigen::Matrix3f Preintegrated::GetDeltaRotation(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    Eigen::Vector3f dbg;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * dbg).matrix());
}

Eigen::Vector3f Preintegrated::GetDeltaVelocity(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    dba << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz;
    return dV + JVg * dbg + JVa * dba;
}

Eigen::Vector3f Preintegrated::GetDeltaPosition(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    dba << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz;
    return dP + JPg * dbg + JPa * dba;
}

Eigen::Matrix3f Preintegrated::GetUpdatedDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return NormalizeRotation(dR * Sophus::SO3f::exp(JRg*db.head(3)).matrix());
}

Eigen::Vector3f Preintegrated::GetUpdatedDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dV + JVg * db.head(3) + JVa * db.tail(3);
}

Eigen::Vector3f Preintegrated::GetUpdatedDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP + JPg*db.head(3) + JPa*db.tail(3);
}

Eigen::Matrix3f Preintegrated::GetOriginalDeltaRotation() {
    std::unique_lock<std::mutex> lock(mMutex);
    return dR;
}

Eigen::Vector3f Preintegrated::GetOriginalDeltaVelocity() {
    std::unique_lock<std::mutex> lock(mMutex);
    return dV;
}

Eigen::Vector3f Preintegrated::GetOriginalDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP;
}

Bias Preintegrated::GetOriginalBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return b;
}

Bias Preintegrated::GetUpdatedBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return bu;
}

Eigen::Matrix<float,6,1> Preintegrated::GetDeltaBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return db;
}

void Bias::CopyFrom(Bias &b)
{
    bax = b.bax;
    bay = b.bay;
    baz = b.baz;
    bwx = b.bwx;
    bwy = b.bwy;
    bwz = b.bwz;
}

std::ostream& operator<< (std::ostream &out, const Bias &b)
{
    if(b.bwx>0)
        out << " ";
    out << b.bwx << ",";
    if(b.bwy>0)
        out << " ";
    out << b.bwy << ",";
    if(b.bwz>0)
        out << " ";
    out << b.bwz << ",";
    if(b.bax>0)
        out << " ";
    out << b.bax << ",";
    if(b.bay>0)
        out << " ";
    out << b.bay << ",";
    if(b.baz>0)
        out << " ";
    out << b.baz;

    return out;
}

void Calib::Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw) {
    mbIsSet = true;
    const float ng2 = ng*ng;
    const float na2 = na*na;
    const float ngw2 = ngw*ngw;
    const float naw2 = naw*naw;

    // Sophus/Eigen
    mTbc = sophTbc;
    mTcb = mTbc.inverse();
    Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
    CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
}

Calib::Calib(const Calib &calib)
{
    mbIsSet = calib.mbIsSet;
    // Sophus/Eigen parameters
    mTbc = calib.mTbc;
    mTcb = calib.mTcb;
    Cov = calib.Cov;
    CovWalk = calib.CovWalk;
}

} //namespace IMU

} //namespace ORB_SLAM2
