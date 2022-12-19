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


#include "Sim3Solver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM3
{


Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12, const bool bFixScale,
                       vector<KeyFrame*> vpKeyFrameMatchedMP):
    mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale),
    pCamera1(pKF1->mpCamera), pCamera2(pKF2->mpCamera)
{
    bool bDifferentKFs = false;
    if(vpKeyFrameMatchedMP.empty())
    {
        bDifferentKFs = true;
        vpKeyFrameMatchedMP = vector<KeyFrame*>(vpMatched12.size(), pKF2);
    }

    mpKF1 = pKF1;
    mpKF2 = pKF2;

    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    mN1 = vpMatched12.size();

    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);

    Eigen::Matrix3f Rcw1 = pKF1->GetRotation();
    Eigen::Vector3f tcw1 = pKF1->GetTranslation();
    Eigen::Matrix3f Rcw2 = pKF2->GetRotation();
    Eigen::Vector3f tcw2 = pKF2->GetTranslation();

    mvAllIndices.reserve(mN1);

    size_t idx=0;

    KeyFrame* pKFm = pKF2; //Default variable
    for(int i1=0; i1<mN1; i1++)
    {
        if(vpMatched12[i1])
        {
            MapPoint* pMP1 = vpKeyFrameMP1[i1];
            MapPoint* pMP2 = vpMatched12[i1];

            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            if(bDifferentKFs)
                pKFm = vpKeyFrameMatchedMP[i1];

            int indexKF1 = get<0>(pMP1->GetIndexInKeyFrame(pKF1));
            int indexKF2 = get<0>(pMP2->GetIndexInKeyFrame(pKFm));

            if(indexKF1<0 || indexKF2<0)
                continue;

            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKFm->mvKeysUn[indexKF2];

            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKFm->mvLevelSigma2[kp2.octave];

            mvnMaxError1.push_back(9.210*sigmaSquare1);
            mvnMaxError2.push_back(9.210*sigmaSquare2);

            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);

            Eigen::Vector3f X3D1w = pMP1->GetWorldPos();
            mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);

            Eigen::Vector3f X3D2w = pMP2->GetWorldPos();
            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    FromCameraToImage(mvX3Dc1,mvP1im1,pCamera1);
    FromCameraToImage(mvX3Dc2,mvP2im2,pCamera2);

    SetRansacParameters();
}

void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;    

    N = mvpMapPoints1.size(); // number of correspondences

    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mnIterations = 0;
}

Eigen::Matrix4f Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers = vector<bool>(mN1,false);
    nInliers=0;

    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return Eigen::Matrix4f::Identity();
    }

    vector<size_t> vAvailableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

        vAvailableIndices = mvAllIndices;

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            P3Dc1i.col(i) = mvX3Dc1[idx];
            P3Dc2i.col(i) = mvX3Dc2[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        ComputeSim3(P3Dc1i,P3Dc2i);

        CheckInliers();

        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i;
            mBestRotation = mR12i;
            mBestTranslation = mt12i;
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return Eigen::Matrix4f::Identity();
}

Eigen::Matrix4f Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers, bool &bConverge)
{
    bNoMore = false;
    bConverge = false;
    vbInliers = vector<bool>(mN1,false);
    nInliers=0;

    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return Eigen::Matrix4f::Identity();
    }

    vector<size_t> vAvailableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int nCurrentIterations = 0;

    Eigen::Matrix4f bestSim3;

    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

        vAvailableIndices = mvAllIndices;

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            P3Dc1i.col(i) = mvX3Dc1[idx];
            P3Dc2i.col(i) = mvX3Dc2[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        ComputeSim3(P3Dc1i,P3Dc2i);

        CheckInliers();

        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i;
            mBestRotation = mR12i;
            mBestTranslation = mt12i;
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                bConverge = true;
                return mBestT12;
            }
            else
            {
                bestSim3 = mBestT12;
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return bestSim3;
}

Eigen::Matrix4f Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

void Sim3Solver::ComputeCentroid(Eigen::Matrix3f &P, Eigen::Matrix3f &Pr, Eigen::Vector3f &C)
{
    C = P.rowwise().sum();
    C = C / P.cols();
    for(int i=0; i<P.cols(); i++)
    Pr.col(i) = P.col(i) - C;
}


void Sim3Solver::ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2)
{
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Step 1: Centroid and relative coordinates

    Eigen::Matrix3f Pr1; // Relative coordinates to centroid (set 1)
    Eigen::Matrix3f Pr2; // Relative coordinates to centroid (set 2)
    Eigen::Vector3f O1; // Centroid of P1
    Eigen::Vector3f O2; // Centroid of P2

    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // Step 2: Compute M matrix

    Eigen::Matrix3f M = Pr2 * Pr1.transpose();

    // Step 3: Compute N matrix
    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    Eigen::Matrix4f N;

    N11 = M(0,0)+M(1,1)+M(2,2);
    N12 = M(1,2)-M(2,1);
    N13 = M(2,0)-M(0,2);
    N14 = M(0,1)-M(1,0);
    N22 = M(0,0)-M(1,1)-M(2,2);
    N23 = M(0,1)+M(1,0);
    N24 = M(2,0)+M(0,2);
    N33 = -M(0,0)+M(1,1)-M(2,2);
    N34 = M(1,2)+M(2,1);
    N44 = -M(0,0)-M(1,1)+M(2,2);

    N << N11, N12, N13, N14,
         N12, N22, N23, N24,
         N13, N23, N33, N34,
         N14, N24, N34, N44;


    // Step 4: Eigenvector of the highest eigenvalue
    Eigen::EigenSolver<Eigen::Matrix4f> eigSolver;
    eigSolver.compute(N);

    Eigen::Vector4f eval = eigSolver.eigenvalues().real();
    Eigen::Matrix4f evec = eigSolver.eigenvectors().real(); //evec[0] is the quaternion of the desired rotation

    int maxIndex; // should be zero
    eval.maxCoeff(&maxIndex);

    Eigen::Vector3f vec = evec.block<3,1>(1,maxIndex); //extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    double ang=atan2(vec.norm(),evec(0,maxIndex));

    vec = 2*ang*vec/vec.norm(); //Angle-axis representation. quaternion angle is the half
    mR12i = Sophus::SO3f::exp(vec).matrix();

    // Step 5: Rotate set 2
    Eigen::Matrix3f P3 = mR12i*Pr2;

    // Step 6: Scale

    if(!mbFixScale)
    {
        double cvnom = Converter::toCvMat(Pr1).dot(Converter::toCvMat(P3));
        double nom = (Pr1.array() * P3.array()).sum();
        if (abs(nom-cvnom)>1e-3)
            std::cout << "sim3 solver: " << abs(nom-cvnom) << std::endl << nom << std::endl;
        Eigen::Array<float,3,3> aux_P3;
        aux_P3 = P3.array() * P3.array();
        double den = aux_P3.sum();

        ms12i = nom/den;
    }
    else
        ms12i = 1.0f;

    // Step 7: Translation
    mt12i = O1 - ms12i * mR12i * O2;

    // Step 8: Transformation

    // Step 8.1 T12
    mT12i.setIdentity();

    Eigen::Matrix3f sR = ms12i*mR12i;
    mT12i.block<3,3>(0,0) = sR;
    mT12i.block<3,1>(0,3) = mt12i;


    // Step 8.2 T21
    mT21i.setIdentity();
    Eigen::Matrix3f sRinv = (1.0/ms12i)*mR12i.transpose();

    // sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    mT21i.block<3,3>(0,0) = sRinv;

    Eigen::Vector3f tinv = -sRinv * mt12i;
    mT21i.block<3,1>(0,3) = tinv;
}


void Sim3Solver::CheckInliers()
{
    vector<Eigen::Vector2f> vP1im2, vP2im1;
    Project(mvX3Dc2,vP2im1,mT12i,pCamera1);
    Project(mvX3Dc1,vP1im2,mT21i,pCamera2);

    mnInliersi=0;

    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        Eigen::Vector2f dist1 = mvP1im1[i] - vP2im1[i];
        Eigen::Vector2f dist2 = vP1im2[i] - mvP2im2[i];

        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }
}

Eigen::Matrix4f Sim3Solver::GetEstimatedTransformation()
{
    return mBestT12;
}

Eigen::Matrix3f Sim3Solver::GetEstimatedRotation()
{
    return mBestRotation;
}

Eigen::Vector3f Sim3Solver::GetEstimatedTranslation()
{
    return mBestTranslation;
}

float Sim3Solver::GetEstimatedScale()
{
    return mBestScale;
}

void Sim3Solver::Project(const vector<Eigen::Vector3f> &vP3Dw, vector<Eigen::Vector2f> &vP2D, Eigen::Matrix4f Tcw, GeometricCamera* pCamera)
{
    Eigen::Matrix3f Rcw = Tcw.block<3,3>(0,0);
    Eigen::Vector3f tcw = Tcw.block<3,1>(0,3);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        Eigen::Vector3f P3Dc = Rcw*vP3Dw[i]+tcw;
        Eigen::Vector2f pt2D = pCamera->project(P3Dc);
        vP2D.push_back(pt2D);
    }
}

void Sim3Solver::FromCameraToImage(const vector<Eigen::Vector3f> &vP3Dc, vector<Eigen::Vector2f> &vP2D, GeometricCamera* pCamera)
{
    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        Eigen::Vector2f pt2D = pCamera->project(vP3Dc[i]);
        vP2D.push_back(pt2D);
    }
}

} //namespace ORB_SLAM
