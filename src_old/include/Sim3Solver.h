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


#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"



namespace ORB_SLAM3
{

class Sim3Solver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true,
               const vector<KeyFrame*> vpKeyFrameMatchedMP = vector<KeyFrame*>());

    void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

    Eigen::Matrix4f find(std::vector<bool> &vbInliers12, int &nInliers);

    Eigen::Matrix4f iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);
    Eigen::Matrix4f iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers, bool &bConverge);

    Eigen::Matrix4f GetEstimatedTransformation();
    Eigen::Matrix3f GetEstimatedRotation();
    Eigen::Vector3f GetEstimatedTranslation();
    float GetEstimatedScale();

protected:

    void ComputeCentroid(Eigen::Matrix3f &P, Eigen::Matrix3f &Pr, Eigen::Vector3f &C);

    void ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2);

    void CheckInliers();

    void Project(const std::vector<Eigen::Vector3f> &vP3Dw, std::vector<Eigen::Vector2f> &vP2D, Eigen::Matrix4f Tcw, GeometricCamera* pCamera);
    void FromCameraToImage(const std::vector<Eigen::Vector3f> &vP3Dc, std::vector<Eigen::Vector2f> &vP2D, GeometricCamera* pCamera);


protected:

    // KeyFrames and matches
    KeyFrame* mpKF1;
    KeyFrame* mpKF2;

    std::vector<Eigen::Vector3f> mvX3Dc1;
    std::vector<Eigen::Vector3f> mvX3Dc2;
    std::vector<MapPoint*> mvpMapPoints1;
    std::vector<MapPoint*> mvpMapPoints2;
    std::vector<MapPoint*> mvpMatches12;
    std::vector<size_t> mvnIndices1;
    std::vector<size_t> mvSigmaSquare1;
    std::vector<size_t> mvSigmaSquare2;
    std::vector<size_t> mvnMaxError1;
    std::vector<size_t> mvnMaxError2;

    int N;
    int mN1;

    // Current Estimation
    Eigen::Matrix3f mR12i;
    Eigen::Vector3f mt12i;
    float ms12i;
    Eigen::Matrix4f mT12i;
    Eigen::Matrix4f mT21i;
    std::vector<bool> mvbInliersi;
    int mnInliersi;

    // Current Ransac State
    int mnIterations;
    std::vector<bool> mvbBestInliers;
    int mnBestInliers;
    Eigen::Matrix4f mBestT12;
    Eigen::Matrix3f mBestRotation;
    Eigen::Vector3f mBestTranslation;
    float mBestScale;

    // Scale is fixed to 1 in the stereo/RGBD case
    bool mbFixScale;

    // Indices for random selection
    std::vector<size_t> mvAllIndices;

    // Projections
    std::vector<Eigen::Vector2f> mvP1im1;
    std::vector<Eigen::Vector2f> mvP2im2;

    // RANSAC probability
    double mRansacProb;

    // RANSAC min inliers
    int mRansacMinInliers;

    // RANSAC max iterations
    int mRansacMaxIts;

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh;
    float mSigma2;

    // Calibration
    //cv::Mat mK1;
    //cv::Mat mK2;

    GeometricCamera* pCamera1, *pCamera2;

};

} //namespace ORB_SLAM

#endif // SIM3SOLVER_H
