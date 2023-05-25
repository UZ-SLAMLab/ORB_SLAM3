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

/******************************************************************************
* Author:   Steffen Urban                                              *
* Contact:  urbste@gmail.com                                          *
* License:  Copyright (c) 2016 Steffen Urban, ANU. All rights reserved.      *
*                                                                            *
* Redistribution and use in source and binary forms, with or without         *
* modification, are permitted provided that the following conditions         *
* are met:                                                                   *
* * Redistributions of source code must retain the above copyright           *
*   notice, this list of conditions and the following disclaimer.            *
* * Redistributions in binary form must reproduce the above copyright        *
*   notice, this list of conditions and the following disclaimer in the      *
*   documentation and/or other materials provided with the distribution.     *
* * Neither the name of ANU nor the names of its contributors may be         *
*   used to endorse or promote products derived from this software without   *
*   specific prior written permission.                                       *
*                                                                            *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
* ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
* SUCH DAMAGE.                                                               *
******************************************************************************/

#include "MLPnPsolver.h"

#include <Eigen/Sparse>


namespace ORB_SLAM3 {
    this->K << F.fx, 0, F.cx,
    0, F.fy, F.cy,
    0, 0, 1;

    MLPnPsolver::MLPnPsolver(const Frame &F, const vector<MapPoint *> &vpMapPointMatches):
            mnInliersi(0), mnIterations(0), mnBestInliers(0), N(0), mpCamera(F.mpCamera){
        mvpMapPointMatches = vpMapPointMatches;
        mvBearingVecs.reserve(F.mvpMapPoints.size());
        mvP2D.reserve(F.mvpMapPoints.size());
        mvSigma2.reserve(F.mvpMapPoints.size());
        mvP3Dw.reserve(F.mvpMapPoints.size());
        mvKeyPointIndices.reserve(F.mvpMapPoints.size());
        mvAllIndices.reserve(F.mvpMapPoints.size());

        int idx = 0;
        for(size_t i = 0, iend = mvpMapPointMatches.size(); i < iend; i++){
            MapPoint* pMP = vpMapPointMatches[i];

            if(pMP){
                if(!pMP -> isBad()){
                    if(i >= F.mvKeysUn.size()) continue;
                    const cv::KeyPoint &kp = F.mvKeysUn[i];

                    mvP2D.push_back(kp.pt);
                    mvSigma2.push_back(F.mvLevelSigma2[kp.octave]);

                    //Bearing vector should be normalized
                    cv::Point3f cv_br = mpCamera->unproject(kp.pt);
                    cv_br /= cv_br.z;
                    bearingVector_t br(cv_br.x,cv_br.y,cv_br.z);
                    mvBearingVecs.push_back(br);

                    //3D coordinates
                    Eigen::Matrix<float,3,1> posEig = pMP -> GetWorldPos();
                    point_t pos(posEig(0),posEig(1),posEig(2));
                    mvP3Dw.push_back(pos);

                    mvKeyPointIndices.push_back(i);
                    mvAllIndices.push_back(idx);

                    idx++;
                }
            }
        }

        SetRansacParameters();
    }

    // Struct for holding the output of pix2rays function
    struct Pix2RaysResult {
        Eigen::MatrixXd v;
        Eigen::MatrixXd sigma_v;
    };

    // Function to convert pixel coordinates to rays
    Pix2RaysResult pix2rays(Eigen::MatrixXd pix, Eigen::MatrixXd cov = Eigen::MatrixXd()) {
        // Add z coordinate if not already existing (=1, on image plane)
        if (pix.rows() == 2) {
            Eigen::MatrixXd ones = Eigen::MatrixXd::Ones(1, pix.cols());
            pix.conservativeResize(3, Eigen::NoChange);
            pix.row(2) = ones;
        }

        Eigen::MatrixXd x = K.inverse() * pix;
        Eigen::VectorXd norm_x = x.colwise().norm();
        Eigen::MatrixXd v = x.array().colwise() / norm_x.array();

        Eigen::MatrixXd sigma_v;

        // If observation covariance is provided, propagate it
        if (cov.size() == 0) {
            sigma_v = Eigen::MatrixXd();
        } else {
            int nb_pts = pix.cols();
            sigma_v = Eigen::MatrixXd::Zero(9, nb_pts);
            Eigen::MatrixXd sigma_x = Eigen::MatrixXd::Zero(3, 3);

            for (int i = 0; i < nb_pts; i++) {
                sigma_x.block<2, 2>(0, 0) = Eigen::Map<Eigen::Matrix2d>(cov.col(i).data());
                Eigen::MatrixXd J = Eigen::MatrixXd::Identity(3, 3) / norm_x(i) - (v.col(i) * v.col(i).transpose());
                sigma_v.col(i) = Eigen::Map<Eigen::VectorXd>((J * sigma_x * J.transpose()).data(), 9);
            }
        }

        return {v, sigma_v};
    }

    //RANSAC methods
    bool MLPnPsolver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers, Eigen::Matrix4f &Tout){
        Tout.setIdentity();
        bNoMore = false;
	    vbInliers.clear();
	    nInliers=0;

	    if(N<mRansacMinInliers)
	    {
	        bNoMore = true;
	        return false;
	    }

	    vector<size_t> vAvailableIndices;

	    int nCurrentIterations = 0;
	    while(mnIterations<mRansacMaxIts || nCurrentIterations<nIterations)
	    {
	        nCurrentIterations++;
	        mnIterations++;

	        vAvailableIndices = mvAllIndices;

            //Bearing vectors and 3D points used for this ransac iteration
            bearingVectors_t bearingVecs(mRansacMinSet);
            points_t p3DS(mRansacMinSet);
            vector<int> indexes(mRansacMinSet);

	        // Get min set of points
	        for(int i = 0; i < mRansacMinSet; ++i)
	        {
	            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

	            int idx = vAvailableIndices[randi];

                bearingVecs[i] = mvBearingVecs[idx];
                p3DS[i] = mvP3Dw[idx];
                indexes[i] = i;

	            vAvailableIndices[randi] = vAvailableIndices.back();
	            vAvailableIndices.pop_back();
	        }

            //By the moment, we are using MLPnP without covariance info
            cov3_mats_t covs(1);

            //Result
            transformation_t result;

	        // Compute camera pose
            computePose(bearingVecs,p3DS,covs,indexes,result);

            //Save result
            mRi[0][0] = result(0,0);
            mRi[0][1] = result(0,1);
            mRi[0][2] = result(0,2);

            mRi[1][0] = result(1,0);
            mRi[1][1] = result(1,1);
            mRi[1][2] = result(1,2);

            mRi[2][0] = result(2,0);
            mRi[2][1] = result(2,1);
            mRi[2][2] = result(2,2);

            mti[0] = result(0,3);mti[1] = result(1,3);mti[2] = result(2,3);

	        // Check inliers
	        CheckInliers();

	        if(mnInliersi>=mRansacMinInliers)
	        {
	            // If it is the best solution so far, save it
	            if(mnInliersi>mnBestInliers)
	            {
	                mvbBestInliers = mvbInliersi;
	                mnBestInliers = mnInliersi;

	                cv::Mat Rcw(3,3,CV_64F,mRi);
	                cv::Mat tcw(3,1,CV_64F,mti);
	                Rcw.convertTo(Rcw,CV_32F);
	                tcw.convertTo(tcw,CV_32F);
                    mBestTcw.setIdentity();
                    mBestTcw.block<3,3>(0,0) = Converter::toMatrix3f(Rcw);
                    mBestTcw.block<3,1>(0,3) = Converter::toVector3f(tcw);

                    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> eigRcw(mRi[0]);
                    Eigen::Vector3d eigtcw(mti);
	            }

	            if(Refine())
	            {
	                nInliers = mnRefinedInliers;
	                vbInliers = vector<bool>(mvpMapPointMatches.size(),false);
	                for(int i=0; i<N; i++)
	                {
	                    if(mvbRefinedInliers[i])
	                        vbInliers[mvKeyPointIndices[i]] = true;
	                }
	                Tout = mRefinedTcw;
	                return true;
	            }

	        }
	    }

	    if(mnIterations>=mRansacMaxIts)
	    {
	        bNoMore=true;
	        if(mnBestInliers>=mRansacMinInliers)
	        {
	            nInliers=mnBestInliers;
	            vbInliers = vector<bool>(mvpMapPointMatches.size(),false);
	            for(int i=0; i<N; i++)
	            {
	                if(mvbBestInliers[i])
	                    vbInliers[mvKeyPointIndices[i]] = true;
	            }
	            Tout = mBestTcw;
	            return true;
	        }
	    }

	    return false;
	}

	void MLPnPsolver::SetRansacParameters(double probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2){
		mRansacProb = probability;
	    mRansacMinInliers = minInliers;
	    mRansacMaxIts = maxIterations;
	    mRansacEpsilon = epsilon;
	    mRansacMinSet = minSet;

	    N = mvP2D.size(); // number of correspondences

	    mvbInliersi.resize(N);

	    // Adjust Parameters according to number of correspondences
	    int nMinInliers = N*mRansacEpsilon;
	    if(nMinInliers<mRansacMinInliers)
	        nMinInliers=mRansacMinInliers;
	    if(nMinInliers<minSet)
	        nMinInliers=minSet;
	    mRansacMinInliers = nMinInliers;

	    if(mRansacEpsilon<(float)mRansacMinInliers/N)
	        mRansacEpsilon=(float)mRansacMinInliers/N;

	    // Set RANSAC iterations according to probability, epsilon, and max iterations
	    int nIterations;

	    if(mRansacMinInliers==N)
	        nIterations=1;
	    else
	        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));

	    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

	    mvMaxError.resize(mvSigma2.size());
	    for(size_t i=0; i<mvSigma2.size(); i++)
	        mvMaxError[i] = mvSigma2[i]*th2;
	}

    void MLPnPsolver::CheckInliers(){
        mnInliersi=0;

        for(int i=0; i<N; i++)
        {
            point_t p = mvP3Dw[i];
            cv::Point3f P3Dw(p(0),p(1),p(2));
            cv::Point2f P2D = mvP2D[i];

            float xc = mRi[0][0]*P3Dw.x+mRi[0][1]*P3Dw.y+mRi[0][2]*P3Dw.z+mti[0];
            float yc = mRi[1][0]*P3Dw.x+mRi[1][1]*P3Dw.y+mRi[1][2]*P3Dw.z+mti[1];
            float zc = mRi[2][0]*P3Dw.x+mRi[2][1]*P3Dw.y+mRi[2][2]*P3Dw.z+mti[2];

            cv::Point3f P3Dc(xc,yc,zc);
            cv::Point2f uv = mpCamera->project(P3Dc);

            float distX = P2D.x-uv.x;
            float distY = P2D.y-uv.y;

            float error2 = distX*distX+distY*distY;

            if(error2<mvMaxError[i])
            {
                mvbInliersi[i]=true;
                mnInliersi++;
            }
            else
            {
                mvbInliersi[i]=false;
            }
        }
    }

    bool MLPnPsolver::Refine(){
        vector<int> vIndices;
        vIndices.reserve(mvbBestInliers.size());

        for(size_t i=0; i<mvbBestInliers.size(); i++)
        {
            if(mvbBestInliers[i])
            {
                vIndices.push_back(i);
            }
        }

        //Bearing vectors and 3D points used for this ransac iteration
        bearingVectors_t bearingVecs;
        points_t p3DS;
        vector<int> indexes;

        for(size_t i=0; i<vIndices.size(); i++)
        {
            int idx = vIndices[i];

            bearingVecs.push_back(mvBearingVecs[idx]);
            p3DS.push_back(mvP3Dw[idx]);
            indexes.push_back(i);
        }

        //By the moment, we are using MLPnP without covariance info
        cov3_mats_t covs(1);

        //Result
        transformation_t result;

        // Compute camera pose
        computePose(bearingVecs,p3DS,covs,indexes,result);

        // Check inliers
        CheckInliers();

        mnRefinedInliers =mnInliersi;
        mvbRefinedInliers = mvbInliersi;

        if(mnInliersi>mRansacMinInliers)
        {
            cv::Mat Rcw(3,3,CV_64F,mRi);
            cv::Mat tcw(3,1,CV_64F,mti);
            Rcw.convertTo(Rcw,CV_32F);
            tcw.convertTo(tcw,CV_32F);
            mRefinedTcw.setIdentity();

            mRefinedTcw.block<3,3>(0,0) = Converter::toMatrix3f(Rcw);
            mRefinedTcw.block<3,1>(0,3) = Converter::toVector3f(tcw);

            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> eigRcw(mRi[0]);
            Eigen::Vector3d eigtcw(mti);

            return true;
        }
        return false;
    }

	//MLPnP methods
	//computePose is called in Refine and in RANSAC funcs
    void MLPnPsolver::computePose(const bearingVectors_t &f, const points_t &p, const cov3_mats_t &covMats,
                                  const std::vector<int> &indices, transformation_t &result) {
        size_t numberCorrespondences = indices.size();
        assert(numberCorrespondences > 5);

        bool planar = false;
        // compute the nullspace of all vectors
        std::vector<Eigen::MatrixXd> nullspaces(numberCorrespondences);
        Eigen::MatrixXd points3(3, numberCorrespondences);
        points_t points3v(numberCorrespondences);
        points4_t points4v(numberCorrespondences);
        for (size_t i = 0; i < numberCorrespondences; i++) {
            bearingVector_t f_current = f[indices[i]];
            points3.col(i) = p[indices[i]];
            // nullspace of right vector
            Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner>
                    svd_f(f_current.transpose(), Eigen::ComputeFullV);
            nullspaces[i] = svd_f.matrixV().block(0, 1, 3, 2);
            points3v[i] = p[indices[i]];
        }

        //TODO: compute covMatrix according to points matrix: points3v[i]
        Eigen::Matrix3d covMatrix;

        //////////////////////////////////////
        // 1. test if we have a planar scene
        //////////////////////////////////////

        Eigen::Matrix3d planarTest = points3 * points3.transpose();
        Eigen::FullPivHouseholderQR<Eigen::Matrix3d> rankTest(planarTest);
        Eigen::Matrix3d eigenRot;
        eigenRot.setIdentity();

        // if yes -> transform points to new eigen frame
        //if (minEigenVal < 1e-3 || minEigenVal == 0.0)
        rankTest.setThreshold(1e-5);
        //rankTest.setThreshold(1e-10); // didn't change
        if (rankTest.rank() == 2) { // TODO:  check setting threshold to lower value
            planar = true;
            // self adjoint is faster and more accurate than general eigen solvers
            // also has closed form solution for 3x3 self-adjoint matrices
            // in addition this solver sorts the eigenvalues in increasing order
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(planarTest);
            eigenRot = eigen_solver.eigenvectors().real();
            eigenRot.transposeInPlace();
            for (size_t i = 0; i < numberCorrespondences; i++)
                points3.col(i) = eigenRot * points3.col(i);
        }
        //////////////////////////////////////
        // 2. stochastic model
        //////////////////////////////////////
        Eigen::SparseMatrix<double> P(2 * numberCorrespondences,
                                      2 * numberCorrespondences);
        bool use_cov = false;
        P.setIdentity(); // standard

        // if we do have covariance information
        // -> fill covariance matrix
        covMats = pix2rays(points3v, covMatrix) // TODO: remove covmax from function sig or xhane here
        if (covMats.size() == numberCorrespondences) { //when is it not equal, check use of COV matrix
            //  numberCorrespondences is the size of input indices vector, and covMats is input
            use_cov = true;
            int l = 0;
            for (size_t i = 0; i < numberCorrespondences; ++i) {
                // invert matrix
                cov2_mat_t temp = nullspaces[i].transpose() * covMats[i] * nullspaces[i];
                temp = temp.inverse().eval();
                P.coeffRef(l, l) = temp(0, 0);
                P.coeffRef(l, l + 1) = temp(0, 1);
                P.coeffRef(l + 1, l) = temp(1, 0);
                P.coeffRef(l + 1, l + 1) = temp(1, 1);
                l += 2;
            }
        }

        //////////////////////////////////////
        // 3. fill the design matrix A
        //////////////////////////////////////
        const int rowsA = 2 * numberCorrespondences;
        int colsA = 12;
        Eigen::MatrixXd A;
        if (planar) {
            colsA = 9;
            A = Eigen::MatrixXd(rowsA, 9);
        } else
            A = Eigen::MatrixXd(rowsA, 12);
        A.setZero();

        // fill design matrix
        if (planar) {
            for (size_t i = 0; i < numberCorrespondences; ++i) {
                point_t pt3_current = points3.col(i);

                // r12
                A(2 * i, 0) = nullspaces[i](0, 0) * pt3_current[1];
                A(2 * i + 1, 0) = nullspaces[i](0, 1) * pt3_current[1];
                // r13
                A(2 * i, 1) = nullspaces[i](0, 0) * pt3_current[2];
                A(2 * i + 1, 1) = nullspaces[i](0, 1) * pt3_current[2];
                // r22
                A(2 * i, 2) = nullspaces[i](1, 0) * pt3_current[1];
                A(2 * i + 1, 2) = nullspaces[i](1, 1) * pt3_current[1];
                // r23
                A(2 * i, 3) = nullspaces[i](1, 0) * pt3_current[2];
                A(2 * i + 1, 3) = nullspaces[i](1, 1) * pt3_current[2];
                // r32
                A(2 * i, 4) = nullspaces[i](2, 0) * pt3_current[1];
                A(2 * i + 1, 4) = nullspaces[i](2, 1) * pt3_current[1];
                // r33
                A(2 * i, 5) = nullspaces[i](2, 0) * pt3_current[2];
                A(2 * i + 1, 5) = nullspaces[i](2, 1) * pt3_current[2];
                // t1
                A(2 * i, 6) = nullspaces[i](0, 0);
                A(2 * i + 1, 6) = nullspaces[i](0, 1);
                // t2
                A(2 * i, 7) = nullspaces[i](1, 0);
                A(2 * i + 1, 7) = nullspaces[i](1, 1);
                // t3
                A(2 * i, 8) = nullspaces[i](2, 0);
                A(2 * i + 1, 8) = nullspaces[i](2, 1);
            }
        } else {
            for (size_t i = 0; i < numberCorrespondences; ++i) {
                point_t pt3_current = points3.col(i);

                // r11
                A(2 * i, 0) = nullspaces[i](0, 0) * pt3_current[0];
                A(2 * i + 1, 0) = nullspaces[i](0, 1) * pt3_current[0];
                // r12
                A(2 * i, 1) = nullspaces[i](0, 0) * pt3_current[1];
                A(2 * i + 1, 1) = nullspaces[i](0, 1) * pt3_current[1];
                // r13
                A(2 * i, 2) = nullspaces[i](0, 0) * pt3_current[2];
                A(2 * i + 1, 2) = nullspaces[i](0, 1) * pt3_current[2];
                // r21
                A(2 * i, 3) = nullspaces[i](1, 0) * pt3_current[0];
                A(2 * i + 1, 3) = nullspaces[i](1, 1) * pt3_current[0];
                // r22
                A(2 * i, 4) = nullspaces[i](1, 0) * pt3_current[1];
                A(2 * i + 1, 4) = nullspaces[i](1, 1) * pt3_current[1];
                // r23
                A(2 * i, 5) = nullspaces[i](1, 0) * pt3_current[2];
                A(2 * i + 1, 5) = nullspaces[i](1, 1) * pt3_current[2];
                // r31
                A(2 * i, 6) = nullspaces[i](2, 0) * pt3_current[0];
                A(2 * i + 1, 6) = nullspaces[i](2, 1) * pt3_current[0];
                // r32
                A(2 * i, 7) = nullspaces[i](2, 0) * pt3_current[1];
                A(2 * i + 1, 7) = nullspaces[i](2, 1) * pt3_current[1];
                // r33
                A(2 * i, 8) = nullspaces[i](2, 0) * pt3_current[2];
                A(2 * i + 1, 8) = nullspaces[i](2, 1) * pt3_current[2];
                // t1
                A(2 * i, 9) = nullspaces[i](0, 0);
                A(2 * i + 1, 9) = nullspaces[i](0, 1);
                // t2
                A(2 * i, 10) = nullspaces[i](1, 0);
                A(2 * i + 1, 10) = nullspaces[i](1, 1);
                // t3
                A(2 * i, 11) = nullspaces[i](2, 0);
                A(2 * i + 1, 11) = nullspaces[i](2, 1);
            }
        }

        //////////////////////////////////////
        // 4. solve least squares
        //////////////////////////////////////
        Eigen::MatrixXd AtPA;
        if (use_cov) // there's covariance, we chacked
            AtPA = A.transpose() * P * A; // setting up the full normal equations seems to be unstable
        else
            AtPA = A.transpose() * A;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(AtPA, Eigen::ComputeFullV); //TODO: Eigen::ComputeThin check
        Eigen::MatrixXd result1 = svd_A.matrixV().col(colsA - 1);

        ////////////////////////////////
        // now we treat the results differently,
        // depending on the scene (planar or not)
        ////////////////////////////////
        rotation_t Rout;
        translation_t tout;
        if (planar) // planar case
        {
            rotation_t tmp;
            // until now, we only estimated
            // row one and two of the transposed rotation matrix
            tmp << 0.0, result1(0, 0), result1(1, 0),
                    0.0, result1(2, 0), result1(3, 0),
                    0.0, result1(4, 0), result1(5, 0);
            // row 3
            tmp.col(0) = tmp.col(1).cross(tmp.col(2));
            tmp.transposeInPlace();

            double scale = 1.0 / std::sqrt(std::abs(tmp.col(1).norm() * tmp.col(2).norm()));
            // find best rotation matrix in frobenius sense
            Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
            rotation_t Rout1 = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();
            // test if we found a good rotation matrix
            if (Rout1.determinant() < 0)
                Rout1 *= -1.0;
            // rotate this matrix back using the eigen frame
            Rout1 = eigenRot.transpose() * Rout1;

            translation_t t = scale * translation_t(result1(6, 0), result1(7, 0), result1(8, 0));
            Rout1.transposeInPlace();
            Rout1 *= -1;
            if (Rout1.determinant() < 0.0)
                Rout1.col(2) *= -1;
            // now we have to find the best out of 4 combinations
            rotation_t R1, R2;
            R1.col(0) = Rout1.col(0);
            R1.col(1) = Rout1.col(1);
            R1.col(2) = Rout1.col(2);
            R2.col(0) = -Rout1.col(0);
            R2.col(1) = -Rout1.col(1);
            R2.col(2) = Rout1.col(2);

            vector<transformation_t, Eigen::aligned_allocator<transformation_t>> Ts(4);
            Ts[0].block<3, 3>(0, 0) = R1;
            Ts[0].block<3, 1>(0, 3) = t;
            Ts[1].block<3, 3>(0, 0) = R1;
            Ts[1].block<3, 1>(0, 3) = -t;
            Ts[2].block<3, 3>(0, 0) = R2;
            Ts[2].block<3, 1>(0, 3) = t;
            Ts[3].block<3, 3>(0, 0) = R2;
            Ts[3].block<3, 1>(0, 3) = -t;

            vector<double> normVal(4);
            for (int i = 0; i < 4; ++i) {
                point_t reproPt;
                double norms = 0.0;
                for (int p = 0; p < 6; ++p) {
                    reproPt = Ts[i].block<3, 3>(0, 0) * points3v[p] + Ts[i].block<3, 1>(0, 3);
                    reproPt = reproPt / reproPt.norm();
                    norms += (1.0 - reproPt.transpose() * f[indices[p]]);
                }
                normVal[i] = norms;
            }
            std::vector<double>::iterator
                    findMinRepro = std::min_element(std::begin(normVal), std::end(normVal));
            int idx = std::distance(std::begin(normVal), findMinRepro);
            Rout = Ts[idx].block<3, 3>(0, 0);
            tout = Ts[idx].block<3, 1>(0, 3);
        } else // non-planar
        {
            rotation_t tmp;
            tmp << result1(0, 0), result1(3, 0), result1(6, 0),
                    result1(1, 0), result1(4, 0), result1(7, 0),
                    result1(2, 0), result1(5, 0), result1(8, 0);
            // get the scale
            double scale = 1.0 /
                           std::pow(std::abs(tmp.col(0).norm() * tmp.col(1).norm() * tmp.col(2).norm()), 1.0 / 3.0);
            //double scale = 1.0 / std::sqrt(std::abs(tmp.col(0).norm() * tmp.col(1).norm()));
            // find best rotation matrix in frobenius sense
            Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Rout = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();
            // test if we found a good rotation matrix
            if (Rout.determinant() < 0)
                Rout *= -1.0;
            // scale translation
            tout = Rout * (scale * translation_t(result1(9, 0), result1(10, 0), result1(11, 0)));

            // find correct direction in terms of reprojection error, just take the first 6 correspondences
            vector<double> error(2);
            vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Ts(2);
            for (int s = 0; s < 2; ++s) {
                error[s] = 0.0;
                Ts[s] = Eigen::Matrix4d::Identity();
                Ts[s].block<3, 3>(0, 0) = Rout;
                if (s == 0)
                    Ts[s].block<3, 1>(0, 3) = tout;
                else
                    Ts[s].block<3, 1>(0, 3) = -tout;
                Ts[s] = Ts[s].inverse().eval();
                for (int p = 0; p < 6; ++p) {
                    bearingVector_t v = Ts[s].block<3, 3>(0, 0) * points3v[p] + Ts[s].block<3, 1>(0, 3);
                    v = v / v.norm();
                    error[s] += (1.0 - v.transpose() * f[indices[p]]);
                }
            }
            if (error[0] < error[1])
                tout = Ts[0].block<3, 1>(0, 3);
            else
                tout = Ts[1].block<3, 1>(0, 3);
            Rout = Ts[0].block<3, 3>(0, 0);

        }

        //////////////////////////////////////
        // 5. gauss newton
        //////////////////////////////////////
        rodrigues_t omega = rot2rodrigues(Rout);
        Eigen::VectorXd minx(6);
        minx[0] = omega[0];
        minx[1] = omega[1];
        minx[2] = omega[2];
        minx[3] = tout[0];
        minx[4] = tout[1];
        minx[5] = tout[2];

        mlpnp_gn(minx, points3v, nullspaces, P, use_cov);

        Rout = rodrigues2rot(rodrigues_t(minx[0], minx[1], minx[2]));
        tout = translation_t(minx[3], minx[4], minx[5]);
        // result inverse as opengv uses this convention
        result.block<3, 3>(0, 0) = Rout;
        result.block<3, 1>(0, 3) = tout;
    }

    Eigen::Matrix3d MLPnPsolver::rodrigues2rot(const Eigen::Vector3d &omega) {
        rotation_t R = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d skewW;
        skewW << 0.0, -omega(2), omega(1),
                omega(2), 0.0, -omega(0),
                -omega(1), omega(0), 0.0;

        double omega_norm = omega.norm();

        if (omega_norm > std::numeric_limits<double>::epsilon())
            R = R + sin(omega_norm) / omega_norm * skewW
                + (1 - cos(omega_norm)) / (omega_norm * omega_norm) * (skewW * skewW);

        return R;
    }

    Eigen::Vector3d MLPnPsolver::rot2rodrigues(const Eigen::Matrix3d &R) {
        rodrigues_t omega;
        omega << 0.0, 0.0, 0.0;

        double trace = R.trace() - 1.0;
        double wnorm = acos(trace / 2.0);
        if (wnorm > std::numeric_limits<double>::epsilon())
        {
            omega[0] = (R(2, 1) - R(1, 2));
            omega[1] = (R(0, 2) - R(2, 0));
            omega[2] = (R(1, 0) - R(0, 1));
            double sc = wnorm / (2.0*sin(wnorm));
            omega *= sc;
        }
        return omega;
    }

    void MLPnPsolver::mlpnp_gn(Eigen::VectorXd &x, const points_t &pts, const std::vector<Eigen::MatrixXd> &nullspaces,
                               const Eigen::SparseMatrix<double> Kll, bool use_cov) {
        const int numObservations = pts.size();
        const int numUnknowns = 6;
        // check redundancy
        assert((2 * numObservations - numUnknowns) > 0);

        // =============
        // set all matrices up
        // =============

        Eigen::VectorXd r(2 * numObservations);
        Eigen::VectorXd rd(2 * numObservations);
        Eigen::MatrixXd Jac(2 * numObservations, numUnknowns);
        Eigen::VectorXd g(numUnknowns, 1);
        Eigen::VectorXd dx(numUnknowns, 1); // result vector

        Jac.setZero();
        r.setZero();
        dx.setZero();
        g.setZero();

        int it_cnt = 0;
        bool stop = false;
        const int maxIt = 5;
        double epsP = 1e-5;

        Eigen::MatrixXd JacTSKll;
        Eigen::MatrixXd A;
        // solve simple gradient descent
        while (it_cnt < maxIt && !stop) {
            mlpnp_residuals_and_jacs(x, pts,
                                     nullspaces,
                                     r, Jac, true);

            if (use_cov)
                JacTSKll = Jac.transpose() * Kll;
            else
                JacTSKll = Jac.transpose();

            A = JacTSKll * Jac;

            // get system matrix
            g = JacTSKll * r;

            // solve
            Eigen::LDLT<Eigen::MatrixXd> chol(A);
            dx = chol.solve(g);
            // this is to prevent the solution from falling into a wrong minimum
            // if the linear estimate is spurious
            if (dx.array().abs().maxCoeff() > 5.0 || dx.array().abs().minCoeff() > 1.0)
                break;
            // observation update
            Eigen::MatrixXd dl = Jac * dx;
            if (dl.array().abs().maxCoeff() < epsP) {
                stop = true;
                x = x - dx;
                break;
            } else
                x = x - dx;

            ++it_cnt;
        }//while
        // result
    }

    void MLPnPsolver::mlpnp_residuals_and_jacs(const Eigen::VectorXd &x, const points_t &pts,
                                               const std::vector<Eigen::MatrixXd> &nullspaces, Eigen::VectorXd &r,
                                               Eigen::MatrixXd &fjac, bool getJacs) {
        rodrigues_t w(x[0], x[1], x[2]);
        translation_t T(x[3], x[4], x[5]);

        rotation_t R = rodrigues2rot(w);
        int ii = 0;

        Eigen::MatrixXd jacs(2, 6);

        for (int i = 0; i < pts.size(); ++i)
        {
            Eigen::Vector3d ptCam = R*pts[i] + T;
            ptCam /= ptCam.norm();

            r[ii] = nullspaces[i].col(0).transpose()*ptCam;
            r[ii + 1] = nullspaces[i].col(1).transpose()*ptCam;
            if (getJacs)
            {
                // jacs
                mlpnpJacs(pts[i],
                          nullspaces[i].col(0), nullspaces[i].col(1),
                          w, T,
                          jacs);
                // r
                fjac.block<1, 6>(ii, 0) = jacs.row(0);

                // s
                fjac.block<1, 6>(ii + 1, 0) = jacs.row(1);

               /* // r
                fjac(ii, 0) = jacs(0, 0);
                fjac(ii, 1) = jacs(0, 1);
                fjac(ii, 2) = jacs(0, 2);

                fjac(ii, 3) = jacs(0, 3);
                fjac(ii, 4) = jacs(0, 4);
                fjac(ii, 5) = jacs(0, 5);
                // s
                fjac(ii + 1, 0) = jacs(1, 0);
                fjac(ii + 1, 1) = jacs(1, 1);
                fjac(ii + 1, 2) = jacs(1, 2);

                fjac(ii + 1, 3) = jacs(1, 3);
                fjac(ii + 1, 4) = jacs(1, 4);
                fjac(ii + 1, 5) = jacs(1, 5);*/

            }
            ii += 2;
        }
    }



    void MLPnPsolver::mlpnpJacs(const point_t& pt, const Eigen::Vector3d& nullspace_r,
                                const Eigen::Vector3d& nullspace_s, const rodrigues_t& w,
                                const translation_t& t, Eigen::MatrixXd& jacs) {
        const double r1 = nullspace_r[0], r2 = nullspace_r[1], r3 = nullspace_r[2];
        const double s1 = nullspace_s[0], s2 = nullspace_s[1], s3 = nullspace_s[2];
        const double X1 = pt[0], Y1 = pt[1], Z1 = pt[2];
        const double w1 = w[0], w2 = w[1], w3 = w[2];
        const double t1 = t[0], t2 = t[1], t3 = t[2];

        const double w1_sq = w1 * w1, w2_sq = w2 * w2, w3_sq = w3 * w3;

        const double w_norm_sq = w1_sq + w2_sq + w3_sq;
        const double w_norm = sqrt(w_norm_sq);
        const double sin_w_norm = sin(w_norm);
        const double cos_w_norm = cos(w_norm);
        const double one_minus_cos_w_norm = cos_w_norm - 1.0;
        const double inv_w_norm_sq = 1.0 / w_norm_sq;
        const double inv_w_norm = 1.0 / w_norm;
        const double inv_w_norm_cubed = 1.0 / (w_norm_sq * w_norm);

        const double w2_w3 = w2 * w3, w1_w3 = w1 * w3, w1_w2 =w1 * w2;
        const Eigen::Vector3d w_cross = (Eigen::Vector3d()<< w2_w3, w1_w3, w1_w2).finished();
        const double t13_t14_w_cross[2] = { one_minus_cos_w_norm * inv_w_norm_sq * w_cross[0], one_minus_cos_w_norm * inv_w_norm_sq * w_cross[2] };


        const Eigen::Matrix3d R = (Eigen::Matrix3d() <<
                cos_w_norm + w1_sq * one_minus_cos_w_norm, t13_t14_w_cross[1] + w1_w2 * one_minus_cos_w_norm, sin_w_norm * inv_w_norm * w_cross[1] + w1_w3 * one_minus_cos_w_norm,
                t13_t14_w_cross[1] + w1_w2 * one_minus_cos_w_norm, cos_w_norm + w2_sq * one_minus_cos_w_norm, t13_t14_w_cross[0] + w2_w3 * one_minus_cos_w_norm,
                sin_w_norm * inv_w_norm * w_cross[1] + w1_w3 * one_minus_cos_w_norm, one_minus_cos_w_norm * inv_w_norm_sq * w_cross[0] + w2_w3 * one_minus_cos_w_norm, cos_w_norm + w3_sq * one_minus_cos_w_norm).finished();

        const double t15 = t1 - Y1 * R(0,1) + Z1 * R(0,2) + X1 * R(0,0);
        const double t16 = t2 - Y1 * R(1,1) + Z1 * R(1,2) + X1 * R(1,0);
        const double t17 = t3 - Y1 * R(2,1) + Z1 * R(2,2) + X1 * R(2,0);

        const double dtdw1 = -2.0 * sin_w_norm * inv_w_norm_cubed * (w1 * t15 + w2 * t16 + w3 * t17);
        const double dtdw2 = (w_norm_sq * sin_w_norm - 2.0 * w_norm * one_minus_cos_w_norm) * inv_w_norm_cubed;

        const double w_norm_sin_w_norm =w_norm * sin_w_norm, w_norm_sin_w_norm_inv_w_norm = w_norm_sin_w_norm * inv_w_norm;
        const double dtdw2_w3= dtdw2 * w3, dtdw1_w1 = dtdw1 * w1;


        const Eigen::Matrix3d dR_dw1 = (Eigen::Matrix3d() <<
                -2.0 * w1 * dtdw2, dtdw1 * w3, -dtdw1 * w2,
                dtdw1 * w3, -dtdw2 * w1, -w_norm_sin_w_norm_inv_w_norm,
                -dtdw1 * w2, w_norm * sin_w_norm * inv_w_norm, -dtdw2 * w1).finished();

        const Eigen::Matrix3d dR_dw2 = (Eigen::Matrix3d() <<
                -dtdw2 * w2, -w_norm_sin_w_norm_inv_w_norm, dtdw1 * w3,
                -w_norm_sin_w_norm_inv_w_norm, -2.0 * w2 * dtdw2, -dtdw1_w1,
                dtdw1 * w3, -dtdw1_w1, -dtdw2 * w2).finished();

        const Eigen::Matrix3d dR_dw3 = (Eigen::Matrix3d() <<
                -dtdw2_w3, dtdw1 * w2, -w_norm_sin_w_norm_inv_w_norm,
                dtdw1 * w2, -dtdw2_w3, dtdw1_w1,
                -w_norm_sin_w_norm_inv_w_norm, dtdw1_w1, -2.0 * w3 * dtdw2).finished();

        const double dtdX1 = r1 * R(0,0) + s1 * R(1,0) + R(2,0);
        const double dtdY1 = r1 * R(0,1) + s1 * R(1,1) + R(2,1);
        const double dtdZ1 = r1 * R(0,2) + s1 * R(1,2) + R(2,2);

        jacs(0, 0) = r1 * dR_dw1(0,0) + s1 * dR_dw1(1,0) + dR_dw1(2,0);
        jacs(0, 1) = r1 * dR_dw2(0,0) + s1 * dR_dw2(1,0) + dR_dw2(2,0);
        jacs(0, 2) = r1 * dR_dw3(0,0) + s1 * dR_dw3(1,0) + dR_dw3(2,0);

        jacs(1, 0) = r2 * dR_dw1(0,1) + s2 * dR_dw1(1,1) + dR_dw1(2,1);
        jacs(1, 1) = r2 * dR_dw2(0,1) + s2 * dR_dw2(1,1) + dR_dw2(2,1);
        jacs(1, 2) = r2 * dR_dw3(0,1) + s2 * dR_dw3(1,1) + dR_dw3(2,1);

        jacs(2, 0) = r3 * dR_dw1(0, 2) + s3 * dR_dw1(1, 2) + dR_dw1(2, 2);
        jacs(2, 1) = r3 * dR_dw2(0, 2) + s3 * dR_dw2(1, 2) + dR_dw2(2, 2);
        jacs(2, 2) = r3 * dR_dw3(0, 2) + s3 * dR_dw3(1, 2) + dR_dw3(2, 2);

        jacs(0, 3) = dtdX1;
        jacs(0, 4) = dtdY1;
        jacs(0, 5) = dtdZ1;

        jacs(1, 3) = dtdY1;
        jacs(1, 4) = r2 * R(0,0) + s2 * R(1,0) + R(2,0);
        jacs(1, 5) = r2 * R(0,2) + s2 * R(1,2) + R(2,2);
    }



}//End namespace ORB_SLAM2