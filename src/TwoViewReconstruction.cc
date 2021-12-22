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

#include "TwoViewReconstruction.h"

#include "Converter.h"
#include "GeometricTools.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include<thread>


using namespace std;
namespace ORB_SLAM3
{
    TwoViewReconstruction::TwoViewReconstruction(const Eigen::Matrix3f& k, float sigma, int iterations)
    {
        mK = k;

        mSigma = sigma;
        mSigma2 = sigma*sigma;
        mMaxIterations = iterations;
    }

    bool TwoViewReconstruction::Reconstruct(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const vector<int> &vMatches12,
                                             Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
    {
        mvKeys1.clear();
        mvKeys2.clear();

        mvKeys1 = vKeys1;
        mvKeys2 = vKeys2;

        // Fill structures with current keypoints and matches with reference frame
        // Reference Frame: 1, Current Frame: 2
        mvMatches12.clear();
        mvMatches12.reserve(mvKeys2.size());
        mvbMatched1.resize(mvKeys1.size());
        for(size_t i=0, iend=vMatches12.size();i<iend; i++)
        {
            if(vMatches12[i]>=0)
            {
                mvMatches12.push_back(make_pair(i,vMatches12[i]));
                mvbMatched1[i]=true;
            }
            else
                mvbMatched1[i]=false;
        }

        const int N = mvMatches12.size();

        // Indices for minimum set selection
        vector<size_t> vAllIndices;
        vAllIndices.reserve(N);
        vector<size_t> vAvailableIndices;

        for(int i=0; i<N; i++)
        {
            vAllIndices.push_back(i);
        }

        // Generate sets of 8 points for each RANSAC iteration
        mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

        DUtils::Random::SeedRandOnce(0);

        for(int it=0; it<mMaxIterations; it++)
        {
            vAvailableIndices = vAllIndices;

            // Select a minimum set
            for(size_t j=0; j<8; j++)
            {
                int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
                int idx = vAvailableIndices[randi];

                mvSets[it][j] = idx;

                vAvailableIndices[randi] = vAvailableIndices.back();
                vAvailableIndices.pop_back();
            }
        }

        // Launch threads to compute in parallel a fundamental matrix and a homography
        vector<bool> vbMatchesInliersH, vbMatchesInliersF;
        float SH, SF;
        Eigen::Matrix3f H, F;

        thread threadH(&TwoViewReconstruction::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
        thread threadF(&TwoViewReconstruction::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

        // Wait until both threads have finished
        threadH.join();
        threadF.join();

        // Compute ratio of scores
        if(SH+SF == 0.f) return false;
        float RH = SH/(SH+SF);

        float minParallax = 1.0;

        // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
        if(RH>0.50) // if(RH>0.40)
        {
            //cout << "Initialization from Homography" << endl;
            return ReconstructH(vbMatchesInliersH,H, mK,T21,vP3D,vbTriangulated,minParallax,50);
        }
        else //if(pF_HF>0.6)
        {
            //cout << "Initialization from Fundamental" << endl;
            return ReconstructF(vbMatchesInliersF,F,mK,T21,vP3D,vbTriangulated,minParallax,50);
        }
    }

    void TwoViewReconstruction::FindHomography(vector<bool> &vbMatchesInliers, float &score, Eigen::Matrix3f &H21)
    {
        // Number of putative matches
        const int N = mvMatches12.size();

        // Normalize coordinates
        vector<cv::Point2f> vPn1, vPn2;
        Eigen::Matrix3f T1, T2;
        Normalize(mvKeys1,vPn1, T1);
        Normalize(mvKeys2,vPn2, T2);
        Eigen::Matrix3f T2inv = T2.inverse();

        // Best Results variables
        score = 0.0;
        vbMatchesInliers = vector<bool>(N,false);

        // Iteration variables
        vector<cv::Point2f> vPn1i(8);
        vector<cv::Point2f> vPn2i(8);
        Eigen::Matrix3f H21i, H12i;
        vector<bool> vbCurrentInliers(N,false);
        float currentScore;

        // Perform all RANSAC iterations and save the solution with highest score
        for(int it=0; it<mMaxIterations; it++)
        {
            // Select a minimum set
            for(size_t j=0; j<8; j++)
            {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            Eigen::Matrix3f Hn = ComputeH21(vPn1i,vPn2i);
            H21i = T2inv * Hn * T1;
            H12i = H21i.inverse();

            currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

            if(currentScore>score)
            {
                H21 = H21i;
                vbMatchesInliers = vbCurrentInliers;
                score = currentScore;
            }
        }
    }


    void TwoViewReconstruction::FindFundamental(vector<bool> &vbMatchesInliers, float &score, Eigen::Matrix3f &F21)
    {
        // Number of putative matches
        const int N = vbMatchesInliers.size();

        // Normalize coordinates
        vector<cv::Point2f> vPn1, vPn2;
        Eigen::Matrix3f T1, T2;
        Normalize(mvKeys1,vPn1, T1);
        Normalize(mvKeys2,vPn2, T2);
        Eigen::Matrix3f T2t = T2.transpose();

        // Best Results variables
        score = 0.0;
        vbMatchesInliers = vector<bool>(N,false);

        // Iteration variables
        vector<cv::Point2f> vPn1i(8);
        vector<cv::Point2f> vPn2i(8);
        Eigen::Matrix3f F21i;
        vector<bool> vbCurrentInliers(N,false);
        float currentScore;

        // Perform all RANSAC iterations and save the solution with highest score
        for(int it=0; it<mMaxIterations; it++)
        {
            // Select a minimum set
            for(int j=0; j<8; j++)
            {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            Eigen::Matrix3f Fn = ComputeF21(vPn1i,vPn2i);

            F21i = T2t * Fn * T1;

            currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

            if(currentScore>score)
            {
                F21 = F21i;
                vbMatchesInliers = vbCurrentInliers;
                score = currentScore;
            }
        }
    }

    Eigen::Matrix3f TwoViewReconstruction::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
    {
        const int N = vP1.size();

        Eigen::MatrixXf A(2*N, 9);

        for(int i=0; i<N; i++)
        {
            const float u1 = vP1[i].x;
            const float v1 = vP1[i].y;
            const float u2 = vP2[i].x;
            const float v2 = vP2[i].y;

            A(2*i,0) = 0.0;
            A(2*i,1) = 0.0;
            A(2*i,2) = 0.0;
            A(2*i,3) = -u1;
            A(2*i,4) = -v1;
            A(2*i,5) = -1;
            A(2*i,6) = v2*u1;
            A(2*i,7) = v2*v1;
            A(2*i,8) = v2;

            A(2*i+1,0) = u1;
            A(2*i+1,1) = v1;
            A(2*i+1,2) = 1;
            A(2*i+1,3) = 0.0;
            A(2*i+1,4) = 0.0;
            A(2*i+1,5) = 0.0;
            A(2*i+1,6) = -u2*u1;
            A(2*i+1,7) = -u2*v1;
            A(2*i+1,8) = -u2;

        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);

        Eigen::Matrix<float,3,3,Eigen::RowMajor> H(svd.matrixV().col(8).data());

        return H;
    }

    Eigen::Matrix3f TwoViewReconstruction::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
    {
        const int N = vP1.size();

        Eigen::MatrixXf A(N, 9);

        for(int i=0; i<N; i++)
        {
            const float u1 = vP1[i].x;
            const float v1 = vP1[i].y;
            const float u2 = vP2[i].x;
            const float v2 = vP2[i].y;

            A(i,0) = u2*u1;
            A(i,1) = u2*v1;
            A(i,2) = u2;
            A(i,3) = v2*u1;
            A(i,4) = v2*v1;
            A(i,5) = v2;
            A(i,6) = u1;
            A(i,7) = v1;
            A(i,8) = 1;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix<float,3,3,Eigen::RowMajor> Fpre(svd.matrixV().col(8).data());

        Eigen::JacobiSVD<Eigen::Matrix3f> svd2(Fpre, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Vector3f w = svd2.singularValues();
        w(2) = 0;

        return svd2.matrixU() * Eigen::DiagonalMatrix<float,3>(w) * svd2.matrixV().transpose();
    }

    float TwoViewReconstruction::CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12, vector<bool> &vbMatchesInliers, float sigma)
    {
        const int N = mvMatches12.size();

        const float h11 = H21(0,0);
        const float h12 = H21(0,1);
        const float h13 = H21(0,2);
        const float h21 = H21(1,0);
        const float h22 = H21(1,1);
        const float h23 = H21(1,2);
        const float h31 = H21(2,0);
        const float h32 = H21(2,1);
        const float h33 = H21(2,2);

        const float h11inv = H12(0,0);
        const float h12inv = H12(0,1);
        const float h13inv = H12(0,2);
        const float h21inv = H12(1,0);
        const float h22inv = H12(1,1);
        const float h23inv = H12(1,2);
        const float h31inv = H12(2,0);
        const float h32inv = H12(2,1);
        const float h33inv = H12(2,2);

        vbMatchesInliers.resize(N);

        float score = 0;

        const float th = 5.991;

        const float invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<N; i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // Reprojection error in first image
            // x2in1 = H12*x2

            const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
            const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
            const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

            const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

            const float chiSquare1 = squareDist1*invSigmaSquare;

            if(chiSquare1>th)
                bIn = false;
            else
                score += th - chiSquare1;

            // Reprojection error in second image
            // x1in2 = H21*x1

            const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
            const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
            const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

            const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

            const float chiSquare2 = squareDist2*invSigmaSquare;

            if(chiSquare2>th)
                bIn = false;
            else
                score += th - chiSquare2;

            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }

        return score;
    }

    float TwoViewReconstruction::CheckFundamental(const Eigen::Matrix3f &F21, vector<bool> &vbMatchesInliers, float sigma)
    {
        const int N = mvMatches12.size();

        const float f11 = F21(0,0);
        const float f12 = F21(0,1);
        const float f13 = F21(0,2);
        const float f21 = F21(1,0);
        const float f22 = F21(1,1);
        const float f23 = F21(1,2);
        const float f31 = F21(2,0);
        const float f32 = F21(2,1);
        const float f33 = F21(2,2);

        vbMatchesInliers.resize(N);

        float score = 0;

        const float th = 3.841;
        const float thScore = 5.991;

        const float invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<N; i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // Reprojection error in second image
            // l2=F21x1=(a2,b2,c2)

            const float a2 = f11*u1+f12*v1+f13;
            const float b2 = f21*u1+f22*v1+f23;
            const float c2 = f31*u1+f32*v1+f33;

            const float num2 = a2*u2+b2*v2+c2;

            const float squareDist1 = num2*num2/(a2*a2+b2*b2);

            const float chiSquare1 = squareDist1*invSigmaSquare;

            if(chiSquare1>th)
                bIn = false;
            else
                score += thScore - chiSquare1;

            // Reprojection error in second image
            // l1 =x2tF21=(a1,b1,c1)

            const float a1 = f11*u2+f21*v2+f31;
            const float b1 = f12*u2+f22*v2+f32;
            const float c1 = f13*u2+f23*v2+f33;

            const float num1 = a1*u1+b1*v1+c1;

            const float squareDist2 = num1*num1/(a1*a1+b1*b1);

            const float chiSquare2 = squareDist2*invSigmaSquare;

            if(chiSquare2>th)
                bIn = false;
            else
                score += thScore - chiSquare2;

            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }

        return score;
    }

    bool TwoViewReconstruction::ReconstructF(vector<bool> &vbMatchesInliers, Eigen::Matrix3f &F21, Eigen::Matrix3f &K,
                                             Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
    {
        int N=0;
        for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
            if(vbMatchesInliers[i])
                N++;

        // Compute Essential Matrix from Fundamental Matrix
        Eigen::Matrix3f E21 = K.transpose() * F21 * K;

        Eigen::Matrix3f R1, R2;
        Eigen::Vector3f t;

        // Recover the 4 motion hypotheses
        DecomposeE(E21,R1,R2,t);

        Eigen::Vector3f t1 = t;
        Eigen::Vector3f t2 = -t;

        // Reconstruct with the 4 hyphoteses and check
        vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
        vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
        float parallax1,parallax2, parallax3, parallax4;

        int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
        int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
        int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
        int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

        int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

        int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

        int nsimilar = 0;
        if(nGood1>0.7*maxGood)
            nsimilar++;
        if(nGood2>0.7*maxGood)
            nsimilar++;
        if(nGood3>0.7*maxGood)
            nsimilar++;
        if(nGood4>0.7*maxGood)
            nsimilar++;

        // If there is not a clear winner or not enough triangulated points reject initialization
        if(maxGood<nMinGood || nsimilar>1)
        {
            return false;
        }

        // If best reconstruction has enough parallax initialize
        if(maxGood==nGood1)
        {
            if(parallax1>minParallax)
            {
                vP3D = vP3D1;
                vbTriangulated = vbTriangulated1;

                T21 = Sophus::SE3f(R1, t1);
                return true;
            }
        }else if(maxGood==nGood2)
        {
            if(parallax2>minParallax)
            {
                vP3D = vP3D2;
                vbTriangulated = vbTriangulated2;

                T21 = Sophus::SE3f(R2, t1);
                return true;
            }
        }else if(maxGood==nGood3)
        {
            if(parallax3>minParallax)
            {
                vP3D = vP3D3;
                vbTriangulated = vbTriangulated3;

                T21 = Sophus::SE3f(R1, t2);
                return true;
            }
        }else if(maxGood==nGood4)
        {
            if(parallax4>minParallax)
            {
                vP3D = vP3D4;
                vbTriangulated = vbTriangulated4;

                T21 = Sophus::SE3f(R2, t2);
                return true;
            }
        }

        return false;
    }

    bool TwoViewReconstruction::ReconstructH(vector<bool> &vbMatchesInliers, Eigen::Matrix3f &H21, Eigen::Matrix3f &K,
                                             Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
    {
        int N=0;
        for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
            if(vbMatchesInliers[i])
                N++;

        // We recover 8 motion hypotheses using the method of Faugeras et al.
        // Motion and structure from motion in a piecewise planar environment.
        // International Journal of Pattern Recognition and Artificial Intelligence, 1988
        Eigen::Matrix3f invK = K.inverse();
        Eigen::Matrix3f A = invK * H21 * K;

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f V = svd.matrixV();
        Eigen::Matrix3f Vt = V.transpose();
        Eigen::Vector3f w = svd.singularValues();

        float s = U.determinant() * Vt.determinant();

        float d1 = w(0);
        float d2 = w(1);
        float d3 = w(2);

        if(d1/d2<1.00001 || d2/d3<1.00001)
        {
            return false;
        }

        vector<Eigen::Matrix3f> vR;
        vector<Eigen::Vector3f> vt, vn;
        vR.reserve(8);
        vt.reserve(8);
        vn.reserve(8);

        //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
        float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
        float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
        float x1[] = {aux1,aux1,-aux1,-aux1};
        float x3[] = {aux3,-aux3,aux3,-aux3};

        //case d'=d2
        float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

        float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
        float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

        for(int i=0; i<4; i++)
        {
            Eigen::Matrix3f Rp;
            Rp.setZero();
            Rp(0,0) = ctheta;
            Rp(0,2) = -stheta[i];
            Rp(1,1) = 1.f;
            Rp(2,0) = stheta[i];
            Rp(2,2) = ctheta;

            Eigen::Matrix3f R = s*U*Rp*Vt;
            vR.push_back(R);

            Eigen::Vector3f tp;
            tp(0) = x1[i];
            tp(1) = 0;
            tp(2) = -x3[i];
            tp *= d1-d3;

            Eigen::Vector3f t = U*tp;
            vt.push_back(t / t.norm());

            Eigen::Vector3f np;
            np(0) = x1[i];
            np(1) = 0;
            np(2) = x3[i];

            Eigen::Vector3f n = V*np;
            if(n(2) < 0)
                n = -n;
            vn.push_back(n);
        }

        //case d'=-d2
        float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

        float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
        float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

        for(int i=0; i<4; i++)
        {
            Eigen::Matrix3f Rp;
            Rp.setZero();
            Rp(0,0) = cphi;
            Rp(0,2) = sphi[i];
            Rp(1,1) = -1;
            Rp(2,0) = sphi[i];
            Rp(2,2) = -cphi;

            Eigen::Matrix3f R = s*U*Rp*Vt;
            vR.push_back(R);

            Eigen::Vector3f tp;
            tp(0) = x1[i];
            tp(1) = 0;
            tp(2) = x3[i];
            tp *= d1+d3;

            Eigen::Vector3f t = U*tp;
            vt.push_back(t / t.norm());

            Eigen::Vector3f np;
            np(0) = x1[i];
            np(1) = 0;
            np(2) = x3[i];

            Eigen::Vector3f n = V*np;
            if(n(2) < 0)
                n = -n;
            vn.push_back(n);
        }


        int bestGood = 0;
        int secondBestGood = 0;
        int bestSolutionIdx = -1;
        float bestParallax = -1;
        vector<cv::Point3f> bestP3D;
        vector<bool> bestTriangulated;

        // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
        // We reconstruct all hypotheses and check in terms of triangulated points and parallax
        for(size_t i=0; i<8; i++)
        {
            float parallaxi;
            vector<cv::Point3f> vP3Di;
            vector<bool> vbTriangulatedi;
            int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

            if(nGood>bestGood)
            {
                secondBestGood = bestGood;
                bestGood = nGood;
                bestSolutionIdx = i;
                bestParallax = parallaxi;
                bestP3D = vP3Di;
                bestTriangulated = vbTriangulatedi;
            }
            else if(nGood>secondBestGood)
            {
                secondBestGood = nGood;
            }
        }


        if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
        {
            T21 = Sophus::SE3f(vR[bestSolutionIdx], vt[bestSolutionIdx]);
            vbTriangulated = bestTriangulated;

            return true;
        }

        return false;
    }


    void TwoViewReconstruction::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, Eigen::Matrix3f &T)
    {
        float meanX = 0;
        float meanY = 0;
        const int N = vKeys.size();

        vNormalizedPoints.resize(N);

        for(int i=0; i<N; i++)
        {
            meanX += vKeys[i].pt.x;
            meanY += vKeys[i].pt.y;
        }

        meanX = meanX/N;
        meanY = meanY/N;

        float meanDevX = 0;
        float meanDevY = 0;

        for(int i=0; i<N; i++)
        {
            vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
            vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

            meanDevX += fabs(vNormalizedPoints[i].x);
            meanDevY += fabs(vNormalizedPoints[i].y);
        }

        meanDevX = meanDevX/N;
        meanDevY = meanDevY/N;

        float sX = 1.0/meanDevX;
        float sY = 1.0/meanDevY;

        for(int i=0; i<N; i++)
        {
            vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
            vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
        }

        T.setZero();
        T(0,0) = sX;
        T(1,1) = sY;
        T(0,2) = -meanX*sX;
        T(1,2) = -meanY*sY;
        T(2,2) = 1.f;
    }

    int TwoViewReconstruction::CheckRT(const Eigen::Matrix3f &R, const Eigen::Vector3f &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                                       const Eigen::Matrix3f &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
    {
        // Calibration parameters
        const float fx = K(0,0);
        const float fy = K(1,1);
        const float cx = K(0,2);
        const float cy = K(1,2);

        vbGood = vector<bool>(vKeys1.size(),false);
        vP3D.resize(vKeys1.size());

        vector<float> vCosParallax;
        vCosParallax.reserve(vKeys1.size());

        // Camera 1 Projection Matrix K[I|0]
        Eigen::Matrix<float,3,4> P1;
        P1.setZero();
        P1.block<3,3>(0,0) = K;

        Eigen::Vector3f O1;
        O1.setZero();

        // Camera 2 Projection Matrix K[R|t]
        Eigen::Matrix<float,3,4> P2;
        P2.block<3,3>(0,0) = R;
        P2.block<3,1>(0,3) = t;
        P2 = K * P2;

        Eigen::Vector3f O2 = -R.transpose() * t;

        int nGood=0;

        for(size_t i=0, iend=vMatches12.size();i<iend;i++)
        {
            if(!vbMatchesInliers[i])
                continue;

            const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
            const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

            Eigen::Vector3f p3dC1;
            Eigen::Vector3f x_p1(kp1.pt.x, kp1.pt.y, 1);
            Eigen::Vector3f x_p2(kp2.pt.x, kp2.pt.y, 1);

            GeometricTools::Triangulate(x_p1, x_p2, P1, P2, p3dC1);


            if(!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
            {
                vbGood[vMatches12[i].first]=false;
                continue;
            }

            // Check parallax
            Eigen::Vector3f normal1 = p3dC1 - O1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = p3dC1 - O2;
            float dist2 = normal2.norm();

            float cosParallax = normal1.dot(normal2) / (dist1*dist2);

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            if(p3dC1(2)<=0 && cosParallax<0.99998)
                continue;

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            Eigen::Vector3f p3dC2 = R * p3dC1 + t;

            if(p3dC2(2)<=0 && cosParallax<0.99998)
                continue;

            // Check reprojection error in first image
            float im1x, im1y;
            float invZ1 = 1.0/p3dC1(2);
            im1x = fx*p3dC1(0)*invZ1+cx;
            im1y = fy*p3dC1(1)*invZ1+cy;

            float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

            if(squareError1>th2)
                continue;

            // Check reprojection error in second image
            float im2x, im2y;
            float invZ2 = 1.0/p3dC2(2);
            im2x = fx*p3dC2(0)*invZ2+cx;
            im2y = fy*p3dC2(1)*invZ2+cy;

            float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

            if(squareError2>th2)
                continue;

            vCosParallax.push_back(cosParallax);
            vP3D[vMatches12[i].first] = cv::Point3f(p3dC1(0), p3dC1(1), p3dC1(2));
            nGood++;

            if(cosParallax<0.99998)
                vbGood[vMatches12[i].first]=true;
        }

        if(nGood>0)
        {
            sort(vCosParallax.begin(),vCosParallax.end());

            size_t idx = min(50,int(vCosParallax.size()-1));
            parallax = acos(vCosParallax[idx])*180/CV_PI;
        }
        else
            parallax=0;

        return nGood;
    }

    void TwoViewReconstruction::DecomposeE(const Eigen::Matrix3f &E, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2, Eigen::Vector3f &t)
    {

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f Vt = svd.matrixV().transpose();

        t = U.col(2);
        t = t / t.norm();

        Eigen::Matrix3f W;
        W.setZero();
        W(0,1) = -1;
        W(1,0) = 1;
        W(2,2) = 1;

        R1 = U * W * Vt;
        if(R1.determinant() < 0)
            R1 = -R1;

        R2 = U * W.transpose() * Vt;
        if(R2.determinant() < 0)
            R2 = -R2;
    }

} //namespace ORB_SLAM
