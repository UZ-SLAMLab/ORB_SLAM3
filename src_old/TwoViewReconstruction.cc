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

#include <thread>

using namespace std;
namespace ORB_SLAM3
{
TwoViewReconstruction::TwoViewReconstruction(const Eigen::Matrix3f &k, float sigma, int iterations)
{
    mK = k;

    mSigma = sigma;
    mSigma2 = sigma * sigma;
    mMaxIterations = iterations;
}

/** 
 * @brief 单目初始化重要的环节，先获得rt在通过三角化恢复3d点坐标
 * @param vKeys1 第一帧的关键点
 * @param vKeys2 第二帧的关键点
 * @param vMatches12 匹配关系，长度与vKeys1一样，对应位置存放vKeys2中关键点的下标
 * @param T21 顾名思义
 * @param vP3D 恢复出的三维点
 * @param vbTriangulated 是否三角化成功，用于统计匹配点数量
 */
bool TwoViewReconstruction::Reconstruct(
    const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2, const vector<int> &vMatches12,
    Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    // 1. 准备工作，提取匹配关系及准备RANSAC
    mvKeys1.clear();
    mvKeys2.clear();

    mvKeys1 = vKeys1;
    mvKeys2 = vKeys2;

    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    // 填写mvMatches12，里面存放的是vKeys1，vKeys2匹配点的索引
    // 存放匹配点的id
    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    // 长度与vKeys1，表示对应位置的vKeys1中的点是否有匹配关系
    mvbMatched1.resize(mvKeys1.size());
    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (vMatches12[i] >= 0)
        {
            mvMatches12.push_back(make_pair(i, vMatches12[i]));
            mvbMatched1[i] = true;
        }
        else
            mvbMatched1[i] = false;
    }

    const int N = mvMatches12.size();

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    // 使用vAllIndices为了保证8个点选不到同一个点
    for (int i = 0; i < N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    // 默认200次
    mvSets = vector<vector<size_t>>(mMaxIterations, vector<size_t>(8, 0));

    DUtils::Random::SeedRandOnce(0);

    // 2. 先遍历把200次先取好
    for (int it = 0; it < mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for (size_t j = 0; j < 8; j++)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);
            int idx = vAvailableIndices[randi];  // 这句不多余，防止重复选择

            mvSets[it][j] = idx;

            // 保证选不到同一个点，这么做的话可以删去vAvailableIndices已选点的索引
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    Eigen::Matrix3f H, F;

    // 3. 双线程分别计算
    // 加ref为了提供引用
    thread threadH(&TwoViewReconstruction::FindHomography, this, ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&TwoViewReconstruction::FindFundamental, this, ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    if (SH + SF == 0.f)
        return false;
    float RH = SH / (SH + SF);

    float minParallax = 1.0;

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    // 4. 看哪个分高用哪个，分别是通过H重建与通过F重建
    // ORBSLAM2这里的值是0.4， TOSEE
    if (RH > 0.50) // if(RH>0.40)
    {
        // cout << "Initialization from Homography" << endl;
        return ReconstructH(vbMatchesInliersH, H, mK, T21, vP3D, vbTriangulated, minParallax, 50);
    }
    else // if(pF_HF>0.6)
    {
        // cout << "Initialization from Fundamental" << endl;
        return ReconstructF(vbMatchesInliersF, F, mK, T21, vP3D, vbTriangulated, minParallax, 50);
    }
}

/** 
 * @brief 计算H矩阵，同时计算得分与内点
 * @param vbMatchesInliers 经过H矩阵验证，是否为内点，大小为mvMatches12
 * @param score 得分
 * @param H21 1到2的H矩阵
 */
void TwoViewReconstruction::FindHomography(vector<bool> &vbMatchesInliers, float &score, Eigen::Matrix3f &H21)
{
    // Number of putative matches
    // 匹配成功的个数
    const int N = mvMatches12.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    Eigen::Matrix3f T1, T2;
    // 像素坐标标准化，先去均值，再除均长
    Normalize(mvKeys1, vPn1, T1);
    Normalize(mvKeys2, vPn2, T2);
    Eigen::Matrix3f T2inv = T2.inverse();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N, false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    Eigen::Matrix3f H21i, H12i;
    vector<bool> vbCurrentInliers(N, false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    // 计算归一化后的H矩阵 p2 = H21*p1
    for (int it = 0; it < mMaxIterations; it++)
    {
        // Select a minimum set
        for (size_t j = 0; j < 8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }
        // 计算标准化后的H矩阵
        Eigen::Matrix3f Hn = ComputeH21(vPn1i, vPn2i);
        // 恢复正常H
        H21i = T2inv * Hn * T1;
        H12i = H21i.inverse();

        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);  // mSigma默认为1

        if (currentScore > score)
        {
            H21 = H21i;
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

/** 
 * @brief 计算F矩阵，同时计算得分与内点
 * @param vbMatchesInliers 经过F矩阵验证，是否为内点，大小为mvMatches12
 * @param score 得分
 * @param F21 1到2的F矩阵
 */
void TwoViewReconstruction::FindFundamental(vector<bool> &vbMatchesInliers, float &score, Eigen::Matrix3f &F21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    Eigen::Matrix3f T1, T2;
    Normalize(mvKeys1, vPn1, T1);
    Normalize(mvKeys2, vPn2, T2);
    Eigen::Matrix3f T2t = T2.transpose();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N, false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    Eigen::Matrix3f F21i;
    vector<bool> vbCurrentInliers(N, false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int it = 0; it < mMaxIterations; it++)
    {
        // Select a minimum set
        for (int j = 0; j < 8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        Eigen::Matrix3f Fn = ComputeF21(vPn1i, vPn2i);

        // FN得到的是基于归一化坐标的F矩阵，必须乘上归一化过程矩阵才是最后的基于像素坐标的F
        F21i = T2t * Fn * T1;

        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

        if (currentScore > score)
        {
            F21 = F21i;
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

/**
 * @brief 从特征点匹配求homography（normalized DLT）
 * |x'|     | h1 h2 h3 ||x|
 * |y'| = a | h4 h5 h6 ||y|  简写: x' = a H x, a为一个尺度因子
 * |1 |     | h7 h8 h9 ||1|
 * 使用DLT(direct linear tranform)求解该模型
 * x' = a H x
 * ---> (x') 叉乘 (H x)  = 0
 * ---> Ah = 0
 * A = | 0  0  0 -x -y -1 xy' yy' y'|  h = | h1 h2 h3 h4 h5 h6 h7 h8 h9 |
 *     |-x -y -1  0  0  0 xx' yx' x'|
 * 通过SVD求解Ah = 0，A'A最小特征值对应的特征向量即为解
 * @param  vP1 归一化后的点, in reference frame
 * @param  vP2 归一化后的点, in current frame
 * @return     单应矩阵
 * @see        Multiple View Geometry in Computer Vision - Algorithm 4.2 p109
 */
Eigen::Matrix3f TwoViewReconstruction::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    Eigen::MatrixXf A(2 * N, 9);

    for (int i = 0; i < N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A(2 * i, 0) = 0.0;
        A(2 * i, 1) = 0.0;
        A(2 * i, 2) = 0.0;
        A(2 * i, 3) = -u1;
        A(2 * i, 4) = -v1;
        A(2 * i, 5) = -1;
        A(2 * i, 6) = v2 * u1;
        A(2 * i, 7) = v2 * v1;
        A(2 * i, 8) = v2;

        A(2 * i + 1, 0) = u1;
        A(2 * i + 1, 1) = v1;
        A(2 * i + 1, 2) = 1;
        A(2 * i + 1, 3) = 0.0;
        A(2 * i + 1, 4) = 0.0;
        A(2 * i + 1, 5) = 0.0;
        A(2 * i + 1, 6) = -u2 * u1;
        A(2 * i + 1, 7) = -u2 * v1;
        A(2 * i + 1, 8) = -u2;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> H(svd.matrixV().col(8).data());

    return H;
}

/**
 * @brief 从特征点匹配求fundamental matrix（normalized 8点法）
 * x'Fx = 0 整理可得：Af = 0
 * A = | x'x x'y x' y'x y'y y' x y 1 |, f = | f1 f2 f3 f4 f5 f6 f7 f8 f9 |
 * 通过SVD求解Af = 0，A'A最小特征值对应的特征向量即为解
 * @param  vP1 归一化后的点, in reference frame
 * @param  vP2 归一化后的点, in current frame
 * @return     基础矩阵
 * @see        Multiple View Geometry in Computer Vision - Algorithm 11.1 p282 (中文版 p191)
 */
Eigen::Matrix3f TwoViewReconstruction::ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    Eigen::MatrixXf A(N, 9);

    for (int i = 0; i < N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A(i, 0) = u2 * u1;
        A(i, 1) = u2 * v1;
        A(i, 2) = u2;
        A(i, 3) = v2 * u1;
        A(i, 4) = v2 * v1;
        A(i, 5) = v2;
        A(i, 6) = u1;
        A(i, 7) = v1;
        A(i, 8) = 1;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Fpre(svd.matrixV().col(8).data());

    Eigen::JacobiSVD<Eigen::Matrix3f> svd2(Fpre, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Vector3f w = svd2.singularValues();
    // 这里注意计算完要强制让第三个奇异值为0
    w(2) = 0;

    return svd2.matrixU() * Eigen::DiagonalMatrix<float, 3>(w) * svd2.matrixV().transpose();
}

/** 
 * @brief 检查结果
 * @param H21 顾名思义
 * @param H12 顾名思义
 * @param vbMatchesInliers 匹配是否合法，大小为mvMatches12
 * @param sigma 默认为1
 */
float TwoViewReconstruction::CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float h11 = H21(0, 0);
    const float h12 = H21(0, 1);
    const float h13 = H21(0, 2);
    const float h21 = H21(1, 0);
    const float h22 = H21(1, 1);
    const float h23 = H21(1, 2);
    const float h31 = H21(2, 0);
    const float h32 = H21(2, 1);
    const float h33 = H21(2, 2);

    const float h11inv = H12(0, 0);
    const float h12inv = H12(0, 1);
    const float h13inv = H12(0, 2);
    const float h21inv = H12(1, 0);
    const float h22inv = H12(1, 1);
    const float h23inv = H12(1, 2);
    const float h31inv = H12(2, 0);
    const float h32inv = H12(2, 1);
    const float h33inv = H12(2, 2);

    vbMatchesInliers.resize(N);

    float score = 0;
    // 基于卡方检验计算出的阈值 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
    const float th = 5.991;

    const float invSigmaSquare = 1.0 / (sigma * sigma);

    for (int i = 0; i < N; i++)
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

        // 计算投影误差，2投1 1投2这么做，计算累计的卡方检验分数，分数越高证明内点与误差越优，这么做为了平衡误差与内点个数，不是说内点个数越高越好，也不是说误差越小越好
        const float w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
        const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
        const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;

        const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if (chiSquare1 > th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
        const float u1in2 = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
        const float v1in2 = (h21 * u1 + h22 * v1 + h23) * w1in2inv;

        const float squareDist2 = (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += th - chiSquare2;

        if (bIn)
            vbMatchesInliers[i] = true;
        else
            vbMatchesInliers[i] = false;
    }

    return score;
}

/** 
 * @brief 检查结果
 * @param F21 顾名思义
 * @param vbMatchesInliers 匹配是否合法，大小为mvMatches12
 * @param sigma 默认为1
 */
float TwoViewReconstruction::CheckFundamental(const Eigen::Matrix3f &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float f11 = F21(0, 0);
    const float f12 = F21(0, 1);
    const float f13 = F21(0, 2);
    const float f21 = F21(1, 0);
    const float f22 = F21(1, 1);
    const float f23 = F21(1, 2);
    const float f31 = F21(2, 0);
    const float f32 = F21(2, 1);
    const float f33 = F21(2, 2);

    vbMatchesInliers.resize(N);

    float score = 0;

    // 基于卡方检验计算出的阈值 自由度为1的卡方分布，显著性水平为0.05，对应的临界阈值
    const float th = 3.841;
    // 基于卡方检验计算出的阈值 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0 / (sigma * sigma);

    for (int i = 0; i < N; i++)
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
        // 计算 img1 上的点在 img2 上投影得到的极线 l2 = F21 * p1 = (a2,b2,c2)
        const float a2 = f11 * u1 + f12 * v1 + f13;
        const float b2 = f21 * u1 + f22 * v1 + f23;
        const float c2 = f31 * u1 + f32 * v1 + f33;

        // 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
        const float num2 = a2 * u2 + b2 * v2 + c2;

        const float squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        // 自由度为1是因为这里的计算是点到线的距离，判定分数自由度为2的原因可能是为了与H矩阵持平
        if (chiSquare1 > th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)
        // 与上面相同只不过反过来了
        const float a1 = f11 * u2 + f21 * v2 + f31;
        const float b1 = f12 * u2 + f22 * v2 + f32;
        const float c1 = f13 * u2 + f23 * v2 + f33;

        const float num1 = a1 * u1 + b1 * v1 + c1;

        const float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if (bIn)
            vbMatchesInliers[i] = true;
        else
            vbMatchesInliers[i] = false;
    }

    return score;
}

/** 
 * @brief 通过基础矩阵重建
 * @param vbMatchesInliers 匹配是否合法，大小为mvMatches12
 * @param F21 顾名思义
 * @param K 相机内参
 * @param T21 旋转平移（要输出的）
 * @param vP3D 恢复的三维点（要输出的）
 * @param vbTriangulated 大小与mvKeys1一致，表示哪个点被重建了
 * @param minParallax 1
 * @param minTriangulated 50
 */
bool TwoViewReconstruction::ReconstructF(
    vector<bool> &vbMatchesInliers, Eigen::Matrix3f &F21, Eigen::Matrix3f &K,
    Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    // 统计了合法的匹配，后面用于对比重建出的点数
    int N = 0;
    for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
        if (vbMatchesInliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    // 1. 计算本质矩阵
    Eigen::Matrix3f E21 = K.transpose() * F21 * K;

    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t;

    // Recover the 4 motion hypotheses
    // 2. 分解本质矩阵，得到4对rt
    DecomposeE(E21, R1, R2, t);

    Eigen::Vector3f t1 = t;
    Eigen::Vector3f t2 = -t;

    // Reconstruct with the 4 hyphoteses and check
    // 3. 使用四对结果分别重建
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1, vbTriangulated2, vbTriangulated3, vbTriangulated4;
    float parallax1, parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1, t1, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D1, 4.0 * mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2, t1, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D2, 4.0 * mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1, t2, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D3, 4.0 * mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2, t2, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D4, 4.0 * mSigma2, vbTriangulated4, parallax4);
    // 统计重建出点的数量最大值
    int maxGood = max(nGood1, max(nGood2, max(nGood3, nGood4)));
    // 起码要重建出超过百分之90的匹配点
    int nMinGood = max(static_cast<int>(0.9 * N), minTriangulated);

    // 4. 看看有没有脱颖而出的结果，且最大值要高于要求的最低值，如果大家都差不多说明有问题，因为4个结果中只有一个会正常
    // 大家都很多的话反而不正常了。。。
    int nsimilar = 0;
    if (nGood1 > 0.7 * maxGood)
        nsimilar++;
    if (nGood2 > 0.7 * maxGood)
        nsimilar++;
    if (nGood3 > 0.7 * maxGood)
        nsimilar++;
    if (nGood4 > 0.7 * maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if (maxGood < nMinGood || nsimilar > 1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    // 5. 使用最好的结果输出
    if (maxGood == nGood1)
    {
        if (parallax1 > minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            T21 = Sophus::SE3f(R1, t1);
            return true;
        }
    }
    else if (maxGood == nGood2)
    {
        if (parallax2 > minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            T21 = Sophus::SE3f(R2, t1);
            return true;
        }
    }
    else if (maxGood == nGood3)
    {
        if (parallax3 > minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            T21 = Sophus::SE3f(R1, t2);
            return true;
        }
    }
    else if (maxGood == nGood4)
    {
        if (parallax4 > minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            T21 = Sophus::SE3f(R2, t2);
            return true;
        }
    }

    return false;
}

/**
 * @brief 从H恢复R t
 * H矩阵分解常见有两种方法：Faugeras SVD-based decomposition 和 Zhang SVD-based decomposition
 * 参考文献：Motion and structure from motion in a piecewise plannar environment
 * 这篇参考文献和下面的代码使用了Faugeras SVD-based decomposition算法
 * @see
 * - Faugeras et al, Motion and structure from motion in a piecewise planar environment. International Journal of Pattern Recognition and Artificial Intelligence, 1988.
 * - Deeper understanding of the homography decomposition for vision-based control
 * 设平面法向量 n = (a, b, c)^t 有一点(x, y, z)在平面上，则ax + by + cz = d  即 1/d * n^t * x = 1 其中d表示
 * x' = R*x + t  从下面开始x 与 x'都表示归一化坐标
 * λ2*x' = R*(λ1*x) + t = R*(λ1*x) + t * 1/d * n^t * (λ1*x)
 * x' = λ*(R + t * 1/d * n^t) * x = H^ * x
 * 对应图像坐标   u' = G * u   G = KH^K.inv
 * H^ ~=  d*R + t * n^t = U∧V^t    ∧ = U^t * H^ * V = d*U^t * R * V + (U^t * t) * (V^t * n)^t
 * s = det(U) * det(V)  s 有可能是 1 或 -1  ∧ = s^2 * d*U^t * R * V + (U^t * t) * (V^t * n)^t = (s*d) * s * U^t * R * V + (U^t * t) * (V^t * n)^t
 * 令 R' = s * U^t * R * V      t' = U^t * t   n' = V^t * n    d' = s * d
 * ∧ = d' * R' + t' * n'^t    所以∧也可以认为是一个单应矩阵，其中加入s只为说明有符号相反的可能 
 * 设∧ = | d1, 0, 0 |    取e1 = (1, 0, 0)^t   e2 = (0, 1, 0)^t   e3 = (0, 0, 1)^t
 *      | 0, d2, 0 |
 *      | 0, 0, d3 |
 * n' = (a1, 0, 0)^t + (0, b1, 0)^t + (0, 0, c1)^t = a1*e1 + b1*e2 + c1*e3
 * 
 * ∧ = [d1*e1, d2*e2, d3*e3] = [d' * R' * e1, d' * R' * e2, d' * R' * e3] + [t' * a1, t' * b1, t' * c1]
 * ==> d1*e1 = d' * R' * e1 + t' * a1
 *     d2*e2 = d' * R' * e2 + t' * b1
 *     d3*e3 = d' * R' * e3 + t' * c1
 * 
 * 
 * 上面式子每两个消去t可得
 * d'R'(b1e1 - a1e2) = d1b1e1 - d2a1e2
 * d'R'(c1e2 - b1e3) = d2c1e1 - d3b1e3          同时取二范数，因为旋转对二范数没影响，所以可以约去
 * d'R'(a1e3 - c1e1) = d3a1e3 - d1c1e1
 * 
 * (d'^2 - d2^2)*a1^2 + (d'^2 - d1^2)*b1^2 = 0
 * (d'^2 - d3^2)*b1^2 + (d'^2 - d2^2)*c1^2 = 0   令 d'^2 - d1^2 = x1       d'^2 - d2^2 = x2       d'^2 - d3^2 = x3 
 * (d'^2 - d1^2)*c1^2 + (d'^2 - d3^2)*a1^2 = 0
 * 
 * 
 * | x2  x1  0 |     | a1^2 |
 * | 0   x3 x2 |  *  | b1^2 |  =  0    ===>  x1 * x2 * x3 = 0      有(d'^2 - d1^2) * (d'^2 - d2^2) * (d'^2 - d3^2) = 0   
 * | x3  0  x1 |     | c1^2 |
 * 由于d1 >= d2 >= d3  所以d' = d2 or -d2
 * 
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * 下面分情况讨论，当d' > 0   再根据a1^2 + b1^2 + c1^2 = 1  ??????哪来的根据，不晓得
 * 能够求得a1 = ε1 * sqrt((d1^2 - d2^2) / (d1^2 - d3^2))
 *       b1 = 0
 *       c1 = ε2 * sqrt((d2^2 - d3^2) / (d1^2 - d3^2))  其中ε1 ε2  可以为正负1
 * 结果带入 d2*e2 = d' * R' * e2 + t' * b1    => R' * e2 = e2
 *           | cosθ 0 -sinθ |
 * ==> R' =  |  0   1   0   |      n'  与   R' 带入  d'R'(a1e3 - c1e1) = d3a1e3 - d1c1e1
 *           | sinθ 0  cosθ |
 *      | cosθ 0 -sinθ |   | -c1 |    | -d1c1 |
 * d' * |  0   1   0   | * |  0  | =  |   0   |   能够解出 sinθ  与 cosθ
 *      | sinθ 0  cosθ |   |  a1 |    |  d3a1 |
 * 
 * 到此为止得到了 R'   再根据 d1*e1 = d' * R' * e1 + t' * a1
 *                         d2*e2 = d' * R' * e2 + t' * b1
 *                         d3*e3 = d' * R' * e3 + t' * c1
 * 
 * 求得 t' = (d1 - d3) * (a1, 0, c1)^t
 * @param vbMatchesInliers 匹配是否合法，大小为mvMatches12
 * @param H21 顾名思义
 * @param K 相机内参
 * @param T21 旋转平移（要输出的）
 * @param vP3D 恢复的三维点（要输出的）
 * @param vbTriangulated 大小与vbMatchesInliers，表示哪个点被重建了
 * @param minParallax 1
 * @param minTriangulated 50
 */
bool TwoViewReconstruction::ReconstructH(
    vector<bool> &vbMatchesInliers, Eigen::Matrix3f &H21, Eigen::Matrix3f &K,
    Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    // 统计了合法的匹配，后面用于对比重建出的点数
    int N = 0;
    for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
        if (vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988
    // step1：SVD分解Homography
    // 因为特征点是图像坐标系，所以讲H矩阵由相机坐标系换算到图像坐标系
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

    // SVD分解的正常情况是特征值降序排列
    if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001)
    {
        return false;
    }

    vector<Eigen::Matrix3f> vR;
    vector<Eigen::Vector3f> vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    // n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    // step2：计算法向量
    // 法向量n'= [x1 0 x3]
    float aux1 = sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    float aux3 = sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    float x1[] = {aux1, aux1, -aux1, -aux1};
    float x3[] = {aux3, -aux3, aux3, -aux3};

    // case d'=d2
    // step3：恢复旋转矩阵
    // step3.1：计算 sin(theta)和cos(theta)，case d'=d2
    float aux_stheta = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);

    float ctheta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    // step3.2：计算四种旋转矩阵R，t
    // 计算旋转矩阵 R‘，计算ppt中公式18
    //      | ctheta      0   -aux_stheta|       | aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      | aux_stheta  0    ctheta    |       |-aux3|

    //      | ctheta      0    aux_stheta|       | aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      |-aux_stheta  0    ctheta    |       | aux3|

    //      | ctheta      0    aux_stheta|       |-aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      |-aux_stheta  0    ctheta    |       |-aux3|

    //      | ctheta      0   -aux_stheta|       |-aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      | aux_stheta  0    ctheta    |       | aux3|
    for (int i = 0; i < 4; i++)
    {
        Eigen::Matrix3f Rp;
        Rp.setZero();
        Rp(0, 0) = ctheta;
        Rp(0, 2) = -stheta[i];
        Rp(1, 1) = 1.f;
        Rp(2, 0) = stheta[i];
        Rp(2, 2) = ctheta;

        Eigen::Matrix3f R = s * U * Rp * Vt;
        vR.push_back(R);

        Eigen::Vector3f tp;
        tp(0) = x1[i];
        tp(1) = 0;
        tp(2) = -x3[i];
        tp *= d1 - d3;

        // 这里虽然对t有归一化，并没有决定单目整个SLAM过程的尺度
        // 因为CreateInitialMapMonocular函数对3D点深度会缩放，然后反过来对 t 有改变
        Eigen::Vector3f t = U * tp;
        vt.push_back(t / t.norm());

        Eigen::Vector3f np;
        np(0) = x1[i];
        np(1) = 0;
        np(2) = x3[i];

        Eigen::Vector3f n = V * np;
        if (n(2) < 0)
            n = -n;
        vn.push_back(n);
    }

    // case d'=-d2
    // step3.3：计算 sin(theta)和cos(theta)，case d'=-d2
    float aux_sphi = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);

    float cphi = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    // step3.4：计算四种旋转矩阵R，t
    // 计算旋转矩阵 R‘
    for (int i = 0; i < 4; i++)
    {
        Eigen::Matrix3f Rp;
        Rp.setZero();
        Rp(0, 0) = cphi;
        Rp(0, 2) = sphi[i];
        Rp(1, 1) = -1;
        Rp(2, 0) = sphi[i];
        Rp(2, 2) = -cphi;

        Eigen::Matrix3f R = s * U * Rp * Vt;
        vR.push_back(R);

        Eigen::Vector3f tp;
        tp(0) = x1[i];
        tp(1) = 0;
        tp(2) = x3[i];
        tp *= d1 + d3;

        Eigen::Vector3f t = U * tp;
        vt.push_back(t / t.norm());

        Eigen::Vector3f np;
        np(0) = x1[i];
        np(1) = 0;
        np(2) = x3[i];

        Eigen::Vector3f n = V * np;
        if (n(2) < 0)
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
    // step4：d'=d2和d'=-d2分别对应8组(R t)，通过恢复3D点并判断是否在相机正前方的方法来确定最优解
    for (size_t i = 0; i < 8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i], vt[i], mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3Di, 4.0 * mSigma2, vbTriangulatedi, parallaxi);

        // 保留最优的和次优的
        if (nGood > bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if (nGood > secondBestGood)
        {
            secondBestGood = nGood;
        }
    }

    // step5：通过判断最优是否明显好于次优，从而判断该次Homography分解是否成功
    if (secondBestGood < 0.75 * bestGood && bestParallax >= minParallax && bestGood > minTriangulated && bestGood > 0.9 * N)
    {
        T21 = Sophus::SE3f(vR[bestSolutionIdx], vt[bestSolutionIdx]);
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

/** 
 * @brief 像素坐标标准化，计算点集的横纵均值，与均值偏差的均值。最后返回的是变化矩阵T 直接乘以像素坐标的齐次向量即可获得去中心去均值后的特征点坐标
 * @param vKeys 特征点
 * @param vNormalizedPoints 去中心去均值后的特征点坐标
 * @param T  变化矩阵
 */
void TwoViewReconstruction::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, Eigen::Matrix3f &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for (int i = 0; i < N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    // 1. 求均值
    meanX = meanX / N;
    meanY = meanY / N;

    float meanDevX = 0;
    float meanDevY = 0;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    // 2. 确定新原点后计算与新原点的距离均值
    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;

    // 3. 去均值化
    float sX = 1.0 / meanDevX;
    float sY = 1.0 / meanDevY;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    // 4. 计算变化矩阵
    T.setZero();
    T(0, 0) = sX;
    T(1, 1) = sY;
    T(0, 2) = -meanX * sX;
    T(1, 2) = -meanY * sY;
    T(2, 2) = 1.f;
}

/**
 * @brief 进行cheirality check，从而进一步找出F分解后最合适的解
 * @param R 旋转
 * @param t 平移
 * @param vKeys1 特征点
 * @param vKeys2 特征点
 * @param vMatches12 匹配关系
 * @param vbMatchesInliers 匹配关系是否有效
 * @param K 内参
 * @param vP3D 三维点
 * @param th2 误差半径
 * @param vbGood 大小与mvKeys1一致，表示哪个点被重建了
 * @param parallax 
 */
int TwoViewReconstruction::CheckRT(
    const Eigen::Matrix3f &R, const Eigen::Vector3f &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
    const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
    const Eigen::Matrix3f &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K(0, 0);
    const float fy = K(1, 1);
    const float cx = K(0, 2);
    const float cy = K(1, 2);

    vbGood = vector<bool>(vKeys1.size(), false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    // 步骤1：得到一个相机的投影矩阵
    // 以第一个相机的光心作为世界坐标系
    Eigen::Matrix<float, 3, 4> P1;
    P1.setZero();
    P1.block<3, 3>(0, 0) = K;

    Eigen::Vector3f O1;
    O1.setZero();

    // Camera 2 Projection Matrix K[R|t]
    // 步骤2：得到第二个相机的投影矩阵
    Eigen::Matrix<float, 3, 4> P2;
    P2.block<3, 3>(0, 0) = R;
    P2.block<3, 1>(0, 3) = t;
    P2 = K * P2;

    // 第二个相机的光心在世界坐标系下的坐标
    Eigen::Vector3f O2 = -R.transpose() * t;

    int nGood = 0;

    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (!vbMatchesInliers[i])
            continue;

        // kp1和kp2是匹配特征点
        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

        Eigen::Vector3f p3dC1;
        Eigen::Vector3f x_p1(kp1.pt.x, kp1.pt.y, 1);
        Eigen::Vector3f x_p2(kp2.pt.x, kp2.pt.y, 1);

        // 步骤3：利用三角法恢复三维点p3dC1
        GeometricTools::Triangulate(x_p1, x_p2, P1, P2, p3dC1);

        if (!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
        {
            vbGood[vMatches12[i].first] = false;
            continue;
        }

        // Check parallax
        // 步骤4：计算视差角余弦值
        Eigen::Vector3f normal1 = p3dC1 - O1;
        float dist1 = normal1.norm();

        Eigen::Vector3f normal2 = p3dC1 - O2;
        float dist2 = normal2.norm();

        float cosParallax = normal1.dot(normal2) / (dist1 * dist2);

        // 步骤5：判断3D点是否在两个摄像头前方
        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        // 步骤5.1：3D点深度为负，在第一个摄像头后方，淘汰
        if (p3dC1(2) <= 0 && cosParallax < 0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        // 步骤5.2：3D点深度为负，在第二个摄像头后方，淘汰
        Eigen::Vector3f p3dC2 = R * p3dC1 + t;

        if (p3dC2(2) <= 0 && cosParallax < 0.99998)
            continue;

        // 步骤6：计算重投影误差
        // Check reprojection error in first image
        // 计算3D点在第一个图像上的投影误差
        float im1x, im1y;
        float invZ1 = 1.0 / p3dC1(2);
        im1x = fx * p3dC1(0) * invZ1 + cx;
        im1y = fy * p3dC1(1) * invZ1 + cy;

        float squareError1 = (im1x - kp1.pt.x) * (im1x - kp1.pt.x) + (im1y - kp1.pt.y) * (im1y - kp1.pt.y);

        // 步骤6.1：重投影误差太大，跳过淘汰
        // 一般视差角比较小时重投影误差比较大
        if (squareError1 > th2)
            continue;

        // Check reprojection error in second image
        // 计算3D点在第二个图像上的投影误差
        float im2x, im2y;
        float invZ2 = 1.0 / p3dC2(2);
        im2x = fx * p3dC2(0) * invZ2 + cx;
        im2y = fy * p3dC2(1) * invZ2 + cy;

        float squareError2 = (im2x - kp2.pt.x) * (im2x - kp2.pt.x) + (im2y - kp2.pt.y) * (im2y - kp2.pt.y);

        // 步骤6.1：重投影误差太大，跳过淘汰
        // 一般视差角比较小时重投影误差比较大
        if (squareError2 > th2)
            continue;

        // 步骤7：统计经过检验的3D点个数，记录3D点视差角
        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1(0), p3dC1(1), p3dC1(2));
        nGood++;

        if (cosParallax < 0.99998)
            vbGood[vMatches12[i].first] = true;
    }

    // 7 得到3D点中较小的视差角，并且转换成为角度制表示
    if (nGood > 0)
    {
        // 从小到大排序，注意vCosParallax值越大，视差越小
        sort(vCosParallax.begin(), vCosParallax.end());

        // !排序后并没有取最小的视差角，而是取一个较小的视差角
		// 作者的做法：如果经过检验过后的有效3D点小于50个，那么就取最后那个最小的视差角(cos值最大)
		// 如果大于50个，就取排名第50个的较小的视差角即可，为了避免3D点太多时出现太小的视差角 
        size_t idx = min(50, int(vCosParallax.size() - 1));
        // 将这个选中的角弧度制转换为角度制
        parallax = acos(vCosParallax[idx]) * 180 / CV_PI;
    }
    else
        parallax = 0;

    return nGood;
}

/**
 * @brief 分解Essential矩阵
 * 解释的比较好的博客：https://blog.csdn.net/weixin_44580210/article/details/90344511
 * F矩阵通过结合内参可以得到Essential矩阵，分解E矩阵将得到4组解
 * 这4组解分别为[R1,t],[R1,-t],[R2,t],[R2,-t]
 * ## 反对称矩阵性质
 * 多视图几何上定义：一个3×3的矩阵是本质矩阵的充要条件是它的奇异值中有两个相等而第三个是0，为什么呢？
 * 首先我们知道 E=[t]×​R=SR其中S为反对称矩阵，反对称矩阵有什么性质呢？
 * 结论1：如果 S 是实的反对称矩阵，那么S=UBU^T，其中 B 为形如diag(a1​Z，a2​Z...am​Z，0，0...0)的分块对角阵，其中 Z = [0, 1; -1, 0]
 * 反对称矩阵的特征矢量都是纯虚数并且奇数阶的反对称矩阵必是奇异的
 * 那么根据这个结论我们可以将 S 矩阵写成 S=kUZU^⊤，而 Z 为
 * | 0, 1, 0 |
 * |-1, 0, 0 |
 * | 0, 0, 0 |
 * Z = diag(1, 1, 0) * W     W 为
 * | 0,-1, 0 |
 * | 1, 0, 0 |
 * | 0, 0, 1 |
 * E=SR=Udiag(1,1,0)(WU^⊤R)  这样就证明了 E 拥有两个相等的奇异值
 * 
 * ## 恢复相机矩阵
 * 假定第一个摄像机矩阵是P=[I∣0]，为了计算第二个摄像机矩阵P′，必须把 E 矩阵分解为反对成举着和旋转矩阵的乘积 SR。
 * 还是根据上面的结论1，我们在相差一个常数因子的前提下有 S=UZU^T，我们假设旋转矩阵分解为UXV^T，注意这里是假设旋转矩阵分解形式为UXV^T，并不是旋转矩阵的svd分解，
 * 其中 UV都是E矩阵分解出的
 * Udiag(1,1,0)V^T = E = SR = (UZU^T)(UXV^⊤) = U(ZX)V^T
 * 则有 ZX = diag(1,1,0)，因此 x=W或者 X=W^T
 * 结论：如果 E 的SVD分解为 Udiag(1,1,0)V^⊤，E = SR有两种分解形式，分别是： S = UZU^⊤    R = UWVTor UW^TV^⊤

 * 接着分析，又因为St=0（自己和自己叉乘肯定为0嘛）以及∥t∥=1（对两个摄像机矩阵的基线的一种常用归一化），因此 t = U(0,0,1)^T = u3​，
 * 即矩阵 U 的最后一列，这样的好处是不用再去求S了，应为t的符号不确定，R矩阵有两种可能，因此其分解有如下四种情况：
 * P′=[UWV^T ∣ +u3​] or [UWV^T ∣ −u3​] or [UW^TV^T ∣ +u3​] or [UW^TV^T ∣ −u3​]
 * @param E  Essential Matrix
 * @param R1 Rotation Matrix 1
 * @param R2 Rotation Matrix 2
 * @param t  Translation
 * @see Multiple View Geometry in Computer Vision - Result 9.19 p259
 */
void TwoViewReconstruction::DecomposeE(const Eigen::Matrix3f &E, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2, Eigen::Vector3f &t)
{

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // 对 t 有归一化，但是这个地方并没有决定单目整个SLAM过程的尺度
    // 因为CreateInitialMapMonocular函数对3D点深度会缩放，然后反过来对 t 有改变
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f Vt = svd.matrixV().transpose();

    t = U.col(2);
    t = t / t.norm();

    Eigen::Matrix3f W;
    W.setZero();
    W(0, 1) = -1;
    W(1, 0) = 1;
    W(2, 2) = 1;

    R1 = U * W * Vt;
    // 旋转矩阵有行列式为1的约束
    if (R1.determinant() < 0)
        R1 = -R1;

    R2 = U * W.transpose() * Vt;
    if (R2.determinant() < 0)
        R2 = -R2;
}

} // namespace ORB_SLAM
