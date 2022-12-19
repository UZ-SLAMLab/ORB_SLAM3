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

#include "GeometricTools.h"

#include "KeyFrame.h"

namespace ORB_SLAM3
{

/** 
 * @brief 计算两个关键帧间的基础矩阵
 * @param pKF1 关键帧1
 * @param pKF2 关键帧2
 */
Eigen::Matrix3f GeometricTools::ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2)
{
    // 获取关键帧1的旋转平移
    Sophus::SE3<float> Tc1w = pKF1->GetPose();
    Sophus::Matrix3<float> Rc1w = Tc1w.rotationMatrix();
    Sophus::SE3<float>::TranslationMember tc1w = Tc1w.translation();

    // 获取关键帧2的旋转平移
    Sophus::SE3<float> Tc2w = pKF2->GetPose();
    Sophus::Matrix3<float> Rc2w = Tc2w.rotationMatrix();
    Sophus::SE3<float>::TranslationMember tc2w = Tc2w.translation();

    // 计算2->1的旋转平移
    Sophus::Matrix3<float> Rc1c2 = Rc1w * Rc2w.transpose();
    Eigen::Vector3f tc1c2 = -Rc1c2 * tc2w + tc1w;

    Eigen::Matrix3f tc1c2x = Sophus::SO3f::hat(tc1c2);

    const Eigen::Matrix3f K1 = pKF1->mpCamera->toK_();
    const Eigen::Matrix3f K2 = pKF2->mpCamera->toK_();

    return K1.transpose().inverse() * tc1c2x * Rc1c2 * K2.inverse();
}

/** 
 * @brief 三角化获得三维点
 * @param x_c1 点在关键帧1下的归一化坐标
 * @param x_c2 点在关键帧2下的归一化坐标
 * @param Tc1w 关键帧1投影矩阵  [K*R | K*t]
 * @param Tc2w 关键帧2投影矩阵  [K*R | K*t]
 * @param x3D 三维点坐标，作为结果输出
 */
bool GeometricTools::Triangulate(
    Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2, Eigen::Matrix<float,3,4> &Tc1w, Eigen::Matrix<float,3,4> &Tc2w,
    Eigen::Vector3f &x3D)
{
    Eigen::Matrix4f A;
    // x = a*P*X， 左右两面乘Pc的反对称矩阵 a*[x]^ * P *X = 0 构成了A矩阵，中间涉及一个尺度a，因为都是归一化平面，但右面是0所以直接可以约掉不影响最后的尺度
    //  0 -1 v    P(0)     -P.row(1) + v*P.row(2)
    //  1 0 -u *  P(1)  =   P.row(0) - u*P.row(2) 
    // -v u  0    P(2)    u*P.row(1) - v*P.row(0)
    // 发现上述矩阵线性相关，所以取前两维，两个点构成了4行的矩阵，就是如下的操作，求出的是4维的结果[X,Y,Z,A]，所以需要除以最后一维使之为1，就成了[X,Y,Z,1]这种齐次形式
    A.block<1,4>(0,0) = x_c1(0) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(0,0);
    A.block<1,4>(1,0) = x_c1(1) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(1,0);
    A.block<1,4>(2,0) = x_c2(0) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(0,0);
    A.block<1,4>(3,0) = x_c2(1) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(1,0);

    // 解方程 AX=0
    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

    Eigen::Vector4f x3Dh = svd.matrixV().col(3);

    if(x3Dh(3)==0)
        return false;

    // Euclidean coordinates
    x3D = x3Dh.head(3)/x3Dh(3);

    return true;
}

} //namespace ORB_SLAM
