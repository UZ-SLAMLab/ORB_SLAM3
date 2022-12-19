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

#include "KannalaBrandt8.h"

#include <boost/serialization/export.hpp>

// BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::KannalaBrandt8)

namespace ORB_SLAM3
{
// BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

/** 
 * @brief 投影
 * xc​ = Xc/Zc, yc = Yc/Zc
 * r^2 = xc^2 + yc^2
 * θ = arctan(r)
 * θd = k0*θ + k1*θ^3 + k2*θ^5 + k3*θ^7 + k4*θ^9
 * xd = θd/r * xc   yd = θd/r * yc
 * u = fx*xd + cx  v = fy*yd + cy
 * @param p3D 三维点
 * @return 像素坐标
 */
cv::Point2f KannalaBrandt8::project(const cv::Point3f &p3D)
{
    const float x2_plus_y2 = p3D.x * p3D.x + p3D.y * p3D.y;
    const float theta = atan2f(sqrtf(x2_plus_y2), p3D.z);
    const float psi = atan2f(p3D.y, p3D.x);

    const float theta2 = theta * theta;
    const float theta3 = theta * theta2;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5 + mvParameters[6] * theta7 + mvParameters[7] * theta9;

    return cv::Point2f(mvParameters[0] * r * cos(psi) + mvParameters[2],
                        mvParameters[1] * r * sin(psi) + mvParameters[3]);
}

/** 
 * @brief 投影
 * @param v3D 三维点
 * @return 像素坐标
 */
Eigen::Vector2d KannalaBrandt8::project(const Eigen::Vector3d &v3D)
{
    const double x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
    const double theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
    const double psi = atan2f(v3D[1], v3D[0]);

    const double theta2 = theta * theta;
    const double theta3 = theta * theta2;
    const double theta5 = theta3 * theta2;
    const double theta7 = theta5 * theta2;
    const double theta9 = theta7 * theta2;
    const double r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5 + mvParameters[6] * theta7 + mvParameters[7] * theta9;

    Eigen::Vector2d res;
    res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
    res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

    return res;
}

/** 
 * @brief 投影
 * @param v3D 三维点
 * @return 像素坐标
 */
Eigen::Vector2f KannalaBrandt8::project(const Eigen::Vector3f &v3D)
{
    const float x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
    const float theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
    const float psi = atan2f(v3D[1], v3D[0]);

    const float theta2 = theta * theta;
    const float theta3 = theta * theta2;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5 + mvParameters[6] * theta7 + mvParameters[7] * theta9;

    Eigen::Vector2f res;
    res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
    res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

    return res;

    /*cv::Point2f cvres = this->project(cv::Point3f(v3D[0],v3D[1],v3D[2]));

    Eigen::Vector2d res;
    res[0] = cvres.x;
    res[1] = cvres.y;

    return res;*/
}

/** 
 * @brief 投影
 * @param p3D 三维点
 * @return 像素坐标
 */
Eigen::Vector2f KannalaBrandt8::projectMat(const cv::Point3f &p3D)
{
    cv::Point2f point = this->project(p3D);
    return Eigen::Vector2f(point.x, point.y);
}

float KannalaBrandt8::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D)
{
    /*Eigen::Matrix<double,2,1> c;
    c << mvParameters[2], mvParameters[3];
    if ((p2D-c).squaredNorm()>57600) // 240*240 (256)
        return 100.f;
    else
        return 1.0f;*/
    return 1.f;
}

/** 
 * @brief 反投影,以Eigen::Vector3f形式返回，KannalaBrandt8::TriangulateMatches中调用
 * @param p2D 特征点
 * @return 
 */
Eigen::Vector3f KannalaBrandt8::unprojectEig(const cv::Point2f &p2D)
{
    cv::Point3f ray = this->unproject(p2D);
    return Eigen::Vector3f(ray.x, ray.y, ray.z);
}

/** 
 * @brief 反投影
 * 投影过程
 * xc​ = Xc/Zc, yc = Yc/Zc
 * r^2 = xc^2 + yc^2
 * θ = arctan(r)
 * θd = k0*θ + k1*θ^3 + k2*θ^5 + k3*θ^7 + k4*θ^9
 * xd = θd/r * xc   yd = θd/r * yc
 * u = fx*xd + cx  v = fy*yd + cy
 * 
 * 
 * 已知u与v 未矫正的特征点像素坐标
 * xd = (u - cx) / fx    yd = (v - cy) / fy
 * 待求的 xc = xd * r / θd  yc = yd * r / θd
 * 待求的 xc = xd * tan(θ) / θd  yc = yd * tan(θ) / θd
 * 其中 θd的算法如下：
 *     xd^2 + yd^2 = θd^2/r^2 * (xc^2 + yc^2)
 *     θd = sqrt(xd^2 + yd^2) / sqrt(xc^2 + yc^2) * r
 *     其中r = sqrt(xc^2 + yc^2)
 *     所以 θd = sqrt(xd^2 + yd^2)  已知
 * 所以待求的只有tan(θ),也就是θ
 * 这里θd = θ + k1*θ^3 + k2*θ^5 + k3*θ^7 + k4*θ^9
 * 直接求解θ比较麻烦，这里用迭代的方式通过误差的一阶导数求θ
 * θ的初始值定为了θd，设θ + k1*θ^3 + k2*θ^5 + k3*θ^7 + k4*θ^9 = f(θ)
 * e(θ) = f(θ) - θd 目标是求解一个θ值另e(θ) = 0
 * 泰勒展开e(θ+δθ) = e(θ) + e`(θ) * δθ = 0
 * e`(θ) = 1 + 3*k1*θ^*2 + 5*k2*θ^4 + 7*k3*θ^6 + 8*k4*θ^8
 * δθ = -e(θ)/e`(θ) 经过多次迭代可求得θ
 * 最后xc = xd * tan(θ) / θd  yc = yd * tan(θ) / θd
 * @param p2D 特征点
 * @return 
 */
cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &p2D)
{
    // Use Newthon method to solve for theta with good precision (err ~ e-6)
    cv::Point2f pw((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1]);
    float scale = 1.f;
    float theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);  // sin(psi) = yc / r
    theta_d = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);  // 不能超过180度

    if (theta_d > 1e-8)
    {
        // Compensate distortion iteratively
        // θ的初始值定为了θd
        float theta = theta_d;

        // 开始迭代
        for (int j = 0; j < 10; j++)
        {
            float theta2 = theta * theta,
                  theta4 = theta2 * theta2,
                  theta6 = theta4 * theta2,
                  theta8 = theta4 * theta4;
            float k0_theta2 = mvParameters[4] * theta2,
                  k1_theta4 = mvParameters[5] * theta4;
            float k2_theta6 = mvParameters[6] * theta6,
                  k3_theta8 = mvParameters[7] * theta8;
            float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
            theta = theta - theta_fix;
            if (fabsf(theta_fix) < precision)  // 如果更新量变得很小，表示接近最终值
                break;
        }
        // scale = theta - theta_d;
        // 求得tan(θ) / θd
        scale = std::tan(theta) / theta_d;
    }

    return cv::Point3f(pw.x * scale, pw.y * scale, 1.f);
}

/** 
 * @brief 求解二维像素坐标关于三维点的雅克比矩阵
 * u = fx * θd * x / sqrt(x^2 + y^2) + cx
 * v = fy * θd * y / sqrt(x^2 + y^2) + cy
 * 这两个式子分别对 xyz 求导
 * θd = θ + k1*θ^3 + k2*θ^5 + k3*θ^7 + k4*θ^9
 * θ = arctan(sqrt(x^2 + y^2), z)
 * @param p3D 三维点
 * @return 
 */
Eigen::Matrix<double, 2, 3> KannalaBrandt8::projectJac(const Eigen::Vector3d &v3D)
{
    double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
    double r2 = x2 + y2;
    double r = sqrt(r2);
    double r3 = r2 * r;
    double theta = atan2(r, v3D[2]);

    double theta2 = theta * theta, theta3 = theta2 * theta;
    double theta4 = theta2 * theta2, theta5 = theta4 * theta;
    double theta6 = theta2 * theta4, theta7 = theta6 * theta;
    double theta8 = theta4 * theta4, theta9 = theta8 * theta;

    double f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] +
                theta9 * mvParameters[7];
    double fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
                9 * mvParameters[7] * theta8;

    Eigen::Matrix<double, 2, 3> JacGood;
    JacGood(0, 0) = mvParameters[0] * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);  // ∂u/∂x
    JacGood(1, 0) =
        mvParameters[1] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);  // ∂v/∂x

    JacGood(0, 1) =
        mvParameters[0] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);  // ∂u/∂y
    JacGood(1, 1) = mvParameters[1] * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);  // ∂v/∂y

    JacGood(0, 2) = -mvParameters[0] * fd * v3D[0] / (r2 + z2);  // ∂u/∂z
    JacGood(1, 2) = -mvParameters[1] * fd * v3D[1] / (r2 + z2);  // ∂v/∂z

    return JacGood;
}

/** 三角化恢复三维点，但要提前做去畸变 单目初始化中使用
 * @param vKeys1 第一帧的关键点
 * @param vKeys2 第二帧的关键点
 * @param vMatches12 匹配关系，长度与vKeys1一样，对应位置存放vKeys2中关键点的下标
 * @param T21 顾名思义
 * @param vP3D 恢复出的三维点
 * @param vbTriangulated 是否三角化成功，用于统计匹配点数量
 */
bool KannalaBrandt8::ReconstructWithTwoViews(
    const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2, const std::vector<int> &vMatches12,
    Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated)
{
    if (!tvr)
    {
        Eigen::Matrix3f K = this->toK_();
        tvr = new TwoViewReconstruction(K);
    }

    // Correct FishEye distortion
    std::vector<cv::KeyPoint> vKeysUn1 = vKeys1, vKeysUn2 = vKeys2;
    std::vector<cv::Point2f> vPts1(vKeys1.size()), vPts2(vKeys2.size());

    for (size_t i = 0; i < vKeys1.size(); i++)
        vPts1[i] = vKeys1[i].pt;
    for (size_t i = 0; i < vKeys2.size(); i++)
        vPts2[i] = vKeys2[i].pt;

    cv::Mat D = (cv::Mat_<float>(4, 1) << mvParameters[4], mvParameters[5], mvParameters[6], mvParameters[7]);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat K = this->toK();
    cv::fisheye::undistortPoints(vPts1, vPts1, K, D, R, K);
    cv::fisheye::undistortPoints(vPts2, vPts2, K, D, R, K);

    for (size_t i = 0; i < vKeys1.size(); i++)
        vKeysUn1[i].pt = vPts1[i];
    for (size_t i = 0; i < vKeys2.size(); i++)
        vKeysUn2[i].pt = vPts2[i];

    return tvr->Reconstruct(vKeysUn1, vKeysUn2, vMatches12, T21, vP3D, vbTriangulated);
}

/**
 * @brief 返回内参矩阵
 * @return K
 */
cv::Mat KannalaBrandt8::toK()
{
    cv::Mat K = (cv::Mat_<float>(3, 3) << 
        mvParameters[0], 0.f,             mvParameters[2],
        0.f,             mvParameters[1], mvParameters[3],
        0.f,             0.f,             1.f);
    return K;
}
Eigen::Matrix3f KannalaBrandt8::toK_()
{
    Eigen::Matrix3f K;
    K << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f;
    return K;
}

bool KannalaBrandt8::epipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                        const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel, const float unc)
{
    Eigen::Vector3f p3D;
    // 用三角化出点并验证的这个过程代替极线验证
    return this->TriangulateMatches(pCamera2, kp1, kp2, R12, t12, sigmaLevel, unc, p3D) > 0.0001f;
}

// 没用
bool KannalaBrandt8::matchAndtriangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, GeometricCamera *pOther,
                                            Sophus::SE3f &Tcw1, Sophus::SE3f &Tcw2,
                                            const float sigmaLevel1, const float sigmaLevel2,
                                            Eigen::Vector3f &x3Dtriangulated)
{
    Eigen::Matrix<float, 3, 4> eigTcw1 = Tcw1.matrix3x4();
    Eigen::Matrix3f Rcw1 = eigTcw1.block<3, 3>(0, 0);
    Eigen::Matrix3f Rwc1 = Rcw1.transpose();
    Eigen::Matrix<float, 3, 4> eigTcw2 = Tcw2.matrix3x4();
    Eigen::Matrix3f Rcw2 = eigTcw2.block<3, 3>(0, 0);
    Eigen::Matrix3f Rwc2 = Rcw2.transpose();

    cv::Point3f ray1c = this->unproject(kp1.pt);
    cv::Point3f ray2c = pOther->unproject(kp2.pt);

    // 获得点1在帧1的归一化坐标
    Eigen::Vector3f r1(ray1c.x, ray1c.y, ray1c.z);
    // 获得点2在帧2的归一化坐标
    Eigen::Vector3f r2(ray2c.x, ray2c.y, ray2c.z);

    // Check parallax between rays
    // 射线在世界坐标系下
    Eigen::Vector3f ray1 = Rwc1 * r1;
    Eigen::Vector3f ray2 = Rwc2 * r2;

    const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

    // If parallax is lower than 0.9998, reject this match
    // 夹角几乎为0时返回，因为表示这个点过远，三角化会带来大量误差
    if (cosParallaxRays > 0.9998)
    {
        return false;
    }

    // Parallax is good, so we try to triangulate
    cv::Point2f p11, p22;

    p11.x = ray1c.x;
    p11.y = ray1c.y;

    p22.x = ray2c.x;
    p22.y = ray2c.y;

    Eigen::Vector3f x3D;

    // 三角化
    Triangulate(p11, p22, eigTcw1, eigTcw2, x3D);

    // Check triangulation in front of cameras
    // 查看点是否位于相机前面
    float z1 = Rcw1.row(2).dot(x3D) + Tcw1.translation()(2);
    if (z1 <= 0)
    { // Point is not in front of the first camera
        return false;
    }

    float z2 = Rcw2.row(2).dot(x3D) + Tcw2.translation()(2);
    if (z2 <= 0)
    { // Point is not in front of the first camera
        return false;
    }

    // Check reprojection error in first keyframe
    //   -Transform point into camera reference system
    // 查看投影误差
    Eigen::Vector3f x3D1 = Rcw1 * x3D + Tcw1.translation();
    Eigen::Vector2f uv1 = this->project(x3D1);

    float errX1 = uv1(0) - kp1.pt.x;
    float errY1 = uv1(1) - kp1.pt.y;

    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel1)
    { // Reprojection error is high
        return false;
    }

    // Check reprojection error in second keyframe;
    //   -Transform point into camera reference system
    Eigen::Vector3f x3D2 = Rcw2 * x3D + Tcw2.translation(); // avoid using q
    Eigen::Vector2f uv2 = pOther->project(x3D2);

    float errX2 = uv2(0) - kp2.pt.x;
    float errY2 = uv2(1) - kp2.pt.y;

    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaLevel2)
    { // Reprojection error is high
        return false;
    }

    // Since parallax is big enough and reprojection errors are low, this pair of points
    // can be considered as a match
    x3Dtriangulated = x3D;

    return true;
}

/** 
 * @brief 通过三角化后的重投影误差判断点匹配的情况，如果比较好则返回深度值
 * @param pCamera2 右相机
 * @param kp1 左相机特征点
 * @param kp2 右相机特征点
 * @param R12 2->1的旋转
 * @param t12 2->1的平移
 * @param sigmaLevel 特征点1的尺度的平方
 * @param unc 特征点2的尺度的平方
 * @param p3D 恢复的三维点
 * @return 点的深度值
 */
float KannalaBrandt8::TriangulateMatches(
    GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
    const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel,
    const float unc, Eigen::Vector3f &p3D)
{
    // 1. 得到对应特征点的归一化平面坐标
    Eigen::Vector3f r1 = this->unprojectEig(kp1.pt);
    Eigen::Vector3f r2 = pCamera2->unprojectEig(kp2.pt);

    // Check parallax
    // 2. 查看射线夹角
    // 这里有点像极线约束，但并不是，将r2通过R12旋转到与r1同方向的坐标系
    // 然后计算他们的夹角，看其是否超过1.14° 
    Eigen::Vector3f r21 = R12 * r2;

    const float cosParallaxRays = r1.dot(r21) / (r1.norm() * r21.norm());

    if (cosParallaxRays > 0.9998)
    {
        return -1;
    }

    // Parallax is good, so we try to triangulate
    cv::Point2f p11, p22;

    p11.x = r1[0];
    p11.y = r1[1];

    p22.x = r2[0];
    p22.y = r2[1];

    Eigen::Vector3f x3D;
    Eigen::Matrix<float, 3, 4> Tcw1;

    // 3. 设定变换矩阵用于三角化
    Tcw1 << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();

    Eigen::Matrix<float, 3, 4> Tcw2;

    Eigen::Matrix3f R21 = R12.transpose();
    Tcw2 << R21, -R21 * t12;

    // 4. 三角化
    Triangulate(p11, p22, Tcw1, Tcw2, x3D);
    // cv::Mat x3Dt = x3D.t();

    // 深度值是否正常
    float z1 = x3D(2);
    if (z1 <= 0)
    {
        return -2;
    }

    float z2 = R21.row(2).dot(x3D) + Tcw2(2, 3);
    if (z2 <= 0)
    {
        return -3;
    }

    // Check reprojection error
    // 5. 做下投影计算重投影误差
    Eigen::Vector2f uv1 = this->project(x3D);

    float errX1 = uv1(0) - kp1.pt.x;
    float errY1 = uv1(1) - kp1.pt.y;

    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel)
    { // Reprojection error is high
        return -4;
    }

    Eigen::Vector3f x3D2 = R21 * x3D + Tcw2.col(3);
    Eigen::Vector2f uv2 = pCamera2->project(x3D2);

    float errX2 = uv2(0) - kp2.pt.x;
    float errY2 = uv2(1) - kp2.pt.y;

    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * unc)
    { // Reprojection error is high
        return -5;
    }

    p3D = x3D;

    return z1;
}

std::ostream &operator<<(std::ostream &os, const KannalaBrandt8 &kb)
{
    os << kb.mvParameters[0] << " " << kb.mvParameters[1] << " " << kb.mvParameters[2] << " " << kb.mvParameters[3] << " "
        << kb.mvParameters[4] << " " << kb.mvParameters[5] << " " << kb.mvParameters[6] << " " << kb.mvParameters[7];
    return os;
}

std::istream &operator>>(std::istream &is, KannalaBrandt8 &kb)
{
    float nextParam;
    for (size_t i = 0; i < 8; i++)
    {
        assert(is.good()); // Make sure the input stream is good
        is >> nextParam;
        kb.mvParameters[i] = nextParam;
    }
    return is;
}

/** 
 * @brief 通过三角化恢复归一化坐标的三维点坐标，该三维点为在左相机坐标系下的点
 * @param p1 右相机
 * @param p2 左相机特征点
 * @param Tcw1 3×4 单位矩阵
 * @param Tcw2 3×4 T21
 * @param x3D 恢复的三维点
 */
void KannalaBrandt8::Triangulate(
    const cv::Point2f &p1, const cv::Point2f &p2, const Eigen::Matrix<float, 3, 4> &Tcw1,
    const Eigen::Matrix<float, 3, 4> &Tcw2, Eigen::Vector3f &x3D)
{
    Eigen::Matrix<float, 4, 4> A;
    // 代码中有用到三角化的地方有：TwoViewReconstruction::Triangulate LocalMapping::CreateNewMapPoints KannalaBrandt8::Triangulate Initializer::Triangulate
    // 见Initializer.cpp的Triangulate函数,A矩阵构建的方式类似，不同的是乘的反对称矩阵那个是像素坐标构成的，而这个是归一化坐标构成的
    // Pc = Tcw*Pw， 左右两面乘Pc的反对称矩阵 [Pc]x * Tcw *Pw = 0 构成了A矩阵，中间涉及一个尺度a，因为都是归一化平面，但右面是0所以直接可以约掉不影响最后的尺度
    //  0 -1 y    Tcw.row(0)     -Tcw.row(1) + y*Tcw.row(2)
    //  1 0 -x *  Tcw.row(1)  =   Tcw.row(0) - x*Tcw.row(2) 
    // -y x  0    Tcw.row(2)    x*Tcw.row(1) - y*Tcw.row(0)
    // 发现上述矩阵线性相关，所以取前两维，两个点构成了4行的矩阵，就是如下的操作，求出的是4维的结果[X,Y,Z,A]，所以需要除以最后一维使之为1，就成了[X,Y,Z,1]这种齐次形式
    A.row(0) = p1.x * Tcw1.row(2) - Tcw1.row(0);
    A.row(1) = p1.y * Tcw1.row(2) - Tcw1.row(1);
    A.row(2) = p2.x * Tcw2.row(2) - Tcw2.row(0);
    A.row(3) = p2.y * Tcw2.row(2) - Tcw2.row(1);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4f x3Dh = svd.matrixV().col(3);
    x3D = x3Dh.head(3) / x3Dh(3);
}

bool KannalaBrandt8::IsEqual(GeometricCamera *pCam)
{
    if (pCam->GetType() != GeometricCamera::CAM_FISHEYE)
        return false;

    KannalaBrandt8 *pKBCam = (KannalaBrandt8 *)pCam;

    if (abs(precision - pKBCam->GetPrecision()) > 1e-6)
        return false;

    if (size() != pKBCam->size())
        return false;

    bool is_same_camera = true;
    for (size_t i = 0; i < size(); ++i)
    {
        if (abs(mvParameters[i] - pKBCam->getParameter(i)) > 1e-6)
        {
            is_same_camera = false;
            break;
        }
    }
    return is_same_camera;
}

}
