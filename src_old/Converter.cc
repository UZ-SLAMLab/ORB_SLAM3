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

#include "Converter.h"

namespace ORB_SLAM3
{

/**
 * @brief 将描述子转换为描述子向量，其实本质上是cv:Mat->std:vector
 */
std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    // 存储转换结果的向量
    std::vector<cv::Mat> vDesc;
    // 创建保留空间
    vDesc.reserve(Descriptors.rows);
    // 对于每一个特征点的描述子
    for (int j=0;j<Descriptors.rows;j++)
        //从描述子这个矩阵中抽取出来存到向量中
        vDesc.push_back(Descriptors.row(j));
    // 返回转换结果
    return vDesc;
}

/**
 * @brief 将变换矩阵转换为李代数se3：cv:Mat->g2o::SE3Quat
 */
g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    // 首先将旋转矩阵提取出来
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    // 然后将平移向量提取出来
    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));
    // 构造g2o::SE3Quat类型并返回
    return g2o::SE3Quat(R,t);
}

/**
 * @brief ophus::SE3f->g2o::SE3Quat
 */
g2o::SE3Quat Converter::toSE3Quat(const Sophus::SE3f &T)
{
    return g2o::SE3Quat(T.unit_quaternion().cast<double>(), T.translation().cast<double>());
}

/**
 * @brief g2o::SE3Quat->cv::Mat
 */
cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

/**
 * @brief 将仿射矩阵由g2o::Sim3->cv::Mat
 */
cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    // 首先将仿射矩阵的旋转部分转换成为Eigen下的矩阵格式
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    // 对于仿射矩阵的平移部分也是要转换成为Eigen下的矩阵格式
    Eigen::Vector3d eigt = Sim3.translation();
    // 获取仿射矩阵的缩放系数
    double s = Sim3.scale();
    // 然后构造cv::Mat格式下的仿射矩阵
    return toCvSE3(s*eigR,eigt);
}

/**
 * @brief Eigen::Matrix<double,4,4> -> cv::Mat，用于变换矩阵T的中间转换
 */
cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    // 首先定义存储计算结果的变量
    cv::Mat cvMat(4,4,CV_32F);
    // 然后逐个元素赋值
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);
    // 返回计算结果，还是用深拷贝函数
    return cvMat.clone();
}

/**
 * @brief Eigen::Matrix<float,4,4> -> cv::Mat，用于变换矩阵T的中间转换
 */
cv::Mat Converter::toCvMat(const Eigen::Matrix<float,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/**
 * @brief Eigen::Matrix<float,3,4> -> cv::Mat
 */
cv::Mat Converter::toCvMat(const Eigen::Matrix<float,3,4> &m)
{
    cv::Mat cvMat(3,4,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/**
 * @brief Eigen::Matrix<double,3,3> -> cv::Mat
 */
cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/**
 * @brief Eigen::Matrix<float,3,3> -> cv::Mat
 */
cv::Mat Converter::toCvMat(const Eigen::Matrix3f &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/**
 * @brief Eigen::MatrixXf -> cv::Mat
 */
cv::Mat Converter::toCvMat(const Eigen::MatrixXf &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/**
 * @brief Eigen::MatrixXd -> cv::Mat
 */
cv::Mat Converter::toCvMat(const Eigen::MatrixXd &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

/**
 * @brief Eigen::Matrix<double,3,1> -> cv::Mat
 */
cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

/**
 * @brief Eigen::Matrix<float,3,1> -> cv::Mat
 */
cv::Mat Converter::toCvMat(const Eigen::Matrix<float,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
        cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

/**
 * @brief 将给定的旋转矩阵R和平移向量t转换成为 以cv::Mat格式存储的李群SE3（其实就是变换矩阵）
 */
cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    // 将旋转矩阵复制到左上角
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    // 将旋转矩阵复制到最右侧的一列
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }
    // 返回计算结果
    return cvMat.clone();
}

/**
 * @brief 将OpenCV中Mat类型的向量转化为Eigen中Matrix类型的变量
 */
Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    // 首先生成用于存储转换结果的向量
    Eigen::Matrix<double,3,1> v;
    // 然后通过逐个赋值的方法完成转换
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);
    // 返回转换结果
    return v;
}

/**
 * @brief 类似上面
 */
Eigen::Matrix<float,3,1> Converter::toVector3f(const cv::Mat &cvVector)
{
    Eigen::Matrix<float,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

/**
 * @brief 类似上面
 */
Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

/**
 * @brief cv::Mat -> Eigen::Matrix<double,3,3>
 */
Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

/**
 * @brief cv::Mat -> Eigen::Matrix<double,4,4>
 */
Eigen::Matrix<double,4,4> Converter::toMatrix4d(const cv::Mat &cvMat4)
{
    Eigen::Matrix<double,4,4> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
         cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
         cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
         cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);
    return M;
}

/**
 * @brief cv::Mat -> Eigen::Matrix<float,3,3>
 */
Eigen::Matrix<float,3,3> Converter::toMatrix3f(const cv::Mat &cvMat3)
{
    Eigen::Matrix<float,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
            cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
            cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

/**
 * @brief cv::Mat -> Eigen::Matrix<float,4,4>
 */
Eigen::Matrix<float,4,4> Converter::toMatrix4f(const cv::Mat &cvMat4)
{
    Eigen::Matrix<float,4,4> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
            cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
            cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
            cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);
    return M;
}

/**
 * @brief 旋转矩阵转四元数以vector形式输出
 */
std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

/**
 * @brief 反对称
 */
cv::Mat Converter::tocvSkewMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<
        0,               -v.at<float>(2),  v.at<float>(1),
        v.at<float>(2),                0, -v.at<float>(0),
        -v.at<float>(1),  v.at<float>(0),              0);
}

/**
 * @brief 判断是否为旋转矩阵
 */
bool Converter::isRotationMatrix(const cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;

}

/**
 * @brief 旋转矩阵转欧拉角
 */
std::vector<float> Converter::toEuler(const cv::Mat &R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
        y = atan2(-R.at<float>(2,0), sy);
        z = atan2(R.at<float>(1,0), R.at<float>(0,0));
    }
    else
    {
        x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
        y = atan2(-R.at<float>(2,0), sy);
        z = 0;
    }

    std::vector<float> v_euler(3);
    v_euler[0] = x;
    v_euler[1] = y;
    v_euler[2] = z;

    return v_euler;
}

/**
 * @brief 转成Sophus::SE3<float>
 */
Sophus::SE3<float> Converter::toSophus(const cv::Mat &T) {
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(T.rowRange(0,3).colRange(0,3));
    Eigen::Quaternionf q(eigMat.cast<float>());

    Eigen::Matrix<float,3,1> t = toVector3d(T.rowRange(0,3).col(3)).cast<float>();

    return Sophus::SE3<float>(q,t);
}

/**
 * @brief 转成Sophus::Sim3f
 */
Sophus::Sim3f Converter::toSophus(const g2o::Sim3& S) {
    return Sophus::Sim3f(Sophus::RxSO3d((float)S.scale(), S.rotation().matrix()).cast<float>() ,
                         S.translation().cast<float>());
}

} //namespace ORB_SLAM
