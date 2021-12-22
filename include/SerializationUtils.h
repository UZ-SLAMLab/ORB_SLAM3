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

#ifndef SERIALIZATION_UTILS_H
#define SERIALIZATION_UTILS_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <sophus/se3.hpp>
#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>

namespace ORB_SLAM3
{

template <class Archive>
void serializeSophusSE3(Archive &ar, Sophus::SE3f &T, const unsigned int version)
{
    Eigen::Vector4f quat;
    Eigen::Vector3f transl;

    if (Archive::is_saving::value)
    {
        Eigen::Quaternionf q = T.unit_quaternion();
        quat << q.w(), q.x(), q.y(), q.z();
        transl = T.translation();
    }

    ar & boost::serialization::make_array(quat.data(), quat.size());
    ar & boost::serialization::make_array(transl.data(), transl.size());

    if (Archive::is_loading::value)
    {
        Eigen::Quaternionf q(quat[0], quat[1], quat[2], quat[3]);
        T = Sophus::SE3f(q, transl);
    }
}

/*template <class Archive, size_t dim>
void serializeDiagonalMatrix(Archive &ar, Eigen::DiagonalMatrix<float, dim> &D, const unsigned int version)
{
    Eigen::Matrix<float,dim,dim> dense;
    if(Archive::is_saving::value)
    {
        dense = D.toDenseMatrix();
    }

    ar & boost::serialization::make_array(dense.data(), dense.size());

    if (Archive::is_loading::value)
    {
        D = dense.diagonal().asDiagonal();
    }
}*/

template<class Archive>
void serializeMatrix(Archive& ar, cv::Mat& mat, const unsigned int version)
{
    int cols, rows, type;
    bool continuous;

    if (Archive::is_saving::value) {
        cols = mat.cols; rows = mat.rows; type = mat.type();
        continuous = mat.isContinuous();
    }

    ar & cols & rows & type & continuous;

    if (Archive::is_loading::value)
        mat.create(rows, cols, type);

    if (continuous) {
        const unsigned int data_size = rows * cols * mat.elemSize();
        ar & boost::serialization::make_array(mat.ptr(), data_size);
    } else {
        const unsigned int row_size = cols*mat.elemSize();
        for (int i = 0; i < rows; i++) {
            ar & boost::serialization::make_array(mat.ptr(i), row_size);
        }
    }
}

template<class Archive>
void serializeMatrix(Archive& ar, const cv::Mat& mat, const unsigned int version)
{
    cv::Mat matAux = mat;

    serializeMatrix(ar, matAux,version);

    if (Archive::is_loading::value)
    {
        cv::Mat* ptr;
        ptr = (cv::Mat*)( &mat );
        *ptr = matAux;
    }
}

template<class Archive>
void serializeVectorKeyPoints(Archive& ar, const std::vector<cv::KeyPoint>& vKP, const unsigned int version)
{
    int NumEl;

    if (Archive::is_saving::value) {
        NumEl = vKP.size();
    }

    ar & NumEl;

    std::vector<cv::KeyPoint> vKPaux = vKP;
    if (Archive::is_loading::value)
        vKPaux.reserve(NumEl);

    for(int i=0; i < NumEl; ++i)
    {
        cv::KeyPoint KPi;

        if (Archive::is_loading::value)
            KPi = cv::KeyPoint();

        if (Archive::is_saving::value)
            KPi = vKPaux[i];

        ar & KPi.angle;
        ar & KPi.response;
        ar & KPi.size;
        ar & KPi.pt.x;
        ar & KPi.pt.y;
        ar & KPi.class_id;
        ar & KPi.octave;

        if (Archive::is_loading::value)
            vKPaux.push_back(KPi);
    }


    if (Archive::is_loading::value)
    {
        std::vector<cv::KeyPoint> *ptr;
        ptr = (std::vector<cv::KeyPoint>*)( &vKP );
        *ptr = vKPaux;
    }
}

} // namespace ORB_SLAM3

#endif // SERIALIZATION_UTILS_H
