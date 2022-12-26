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

#ifndef ORB_SLAM3_OPTIMIZABLETYPES_H
#define ORB_SLAM3_OPTIMIZABLETYPES_H

#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <Thirdparty/g2o/g2o/types/sim3.h>

#include <Eigen/Geometry>
#include <include/CameraModels/GeometricCamera.h>

namespace ORB_SLAM3
{
// 左目纯位姿优化的边，左目点的重投影误差相对于左目位姿
class EdgeSE3ProjectXYZOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose() {}

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project(v1->estimate().map(Xw));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return (v1->estimate().map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera *pCamera;
};

// 两个相机中的右目上的重投影误差与左目位姿的边
class EdgeSE3ProjectXYZOnlyPoseToBody : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPoseToBody() {}

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project((mTrl * v1->estimate()).map(Xw));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera *pCamera;

    g2o::SE3Quat mTrl;
};


// 左目纯位姿优化的边，左目点的重投影误差相对于左目位姿以及三维点
class EdgeSE3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ();

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2) > 0.0);
    }

    virtual void linearizeOplus();

    GeometricCamera *pCamera;
};

// 两个相机中的右目上的重投影误差与左目位姿以及三维点的边
class EdgeSE3ProjectXYZToBody : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZToBody();

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(v2->estimate()))(2) > 0.0;
    }

    virtual void linearizeOplus();

    GeometricCamera *pCamera;
    g2o::SE3Quat mTrl;
};

// sim3节点
class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3Expmap();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    // 原始值
    virtual void setToOriginImpl()
    {
        _estimate = g2o::Sim3();
    }

    // 更新
    virtual void oplusImpl(const double *update_)
    {
        Eigen::Map<g2o::Vector7d> update(const_cast<double *>(update_));

        if (_fix_scale)
            update[6] = 0;

        g2o::Sim3 s(update);
        setEstimate(s * estimate());
    }

    GeometricCamera *pCamera1, *pCamera2;

    bool _fix_scale;
};

// sim3边
class EdgeSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, ORB_SLAM3::VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZ();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap *v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs - v1->pCamera1->project(v1->estimate().map(v2->estimate()));
    }

    // 自动求导，没错g2o也有自动求导
    // virtual void linearizeOplus();
};

// sim3反投的边
class EdgeInverseSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap *v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs - v1->pCamera2->project((v1->estimate().inverse().map(v2->estimate())));
    }

    // 自动求导，没错g2o也有自动求导
    // virtual void linearizeOplus();
};

}

#endif // ORB_SLAM3_OPTIMIZABLETYPES_H
