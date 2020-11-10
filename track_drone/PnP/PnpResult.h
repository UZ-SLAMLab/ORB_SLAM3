//
// Created by yuval on 7/12/20.
//

#ifndef PNP_SOLVER_PNPRESULT_H
#define PNP_SOLVER_PNPRESULT_H

#include <memory>
#include "Definitions.h"
#include "PnpObjective.h"

namespace PnP{
    class PnpResult {
    public:
        Eigen::Vector4d quaternions;
        std::shared_ptr<PnpObjective> pnp_objective;

        PnpResult(Eigen::Vector4d quats, std::shared_ptr<PnpObjective> objective)
            :quaternions(std::move(quats)), pnp_objective(std::move(objective)){}

        double cost();
        Eigen::Matrix3d rotation_matrix();
        Eigen::Vector3d translation_vector();
    };
}



#endif //PNP_SOLVER_PNPRESULT_H
