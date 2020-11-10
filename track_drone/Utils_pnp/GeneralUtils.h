//
// Created by yuval on 6/14/20.
//

#ifndef PNP_USING_EIGEN_LIBRARY_GENERALUTILS_H
#define PNP_USING_EIGEN_LIBRARY_GENERALUTILS_H

#include "functional"
#include "Definitions.h"

namespace PnP{
    double find_zero_bin_search(const std::function<double(double)>& func, double min, double max, int depth);

    template<typename T>
    inline T min2(T one, T two) { return one < two ? one : two; }
}

#endif //PNP_USING_EIGEN_LIBRARY_GENERALUTILS_H
