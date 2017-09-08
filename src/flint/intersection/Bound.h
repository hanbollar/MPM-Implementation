
#pragma once

#include <array>
#include <Eigen/Dense>
#include "core/AxisAlignedBox.h"
#include "Ray.h"
#include "RayGroup.h"

namespace intersection {

template<int GroupSize, typename Ray, typename T>
std::array<Eigen::Array<float, GroupSize, 1>, 2> IntersectBound(const core::AxisAlignedBox<Ray::kDimension, T> &bound, const intersection::RayGroup<Ray, GroupSize>& rays) {
    Eigen::Array<float, GroupSize, Ray::kDimension> t0;
    Eigen::Array<float, GroupSize, Ray::kDimension> t1;
    Eigen::Array<float, GroupSize, Ray::kDimension> tMin;
    Eigen::Array<float, GroupSize, Ray::kDimension> tMax;

    for (unsigned int n = 0; n < Ray::kDimension; ++n) {
        t0.col(n) = (bound.min(n) - rays.Origins(n).array()) * rays.Directions(n).array();
        t1.col(n) = (bound.max(n) - rays.Origins(n).array()) * rays.Directions(n).array();
    }
    for (unsigned int n = 0; n < Ray::kDimension; ++n) {
        tMin.col(n) = t0.col(n).min(t1.col(n));
        tMax.col(n) = t0.col(n).max(t1.col(n));
    }

    std::array<Eigen::Array<float, GroupSize, 1>, 2> result;

    Eigen::Array<float, GroupSize, 1>& tNear = result[0];
    Eigen::Array<float, GroupSize, 1>& tFar = result[1];

    tNear = tMin.col(0);
    tFar = tMax.col(0);

    for (unsigned int n = 1; n < Ray::kDimension; ++n) {
        tNear = tNear.max(tMin.col(n));
        tFar = tFar.min(tMax.col(n));
    }

    return result;
}

}