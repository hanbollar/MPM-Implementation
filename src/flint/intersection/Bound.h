
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
        auto mask = (rays.Directions(n).array() != 0.0);

        using DimensionDistance = Eigen::Array<float, GroupSize, 1>;

        Eigen::Select<decltype(mask), DimensionDistance, DimensionDistance> select0(
            mask,
            (bound.min(n) - rays.Origins(n).array()) / rays.Directions(n).array(),
            DimensionDistance::Constant(0)
        );

        Eigen::Select<decltype(mask), DimensionDistance, DimensionDistance> select1(
            mask,
            (bound.max(n) - rays.Origins(n).array()) / rays.Directions(n).array(),
            DimensionDistance::Constant(0)
        );

        t0.col(n) = select0;
        t1.col(n) = select1;
    }

    using Distance = Eigen::Array<float, GroupSize, 1>;

    for (unsigned int n = 0; n < Ray::kDimension; ++n) {
        auto mask = (rays.Directions(n).array() != 0.0).template cast<float>();
        tMin.col(n) = mask * t0.col(n).min(t1.col(n)) + (1.0 - mask) * Distance::Constant(std::numeric_limits<float>::max());
        tMax.col(n) = mask * t0.col(n).max(t1.col(n)) + (1.0 - mask) * Distance::Constant(std::numeric_limits<float>::lowest());
    }

    std::array<Distance, 2> result;
    Distance& tNear = result[0];
    Distance& tFar = result[1];

    Eigen::Array<bool, GroupSize, 1> initialized = Eigen::Array<bool, GroupSize, 1>::Constant(false);
    for (unsigned int n = 0; n < Ray::kDimension; ++n) {
        auto mask = (rays.Directions(n).array() != 0.0);

        for (unsigned int r = 0; r < GroupSize; ++r) {
            if (mask(r, 0)) {
                if (!initialized(r, 0)) {
                    tNear = tMin(r, n);
                    tFar = tMax(r, n);
                    initialized(r, 0) = true;
                } else {
                    tNear = tNear.max(tMin.col(n));
                    tFar = tFar.min(tMax.col(n));
                }
            }
        }
    }

    return result;
}

}
