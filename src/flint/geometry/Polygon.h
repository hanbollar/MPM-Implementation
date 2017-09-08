
#pragma once

#include <vector>
#include <Eigen/Dense>
#include "Geometry.h"

namespace geometry {

template <unsigned int D, typename T = precision_t>
class Polygon : public Geometry<D> {
    std::vector<Eigen::Matrix<T, D, 1>> points;

    public:
        core::Optional<core::AxisAlignedBox<D, T>> getAxisAlignedBound() const override {
            core::Optional<core::AxisAlignedBox<D, T>> bound;
            if (points.size() == 0) return bound;

            Merge(bound, points[0]);
            std::for_each(points.begin() + 1, points.end(), [&bound](const auto &point) {
                Merge(*bound, point);
            });
            return bound;
        }

        core::Optional<Eigen::Matrix<T, D, 1>> getCentroid() const override {
            core::Optional<Eigen::Matrix<T, D, 1>> centroid;
            if (points.size() == 0) return centroid;

            *centroid = points[0];
            std::for_each(points.begin() + 1, points.end(), [&centroid](const auto &point) {
                *centroid += point;
            });
            *centroid /= points.size();
            return centroid;
        }
};

}
