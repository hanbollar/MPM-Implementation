
#pragma once

#include <array>
#include <numeric>
#include <Eigen/Dense>
#include "Geometry.h"

namespace geometry {

template <unsigned int N, unsigned int D, typename T = precision_t>
class FixedPolygon : public Geometry<D> {

    protected:
        Eigen::Matrix<T, D, N> points;

    public:

        FixedPolygon(const std::array<Eigen::Matrix<T, D, 1>, N> &points) {
            for (unsigned int i = 0; i < N; ++i) {
                this->points.col(i) = points[i];
            }
        }

        FixedPolygon(const Eigen::Matrix<T, D, 3> &points) : points(points) {
        }

        const Eigen::Matrix<T, D, N>& getPoints() const {
            return points;
        }

        Eigen::Matrix<T, D, N>& getPoints() {
            return points;
        }

        core::Optional<core::AxisAlignedBox<D, T>> getAxisAlignedBound() const override {
            core::Optional<core::AxisAlignedBox<D, T>> bound;
            if (N == 0) return bound;

            Eigen::Matrix<T, D, 1> m = points.col(0);
            Merge<D, T>(bound, m);
            for (unsigned int i = 1; i < N; ++i) {
                m = points.col(i);
                Merge<D, T>(*bound, m);
            }
            return bound;
        }

        core::Optional<Eigen::Matrix<T, D, 1>> getCentroid() const override {
            core::Optional<Eigen::Matrix<T, D, 1>> centroid;
            if (N == 0) return centroid;

            centroid.set(points.col(0));
            for (unsigned int i = 1; i < N; ++i) {
                *centroid += points.col(i);
            }
            *centroid /= N;
            return centroid;
        }

        virtual ~FixedPolygon() {

        }
};

}
