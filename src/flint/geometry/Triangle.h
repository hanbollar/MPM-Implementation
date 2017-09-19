
#pragma once

#include "FixedPolygon.h"

namespace geometry {

class TriangleBase { };

template <unsigned int D, typename T = precision_t>
class Triangle : public FixedPolygon<3, D, T>, TriangleBase {

    public:
        Triangle(const std::array<Eigen::Matrix<T, D, 1>, 3> &points) : FixedPolygon<3, D, T>(points) {

        }

        Triangle(const Eigen::Matrix<T, D, 3> &points) : FixedPolygon<3, D, T>(points) {

        }
};

}
