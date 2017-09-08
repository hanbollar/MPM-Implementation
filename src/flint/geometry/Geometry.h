
#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include "core/Optional.h"
#include "core/AxisAlignedBox.h"

namespace geometry {

class GeometryBase {
  static uint64_t NextGeometryID;
  uint64_t id;

  public:
    GeometryBase();
    uint64_t getGeometryID() const;
};

using precision_t = float;

template <unsigned int D>
class Geometry : public GeometryBase {

  public:
    static constexpr unsigned int kDimension = D;

    virtual core::Optional<core::AxisAlignedBox<D, precision_t>> getAxisAlignedBound() const {
      return core::Optional<core::AxisAlignedBox<D, precision_t>>();
    }

    virtual core::Optional<Eigen::Matrix<precision_t, D, 1>> getCentroid() const {
      return core::Optional<Eigen::Matrix<precision_t, D, 1>>();
    }

    virtual ~Geometry() {

    }

};

}
