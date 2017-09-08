
#include "Geometry.h"

namespace geometry {
  uint64_t GeometryBase::NextGeometryID = 0;

  GeometryBase::GeometryBase() : id(GeometryBase::NextGeometryID++) { }

  uint64_t GeometryBase::getGeometryID() const {
    return id;
  }
}
