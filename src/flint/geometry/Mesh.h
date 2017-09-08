
#pragma once

#include <vector>

#include "Geometry.h"

namespace geometry {

class MeshBase { };

template <unsigned int D, typename _GeometryType = Geometry<D>>
class Mesh : public Geometry<D>, MeshBase {
    
    public:
        using GeometryType = _GeometryType;

        void AddGeometry(GeometryType* geometry) {
            _geometries.push_back(geometry);
        }

        const std::vector<GeometryType*>& geometries() const {
            return _geometries;
        }

        virtual ~Mesh() {
            
        }

    private:
        std::vector<GeometryType*> _geometries;
};

}
