
#pragma once

#include <Eigen/Dense>

namespace intersection {

template<int D, typename _Precision>
class Ray {
    
    public:
        static constexpr unsigned int kDimension = D;
        using Precision = _Precision;
        using Vector = Eigen::Matrix<Precision, kDimension, 1>;
        
        Ray() {

        }
        
        Ray(const Precision* originData, const Precision* directionData) 
          : origin(originData), direction(directionData) {

        }

        const Vector& Origin() const {
            return origin;
        }

        const Vector& Direction() const {
            return direction;
        }

        Vector& Origin() {
            return origin;
        }

        Vector& Direction() {
            return direction;
        }

    private:
        Vector origin;
        Vector direction;
};

}