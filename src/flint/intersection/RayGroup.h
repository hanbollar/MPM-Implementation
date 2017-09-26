
#pragma once

#include <bitset>
#include <Eigen/Dense>
#include "Ray.h"

namespace intersection {

template<typename Ray, int N>
class RayGroup  {

    public:
        using Precision = typename Ray::Precision;
        static constexpr unsigned int kDimension = Ray::kDimension;

    private:
        using data_t = Eigen::Matrix<Precision, N, 2*kDimension>;
        using Scalar = typename data_t::Scalar;
        using Index = typename data_t::Index;

    public:
        template <typename RayIterator>
        RayGroup(RayIterator begin, RayIterator end) {
            for (unsigned int i = 0; i < (end - begin); ++i) {
                Origin(i) = (begin + i)->Origin();
                Direction(i) = (begin + i)->Direction();
                _mask.set(i, true);
            }
            for (unsigned int i = static_cast<unsigned int>(end - begin); i < N; ++i) {
                _mask.set(i, false);
            }
        }

        const Eigen::Transpose<const typename data_t::template ConstFixedBlockXpr<1, kDimension>::Type> Origin(Index row) const {
            return _data.template block<1, kDimension>(row, 0, 1, kDimension).transpose();
        }

        const Eigen::Transpose<const typename data_t::template ConstFixedBlockXpr<1, kDimension>::Type> Direction(Index row) const {
            return _data.template block<1, kDimension>(row, kDimension, 1, kDimension).transpose();
        }

        Eigen::Transpose<typename data_t::template FixedBlockXpr<1, kDimension>::Type> Origin(Index row) {
            return _data.template block<1, kDimension>(row, 0, 1, kDimension).transpose();
        }

        Eigen::Transpose<typename data_t::template FixedBlockXpr<1, kDimension>::Type> Direction(Index row) {
            return _data.template block<1, kDimension>(row, kDimension, 1, kDimension).transpose();
        }

        const data_t& Data() const {
            return _data;
        }

        typename data_t::ConstColXpr Origins(Index col) const {
            return _data.col(col);
        }

        typename data_t::ConstColXpr Directions(Index col) const {
            return _data.col(kDimension + col);
        }

        typename data_t::ColXpr Origins(Index col) {
            return _data.col(col);
        }

        typename data_t::ColXpr Directions(Index col) {
            return _data.col(kDimension + col);
        }

        typename data_t::template ConstFixedBlockXpr<N, kDimension>::Type Origins() const {
            return _data.template block<N, kDimension>(0, 0, _data.rows(), kDimension);
        }
        
        typename data_t::template ConstFixedBlockXpr<N, kDimension>::Type Directions() const {
            return _data.template block<N, kDimension>(0, kDimension, _data.rows(), kDimension);
        }

        typename data_t::template FixedBlockXpr<N, kDimension>::Type Origins() {
            return _data.template block<N, kDimension>(0, 0, _data.rows(), kDimension);
        }
        
        typename data_t::template FixedBlockXpr<N, kDimension>::Type Directions() {
            return _data.template block<N, kDimension>(0, kDimension, _data.rows(), kDimension);
        }

        const std::bitset<N>& Mask() const {
            return _mask;
        }

    private:
        data_t _data;
        std::bitset<N> _mask;
};

template <int GroupSize, typename Ray>
std::vector<RayGroup<Ray, GroupSize>> CreateRayGroups(const Ray* rays, unsigned int count) {
    std::vector<RayGroup<Ray, GroupSize>> groups;
    unsigned int groupCount = (count + GroupSize - 1) / GroupSize;
    groups.reserve(groupCount);
    for (unsigned int i = 0; i < count; i += GroupSize) {
        unsigned int j = i + GroupSize;
        j = j > count ? count : j;
        groups.emplace_back(&rays[i], &rays[j]);
    }
    return groups;
}

}
