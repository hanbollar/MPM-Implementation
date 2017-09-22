
#pragma once

#include "Ray.h"

namespace intersection {

enum IntersectionOptions {
    Nearest = 1 << 0,
    Farthest = 1 << 1,
    Count = 1 << 2,
};

template <int N, IntersectionOptions Options>
struct IntersectionInfoGroup : public Eigen::Matrix<float, N, (Options & IntersectionOptions::Nearest) + (Options & IntersectionOptions::Farthest) + 1> {
    
    using Base = Eigen::Matrix<float, N, (Options & IntersectionOptions::Nearest) + (Options & IntersectionOptions::Farthest) + 1>;
    
    static constexpr unsigned int kNearIndex = 0;
    static constexpr unsigned int kFarIndex = kNearIndex + (Options & IntersectionOptions::Nearest);
    static constexpr unsigned int kCountIndex = kFarIndex + (Options & IntersectionOptions::Farthest);
    static constexpr unsigned int kColumns = kCountIndex + 1;

    IntersectionInfoGroup() : Base(Base::Zero()) { }
    IntersectionInfoGroup(unsigned int size) : Base(Base::Zero(size, 1)) {
    }
};

// template <typename Object, typename Ray, IntersectionOptions Options, typename Impl>
// class Intersect {

//     public:
//         template <int GroupSize, typename RayIterator>
//         typename std::enable_if<std::is_same<typename std::iterator_traits<RayIterator>::value_type, Ray>::value, IntersectionInfoGroup<GroupSize, Options>>::type
//         IntersectRays(const Object* object, RayIterator rayBegin, RayIterator rayEnd) {
//             return static_cast<const Impl*>(this)->template IntersectRaysImpl<GroupSize>(object, rayBegin, rayEnd);
//         }
// };

template <typename Ray, IntersectionOptions _Options>
struct IntersectBase {
    static constexpr IntersectionOptions Options = _Options;

    template <int N>
    using IntersectionInfoGroup = intersection::IntersectionInfoGroup<N, Options>;
};

}
