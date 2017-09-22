
#pragma once

#include <Eigen/Dense>
#include "accel/bvh/Tree.h"
#include "geometry/Triangle.h"
#include "intersection/Bound.h"
#include "intersection/Intersect.h"
#include "intersection/RayGroup.h"
#include "intersection/object/Triangle.h"

namespace intersection {
namespace BVH {

    namespace detail {
        template <typename Tree, typename Ray, IntersectionOptions Options>
        struct Intersect : public IntersectBase<Ray, Options> {

            template <int GroupSize>
            using IntersectionInfoGroup = intersection::IntersectionInfoGroup<GroupSize, Options>;

            template <int GroupSize>
            IntersectionInfoGroup<GroupSize> IntersectRayGroup(const Tree* tree, const intersection::RayGroup<Ray, GroupSize>& rays) const {
                IntersectionInfoGroup<GroupSize> intersections;

                using NodePtr = decltype(tree->GetRoot());

                struct Data {
                    std::bitset<GroupSize> mask;
                };

                auto shouldVisit = [&](NodePtr node, Data& data) -> bool {
                    const auto& bound = node->GetBound();

                    auto boundIntersection = IntersectBound(bound, rays);
                    const auto& tNear = boundIntersection[0];
                    const auto& tFar = boundIntersection[1];
                    auto boundHitMask = (tFar.array() >= 0.f) && (tNear.array() <= tFar.array());

                    for (unsigned int r = 0; r < GroupSize; ++r) {
                        data.mask.set(r, data.mask[r] && boundHitMask(r, 0));
                    }

                    return !data.mask.none();
                };

                auto visit = [&rays, &intersections](NodePtr node, Data& data) {
                    for (const auto* object : node->IterateObjects()) {
                        using IntersectObject = object::IntersectObject<typename std::remove_pointer<decltype(object)>::type, Ray, Options>;
                        const auto objectIntersections = IntersectObject::template Intersect<GroupSize>(object, rays);

                        static constexpr unsigned int kCountIndex = IntersectionInfoGroup<GroupSize>::kCountIndex;
                        static constexpr unsigned int kNearIndex = IntersectionInfoGroup<GroupSize>::kNearIndex;
                        static constexpr unsigned int kFarIndex = IntersectionInfoGroup<GroupSize>::kFarIndex;

                        intersections.col(kCountIndex) += objectIntersections.col(kCountIndex);

                        auto mask = (objectIntersections.col(kCountIndex).array() != 0.0).template cast<float>();
                        if (Options & IntersectionOptions::Nearest) {
                            intersections.col(kNearIndex) =
                                mask * objectIntersections.col(kNearIndex).array().min(intersections.col(kNearIndex).array()) +
                                (1.0 - mask) * intersections.col(kNearIndex).array();
                        }

                        if (Options & IntersectionOptions::Farthest) {
                            intersections.col(kFarIndex) =
                                mask * objectIntersections.col(kFarIndex).array().max(intersections.col(kFarIndex).array()) +
                                (1.0 - mask) * intersections.col(kFarIndex).array();
                        }
                    }
                };

                auto initialData = Data { rays.Mask() };

                auto nextData = [](NodePtr child, NodePtr parent, const Data& parentData) {
                    return parentData;
                };

                tree->PreorderDFS(shouldVisit, visit, initialData, nextData);
                return intersections;
            }
        };
    }

    template <typename Tree, typename Ray, IntersectionOptions Options = IntersectionOptions::Nearest>
    using Intersect = typename std::enable_if<std::is_base_of<accel::TreeBase, Tree>::value, detail::Intersect<Tree, Ray, Options>>::type;

}

}
