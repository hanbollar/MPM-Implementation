
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
        // struct Intersect : public intersection::Intersect<Tree, Ray, Options, Intersect<Tree, Ray, Options>> {
            // using Base = intersection::Intersect<Tree, Ray, Options, Intersect<Tree, Ray, Options>>;

            template <int GroupSize>
            using IntersectionInfoGroup = intersection::IntersectionInfoGroup<GroupSize, Options>;

            // template <int GroupSize, typename RayIterator>
            // IntersectionInfoGroup<GroupSize> IntersectRaysImpl(const Tree* tree, RayIterator rayBegin, RayIterator rayEnd) const {
            //     using RayGroup = intersection::RayGroup<Ray, GroupSize>;
            //     using Intersection = intersection::IntersectionInfoGroup<GroupSize, Options>;
            //     Intersection intersections(rayEnd - rayBegin);
                
            //     std::vector<RayGroup> groups;
            //     unsigned int groupCount = (rayEnd - rayBegin + GroupSize - 1) / GroupSize;
            //     groups.reserve(groupCount);

            //     for (auto i = rayBegin; i < rayEnd; i += GroupSize) {
            //         auto end = i + GroupSize;
            //         groups.emplace_back(i, end > rayEnd ? rayEnd : end);
            //     }

            //     for (unsigned int i = 0; i < groupCount; ++i) {
            //         intersections.block(GroupSize*i, Intersection::kCountIndex, GroupSize, 1) = IntersectRayGroup<GroupSize>(tree, groups[i]);
            //     }

            //     return intersections;
            // }

            template <int GroupSize>
            IntersectionInfoGroup<GroupSize> IntersectRayGroup(const Tree* tree, const intersection::RayGroup<Ray, GroupSize>& rays) const {
                IntersectionInfoGroup<GroupSize> intersections;

                const auto* current = tree->GetRoot();

                struct Node {
                    decltype(current) node;
                    std::bitset<GroupSize> mask;
                };

                std::vector<Node> stack;
                stack.emplace_back(Node { current, rays.Mask() });
                
                do {
                    auto current = stack.back();
                    stack.pop_back();
                    const auto& bound = current.node->GetBound();
                    
                    auto boundIntersection = IntersectBound(bound, rays);
                    const auto& tNear = boundIntersection[0];
                    const auto& tFar = boundIntersection[1];

                    for (unsigned int r = 0; r < GroupSize; ++r) {
                        current.mask.set(r, current.mask[r] && (tNear(r, 0) <= tFar(r, 0)));
                    }
                    
                    if (current.mask.none()) {
                        continue;
                    } else {
                        for (const auto* _child : current.node->IterateChildren()) {
                            const auto* child = static_cast<const typename Tree::Node*>(_child);
                            stack.emplace_back(Node { child, current.mask });
                        }

                        for (const auto* object : current.node->IterateObjects()) {
                            using IntersectObject = object::IntersectObject<typename std::remove_pointer<decltype(object)>::type, Ray, Options>;
                            const auto objectIntersections = IntersectObject::template Intersect<GroupSize>(object, rays);
                            
                            intersections.col(IntersectionInfoGroup<GroupSize>::kCountIndex) += objectIntersections.col(IntersectionInfoGroup<GroupSize>::kCountIndex);
                            
                            if (Options & IntersectionOptions::Nearest) {
                                intersections.col(IntersectionInfoGroup<GroupSize>::kNearIndex) = objectIntersections.col(IntersectionInfoGroup<GroupSize>::kNearIndex).array().min(intersections.col(IntersectionInfoGroup<GroupSize>::kNearIndex).array());
                            }

                            if (Options & IntersectionOptions::Farthest) {
                                intersections.col(IntersectionInfoGroup<GroupSize>::kFarIndex) = objectIntersections.col(IntersectionInfoGroup<GroupSize>::kFarIndex).array().max(intersections.col(IntersectionInfoGroup<GroupSize>::kFarIndex).array());
                            }
                        }
                    }
                } while(stack.size() > 0);

                return intersections;
            }
        };
    }

    template <typename Tree, typename Ray, IntersectionOptions Options = IntersectionOptions::Nearest>
    using Intersect = typename std::enable_if<std::is_base_of<accel::TreeBase, Tree>::value, detail::Intersect<Tree, Ray, Options>>::type;

}

}