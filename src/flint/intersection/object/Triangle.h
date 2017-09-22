
#pragma once

#include "intersection/Intersect.h"

namespace intersection {
namespace object {

    namespace detail {
        template <typename Triangle, typename Ray, IntersectionOptions Options>
        struct IntersectTriangle : public IntersectBase<Ray, Options> {

            template <unsigned int GroupSize>
            static intersection::IntersectionInfoGroup<GroupSize, Options> Intersect(const Triangle* triangle, const intersection::RayGroup<Ray, GroupSize>& rays) {
				using IntersectionInfo = intersection::IntersectionInfoGroup<GroupSize, Options>;
				IntersectionInfo intersections;
                
                const auto& points = triangle->getPoints();
                
                using Scalar = typename Ray::Precision;
                using ColVec = Eigen::Matrix<Scalar, Ray::kDimension, 1>;
                using RowVec = Eigen::Matrix<Scalar, 1, GroupSize>;
                using ColVecN = Eigen::Matrix<Scalar, Ray::kDimension, GroupSize>;
                
                ColVec e0 = points.col(1) - points.col(0);
                ColVec e1 = points.col(2) - points.col(0);
                
                ColVecN T, P, Q;
            
                for (unsigned int r = 0; r < GroupSize; ++r) {
                    T.col(r) = rays.Origin(r) - points.col(0);
                }
                
                auto directions = rays.Directions().transpose();

                
                for (unsigned int r = 0; r < GroupSize; ++r) {
                    P.col(r) = directions.col(r).cross(e1);
                    Q.col(r) = T.col(r).cross(e0);
                }
                
                RowVec det, denom, u, v, t;

                for (unsigned int r = 0; r < GroupSize; ++r) {
                    det(0, r) = P.col(r).dot(e0);
                    denom(0, r) = 1 / det(0, r);
                }
                
                for (unsigned int r = 0; r < GroupSize; ++r) {
                    u(0, r) = T.col(r).dot(P.col(r));
                    v(0, r) = Q.col(r).dot(directions.col(r));
                    t(0, r) = Q.col(r).dot(e1);
                }
                
                u = u.cwiseProduct(denom);
                v = v.cwiseProduct(denom);
                t = t.cwiseProduct(denom);

                intersections.col(IntersectionInfo::kCountIndex) = (
                    (u.array() >= RowVec::Zero().array()) && 
                    (u.array() < RowVec::Ones().array()) &&
                    (v.array() >= RowVec::Zero().array()) &&
                    (u.array() + v.array() < RowVec::Ones().array()) &&
                    (det.array().abs() > RowVec::Constant(static_cast<Scalar>(0.00001)).array()) &&
                    (t.array() >= RowVec::Zero().array())
                ).transpose().template cast<float>();

                if (Options & IntersectionOptions::Nearest) {
                    intersections.col(IntersectionInfo::kNearIndex) = t.transpose();
                }

                if (Options & IntersectionOptions::Farthest) {
                    intersections.col(IntersectionInfo::kFarIndex) = t.transpose();
                }

                return intersections;
            }
            
        };
    }

    template <typename Object, typename Ray, IntersectionOptions Options>
    using IntersectObject =
    typename std::conditional<
        std::is_base_of<geometry::TriangleBase, Object>::value,
        detail::IntersectTriangle<typename std::decay<Object>::type, Ray, Options>,

    typename std::conditional<
        std::is_base_of<geometry::TriangleBase, Object>::value,
        detail::IntersectTriangle<typename std::decay<Object>::type, Ray, Options>,
        void
    >::type
    >::type;

}
}