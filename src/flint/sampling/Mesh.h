
#pragma once

#include "accel/bvh/TreeBuilder.h"
#include "accel/bvh/Tree.h"
#include "core/AxisAlignedBox.h"
#include "core/Optional.h"
#include "geometry/Triangle.h"
#include "import/ObjLoader.h"
#include "intersection/Ray.h"
#include "intersection/bvh/Tree.h"
#include "sampling/Box.h"

namespace sampling {

template <typename SamplePrecision, typename Mesh>
typename std::enable_if<
    std::is_base_of<geometry::MeshBase, Mesh>::value,
    std::vector<Eigen::Array<SamplePrecision, Mesh::kDimension, 1>>
>::type SampleMesh(const Mesh* mesh, SamplePrecision minDistance, RandomGenerator rng = RandomGenerator()) {

    core::Optional<core::AxisAlignedBox<3, SamplePrecision>> boundingBox;
    for (const auto* geometry : mesh->geometries()) {
        Merge(boundingBox, geometry->getAxisAlignedBound());
    }

    std::vector<Eigen::Array<float, 3, 1>> samples = sampling::SampleBox<SamplePrecision>(*boundingBox, minDistance);
    std::cout << "Generated " << samples.size() << " samples" << std::endl;

    struct MortonSample {
        Eigen::Array<SamplePrecision, Mesh::kDimension, 1> sample;
        uint64_t mortonCode;
    };

    std::vector<MortonSample> sortedSamples(samples.size());
    for (unsigned int i = 0; i < samples.size(); ++i) {
        sortedSamples[i].sample = samples[i];
        uint64_t mortonCode = 0;
        for (unsigned n = 0; n < Mesh::kDimension; ++n) {
            auto t01 = (samples[i](n, 0) - boundingBox->min(n)) / boundingBox->Extent(n);
            uint64_t val = static_cast<uint64_t>(t01 * UINT64_MAX);
            for (uint64_t b = 0; b < (sizeof(uint64_t) * CHAR_BIT) / Mesh::kDimension; ++b) {
                auto mask = static_cast<uint64_t>(1) << b;
                auto position = (Mesh::kDimension - n) * b;
                mortonCode |= val & mask << position;
            }
            sortedSamples[i].mortonCode = mortonCode;
        }
    }

    std::sort(sortedSamples.begin(), sortedSamples.end(), [](const auto& s0, const auto& s1) {
        return s0.mortonCode < s1.mortonCode;
    });

    using Ray = intersection::Ray<Mesh::kDimension, SamplePrecision>;
    std::vector<Ray> rays(samples.size());

    for (unsigned int i = 0; i < samples.size(); ++i) {
        rays[i].Origin() = sortedSamples[i].sample;
        rays[i].Direction() = Ray::Vector::Zero();
        rays[i].Direction()(0, 0) = 1;
    }

    static constexpr unsigned int kRayGroupSize = 4;

    auto rayGroups = intersection::CreateRayGroups<kRayGroupSize>(rays.data(), static_cast<unsigned int>(rays.size()));

    using GeometryType = typename Mesh::GeometryType;

    unsigned int hitCount = 0;

    {
        accel::BVH::TreeBuilder<GeometryType*, 2, 1> treeBuilder;
        auto* tree = treeBuilder.Build(mesh->geometries().begin(), mesh->geometries().end());
        using Tree = typename std::remove_pointer<decltype(tree)>::type;

        for (unsigned int i = 0; i < rayGroups.size(); ++i) {
			intersection::BVH::Intersect<Tree, Ray, intersection::IntersectionOptions::Count> intersect;
            auto intersections = intersect.IntersectRayGroup(tree, rayGroups[i]);

            using array_t = decltype(intersections.col(0).array());
            auto inside = intersections.col(0).array().cwiseProduct(array_t::Constant(0.5)).floor().cwiseProduct(array_t::Constant(2.f)).cwiseNotEqual(intersections.col(0).array());

            for (unsigned int r = 0; r < kRayGroupSize; ++r) {
                if (inside(r, 0)) {
                    samples[hitCount++] = sortedSamples[4*i + r].sample;
                }
            }
        }

        delete tree;
    }

    std::cout << hitCount << " hits" << std::endl;
    samples.resize(hitCount);

    return samples;
}

}
