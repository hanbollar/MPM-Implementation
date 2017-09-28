
#include <array>
#include <gtest/gtest.h>
#include "flint/core/AxisAlignedBox.h"
#include "flint/intersection/Bound.h"
#include "flint/intersection/RayGroup.h"

using namespace core;
using namespace intersection;

TEST(AxisAlignedBoxTest, Inside) {
    AxisAlignedBox<3, float> box({ -1, -1, -1 }, { 1, 1, 1 });
    std::array<Ray<3, float>, 14> rays;

    for (unsigned int i = 0; i < 14; ++i) {
        rays[i].Origin() = { 0, 0, 0 };
    }

    rays[0].Direction() = { 1, 0, 0 };
    rays[1].Direction() = {-1, 0, 0 };
    rays[2].Direction() = { 0, 1, 0 };
    rays[3].Direction() = { 0,-1, 0 };
    rays[4].Direction() = { 0, 0, 1 };
    rays[5].Direction() = { 0, 0,-1 };

    static const float c = static_cast<float>(1 / std::sqrt(3));
    rays[6].Direction() = { c, c, c };
    rays[7].Direction() = { -c, c, c };
    rays[8].Direction() = { -c, -c, c };
    rays[9].Direction() = { c, -c, c };
    rays[10].Direction() = { c, c, -c };
    rays[11].Direction() = { -c, c, -c };
    rays[12].Direction() = { -c, -c, -c };
    rays[13].Direction() = { c, -c, -c };

    RayGroup<Ray<3, float>, 14> rayGroup(rays.begin(), rays.end());

    auto intersections = IntersectBound(box, rayGroup);

    for (unsigned int i = 0; i < 6; ++i) {
        auto tNear = intersections[0](i, 0);
        auto tFar = intersections[1](i, 0);
        EXPECT_FLOAT_EQ(tNear, -1.f);
        EXPECT_FLOAT_EQ(tFar, 1.f);
    }

    for (unsigned int i = 6; i < 14; ++i) {
        auto tNear = intersections[0](i, 0);
        auto tFar = intersections[1](i, 0);
        EXPECT_FLOAT_EQ(tNear, -static_cast<float>(std::sqrt(3)));
        EXPECT_FLOAT_EQ(tFar, static_cast<float>(std::sqrt(3)));
    }
}

TEST(AxisAlignedBoxTest, OutsideMiss) {
    AxisAlignedBox<3, float> box({ -0.1f, -0.1f, -0.1f }, { 0.1f, 0.1f, 0.1f });
    std::array<Ray<3, float>, 14> rays;

    rays[0].Origin() = { 1, 0, 0 };
    rays[1].Origin() = { -1, 0, 0 };
    rays[2].Origin() = { 0, 1, 0 };
    rays[3].Origin() = { 0,-1, 0 };
    rays[4].Origin() = { 0, 0, 1 };
    rays[5].Origin() = { 0, 0,-1 };

    rays[0].Direction() = { 1, 0, 0 };
    rays[1].Direction() = { -1, 0, 0 };
    rays[2].Direction() = { 0, 1, 0 };
    rays[3].Direction() = { 0,-1, 0 };
    rays[4].Direction() = { 0, 0, 1 };
    rays[5].Direction() = { 0, 0,-1 };

    static const float c = static_cast<float>(1 / std::sqrt(3));

    rays[6].Origin() = { c, c, c };
    rays[7].Origin() = { -c, c, c };
    rays[8].Origin() = { -c, -c, c };
    rays[9].Origin() = { c, -c, c };
    rays[10].Origin() = { c, c, -c };
    rays[11].Origin() = { -c, c, -c };
    rays[12].Origin() = { -c, -c, -c };
    rays[13].Origin() = { c, -c, -c };

    rays[6].Direction() = { c, c, c };
    rays[7].Direction() = { -c, c, c };
    rays[8].Direction() = { -c, -c, c };
    rays[9].Direction() = { c, -c, c };
    rays[10].Direction() = { c, c, -c };
    rays[11].Direction() = { -c, c, -c };
    rays[12].Direction() = { -c, -c, -c };
    rays[13].Direction() = { c, -c, -c };

    RayGroup<Ray<3, float>, 14> rayGroup(rays.begin(), rays.end());

    auto intersections = IntersectBound(box, rayGroup);

    for (unsigned int i = 0; i < 14; ++i) {
        auto tNear = intersections[0](i, 0);
        auto tFar = intersections[1](i, 0);
        EXPECT_LT(tNear, 0);
        EXPECT_LT(tFar, 0);
    }
}

TEST(AxisAlignedBoxTest, OutsideHit) {
    AxisAlignedBox<3, float> box({ -0.1f, -0.1f, -0.1f }, { 0.1f, 0.1f, 0.1f });
    std::array<Ray<3, float>, 14> rays;

    rays[0].Origin() = { 1, 0, 0 };
    rays[1].Origin() = { -1, 0, 0 };
    rays[2].Origin() = { 0, 1, 0 };
    rays[3].Origin() = { 0,-1, 0 };
    rays[4].Origin() = { 0, 0, 1 };
    rays[5].Origin() = { 0, 0,-1 };

    rays[0].Direction() = {-1, 0, 0 };
    rays[1].Direction() = { 1, 0, 0 };
    rays[2].Direction() = { 0,-1, 0 };
    rays[3].Direction() = { 0, 1, 0 };
    rays[4].Direction() = { 0, 0,-1 };
    rays[5].Direction() = { 0, 0, 1 };

    static const float c = static_cast<float>(1 / std::sqrt(3));

    rays[6].Origin() = { c, c, c };
    rays[7].Origin() = { -c, c, c };
    rays[8].Origin() = { -c, -c, c };
    rays[9].Origin() = { c, -c, c };
    rays[10].Origin() = { c, c, -c };
    rays[11].Origin() = { -c, c, -c };
    rays[12].Origin() = { -c, -c, -c };
    rays[13].Origin() = { c, -c, -c };

    rays[6].Direction() = { -c, -c, -c };
    rays[7].Direction() = { c, -c, -c };
    rays[8].Direction() = { c, c, -c };
    rays[9].Direction() = { -c, c, -c };
    rays[10].Direction() = { -c, -c, c };
    rays[11].Direction() = { c, -c, c };
    rays[12].Direction() = { c, c, c };
    rays[13].Direction() = { -c, c, c };

    RayGroup<Ray<3, float>, 14> rayGroup(rays.begin(), rays.end());

    auto intersections = IntersectBound(box, rayGroup);

    for (unsigned int i = 0; i < 14; ++i) {
        auto tNear = intersections[0](i, 0);
        auto tFar = intersections[1](i, 0);
        EXPECT_GT(tNear, 0);
        EXPECT_GT(tFar, 0);
        EXPECT_GT(tFar, tNear);
    }
}

TEST(AxisAlignedBoxTest, InsideOutsideMix) {
    AxisAlignedBox<3, float> box({ -0.1f, -0.1f, -0.1f }, { 0.1f, 0.1f, 0.1f });
    std::array<Ray<3, float>, 14> rays;

    static const float c = static_cast<float>(1 / std::sqrt(3));

    auto randFloat11 = []() {
        return 2.f * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 0.5f);
    };

    rays[0].Origin() = { 0, 0, 0 };
    rays[1].Origin() = { 0, 0, 0 };
    rays[2].Origin() = { 0, 0, 0 };
    rays[3].Origin() = { 0, 0, 0 };
    rays[4].Origin() = { 0, 0, 0 };
    rays[5].Origin() = { 0, 0, 0 };

    rays[0].Direction() = { randFloat11(), randFloat11(), randFloat11() };
    rays[1].Direction() = { randFloat11(), randFloat11(), randFloat11() };
    rays[2].Direction() = { randFloat11(), randFloat11(), randFloat11() };
    rays[3].Direction() = { randFloat11(), randFloat11(), randFloat11() };
    rays[4].Direction() = { randFloat11(), randFloat11(), randFloat11() };
    rays[5].Direction() = { randFloat11(), randFloat11(), randFloat11() };

    // Outside the box
    rays[6].Origin() = { c, c, c };
    rays[7].Origin() = { -c, c, c };
    rays[8].Origin() = { -c, -c, c };
    rays[9].Origin() = { c, -c, c };
    rays[10].Origin() = { c, c, -c };
    rays[11].Origin() = { -c, c, -c };
    rays[12].Origin() = { -c, -c, -c };
    rays[13].Origin() = { c, -c, -c };

    // Outward
    rays[6].Direction() = { c, c, c };
    rays[7].Direction() = { -c, c, c };
    rays[8].Direction() = { -c, -c, c };
    rays[9].Direction() = { c, -c, c };

    // Inward
    rays[10].Direction() = { -c, -c, c };
    rays[11].Direction() = { c, -c, c };
    rays[12].Direction() = { c, c, c };
    rays[13].Direction() = { -c, c, c };

    RayGroup<Ray<3, float>, 14> rayGroup(rays.begin(), rays.end());

    auto intersections = IntersectBound(box, rayGroup);

    // These hit
    for (unsigned int i = 0; i < 6; ++i) {
        auto tNear = intersections[0](i, 0);
        auto tFar = intersections[1](i, 0);
        EXPECT_LT(tNear, 0.f);
        EXPECT_GT(tFar, 0.f);
    }

    // These miss
    for (unsigned int i = 6; i < 10; ++i) {
        auto tNear = intersections[0](i, 0);
        auto tFar = intersections[1](i, 0);
        EXPECT_LT(tNear, 0.f);
        EXPECT_LT(tFar, 0.f);
    }

    // These hit
    for (unsigned int i = 10; i < 14; ++i) {
        auto tNear = intersections[0](i, 0);
        auto tFar = intersections[1](i, 0);
        EXPECT_GT(tNear, 0.f);
        EXPECT_GT(tFar, 0.f);
    }
}

TEST(AxisAlignedBoxTest, ParallelMiss) {
    AxisAlignedBox<3, float> box({ -0.1f, -0.1f, -0.1f }, { 0.1f, 0.1f, 0.1f });
    std::array<Ray<3, float>, 6> rays;

    rays[0].Origin() = { 1, 0, 0 };
    rays[1].Origin() = {-1, 0, 0 };
    rays[2].Origin() = { 0, 1, 0 };
    rays[3].Origin() = { 0,-1, 0 };
    rays[4].Origin() = { 0, 0, 1 };
    rays[5].Origin() = { 0, 0,-1 };

    rays[0].Direction() = { 0, 1, 1 };
    rays[1].Direction() = { 0, 1, 1 };
    rays[2].Direction() = { 1, 0, 1 };
    rays[3].Direction() = { 1, 0, 1 };
    rays[4].Direction() = { 1, 1, 0 };
    rays[5].Direction() = { 1, 1, 0 };

    RayGroup<Ray<3, float>, 6> rayGroup(rays.begin(), rays.end());

    auto intersections = IntersectBound(box, rayGroup);

    // All miss
    for (unsigned int i = 0; i < 6; ++i) {
        auto tNear = intersections[0](i, 0);
        auto tFar = intersections[1](i, 0);
        EXPECT_GT(tNear, tFar);
    }
}

TEST(AxisAlignedBoxTest, RandomInsideHit) {
    AxisAlignedBox<3, float> box({ -1.f, -1.f, -1.f }, { 1.f, 1.f, 1.f });

    auto randFloat11 = []() {
        return 2.f * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 0.5f);
    };

    std::vector<Ray<3, float>> rays(1000);

    for (auto& ray : rays) {
        ray.Origin() = { 0, 0, 0 };
        ray.Direction() = { randFloat11(), randFloat11(), randFloat11() };
        ray.Direction().normalize();
    }

    std::vector<RayGroup<Ray<3, float>, 4>> rayGroups;
    for (unsigned int i = 0; i < static_cast<unsigned int>(rays.size() - 4); i += 4) {
        rayGroups.emplace_back(rays.begin() + i, rays.begin() + i + 4);
    }

    for (auto& rayGroup : rayGroups) {
        auto intersections = IntersectBound(box, rayGroup);
        for (unsigned int i = 0; i < 4; ++i) {
            auto tNear = intersections[0](i, 0);
            auto tFar = intersections[1](i, 0);
            EXPECT_LT(tNear, tFar);
            EXPECT_LT(tNear, 0.f);
            EXPECT_GT(tFar, 0.f);
        }
    }
}

TEST(AxisAlignedBoxTest, RandomOutsideHit) {
    AxisAlignedBox<3, float> box({ -0.1f, -0.1f, -0.1f }, { 0.1f, 0.1f, 0.1f });

    auto randFloat11 = []() {
        return 2.f * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 0.5f);
    };

    std::vector<Ray<3, float>> rays(1000);

    for (auto& ray : rays) {
        ray.Origin() = { randFloat11(), randFloat11(), randFloat11() };
        ray.Origin().normalize();
        ray.Direction() = -ray.Origin();
    }

    std::vector<RayGroup<Ray<3, float>, 4>> rayGroups;
    for (unsigned int i = 0; i < static_cast<unsigned int>(rays.size() - 4); i += 4) {
        rayGroups.emplace_back(rays.begin() + i, rays.begin() + i + 4);
    }

    for (auto& rayGroup : rayGroups) {
        auto intersections = IntersectBound(box, rayGroup);
        for (unsigned int i = 0; i < 4; ++i) {
            auto tNear = intersections[0](i, 0);
            auto tFar = intersections[1](i, 0);
            EXPECT_LT(tNear, tFar);
            EXPECT_GT(tNear, 0.f);
            EXPECT_GT(tFar, 0.f);
        }
    }
}
