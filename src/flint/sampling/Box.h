
#pragma once

#include <chrono>
#include <random>
#include "core/AxisAlignedBox.h"
#include "core/Math.h"
#include "core/MultiGrid.h"
#include "Random.h"

namespace sampling {

// https://www.cct.lsu.edu/~fharhad/ganbatte/siggraph2007/CD2/content/sketches/0250.pdf
template <typename SamplePrecision, int N, typename T>
std::vector<Eigen::Array<SamplePrecision, N, 1>> SampleBox(const core::AxisAlignedBox<N, T> &box, SamplePrecision minDistance, RandomGenerator rng = RandomGenerator()) {
    SamplePrecision cellSize = static_cast<SamplePrecision>(minDistance / std::sqrt(N));

    using sample_t = Eigen::Array<SamplePrecision, N, 1>;
    using sample_cell_t = Eigen::Array<unsigned int, N, 1>;

    sample_cell_t gridSize;
    for (int i = 0; i < N; ++i) {
        gridSize(i, 0) = static_cast<unsigned int>(std::ceil(box.Extent(i) / cellSize));
    }

    auto cellToGridIndex = [](const sample_cell_t &cell) {
        core::MultiGrid<float, N>::Index index;
        for (unsigned int i = 0; i < N; ++i) {
            index[i] = cell(i, 0);
        }
        return index;
    };

    std::vector<sample_t> samples;
    std::vector<unsigned int> activeList;

    core::MultiGrid<float, N>::Index backgroundGridSize = cellToGridIndex(gridSize);
    core::MultiGrid<float, N> backgroundGrid(backgroundGridSize);
    backgroundGrid.Fill(-1);



    std::uniform_real_distribution<float> unif01(0, 1);
    std::uniform_real_distribution<SamplePrecision> unif11(-1, 1);
    std::uniform_real_distribution<SamplePrecision> unifR(std::pow(minDistance, N), std::pow(2 * minDistance, N));

    sample_t x0;
    for (unsigned int n = 0; n < N; ++n) {
        x0(n, 0) = unif01(rng);
    }
    x0 = x0 * (box.max() - box.min()) + box.min();

    samples.push_back(x0);
    backgroundGrid[cellToGridIndex(((x0 - box.min()) / cellSize).template cast<unsigned int>())] = 0;
    activeList.push_back(0);

    static constexpr unsigned int k = 32;
    std::array<sample_t, k> testSamples;

    while(unsigned int n = static_cast<unsigned int>(activeList.size())) {
        unsigned int i = static_cast<unsigned int>(static_cast<float>(n) * unif01(rng));
        sample_t& xi = samples[activeList[i]];

        for (unsigned int s = 0; s < k; ++s) {
            sample_t& sample = testSamples[s];
            for (unsigned int n = 0; n < N; ++n) {
                sample(n, 0) = unif11(rng);
            }

            sample /= sample.matrix().norm();
            sample *= static_cast<SamplePrecision>(std::pow(unifR(rng), 1.0 / N));
        }

        for (unsigned int s = 0; s < k; ++s) {
            testSamples[s] += xi;
        }

        static constexpr unsigned int cellCount = static_cast<unsigned int>(core::constPow(3, N));
        std::array<sample_cell_t, cellCount> offsets;
        for (unsigned int c = 0; c < cellCount; ++c) {
            for (unsigned int n = 0; n < N; ++n) {
                offsets[c](n, 0) = static_cast<unsigned int>(c / (cellCount / 3)) % 3;
            }
        }

        auto cellInBounds = [&gridSize](const sample_cell_t& cell) -> bool {
            return (cell >= 0).all() && (cell < gridSize).all();
        };

        auto sampleInBounds = [&box](const sample_t& sample) -> bool {
            return (sample >= box.min()).all() && (sample < box.max()).all();
        };

        std::array<int, cellCount> indices;
        bool keep = false;
        for (unsigned int s = 0; s < k; ++s) {
            bool found = true;

            sample_cell_t origin = ((testSamples[s] - box.min()) / cellSize).template cast<unsigned int>();
            if (!sampleInBounds(testSamples[s])) {
                continue;
            }

            for (unsigned int c = 0; c < cellCount; ++c) {
                auto cell = origin + offsets[c];
                indices[c] = cellInBounds(cell) ? backgroundGrid[cellToGridIndex(cell)] : -1;

                int i = indices[c];
                if (i < 0) continue;
                auto distance = (testSamples[s] - samples[indices[c]]).matrix().squaredNorm();
                if (distance < std::pow(minDistance, 2)) {
                    found = false;
                    break;
                }
            }

            if (found) {
                keep = true;
                activeList.push_back(static_cast<unsigned int>(samples.size()));
                backgroundGrid[cellToGridIndex(origin)] = static_cast<int>(samples.size());
                samples.push_back(testSamples[s]);
                break;
            }
        }

        if (!keep) {
            activeList.erase(activeList.begin() + i);
        }
    }

    return samples;
}

}
