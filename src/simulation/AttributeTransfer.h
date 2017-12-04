
#pragma once
#include <functional>
#include <Eigen/Dense>
#include "flint/core/MultiGrid.h"
#include "flint/core/VectorUtils.h"

namespace simulation {

    template <unsigned int Dimension, typename T, typename Attribute, Attribute Key>
    struct AttributeTransfer {

        /* PARTICLE TRANSFER CALCULATIONS */
        using GridIndex = Eigen::Array<unsigned int, Dimension, 1>;

        class WeightVals {

        public:
            Eigen::Array<T, Dimension, 3> N;
            Eigen::Array<T, Dimension, 3> N_deriv;

            WeightVals() :
                N(Eigen::Array<T, Dimension, 3>()),
                N_deriv(Eigen::Array<T, Dimension, 3>()) {}

            static Eigen::Array<T, Dimension, 1> particleLoc(Eigen::Array<T, Dimension, 1> xp, const Eigen::Array<T, Dimension, 1> &origin) {
                return xp - origin;
            }

            static Eigen::Array<unsigned int, Dimension, 1> baseNode(Eigen::Array < T, Dimension, 1> xp, const Eigen::Array<T, Dimension, 1> &origin, T cellSize) {
                // return (((xp - origin) / cellSize).floor() - 1).cast<unsigned int>();
                return ((xp - origin - cellSize * 0.5) / cellSize).floor().template cast<unsigned int>();
            }

            static Eigen::Array<T, Dimension, 1> baseNodeLoc(Eigen::Array < T, Dimension, 1> xp, const Eigen::Array<T, Dimension, 1> &origin, T cellSize) {
                //return (((xp - origin) / cellSize).floor() - 1) * cellSize;
                return ((xp - origin - cellSize * 0.5) / cellSize).floor() * cellSize;
            }

            // calc N and N' values for curr particle positions
            void fillWeights(Eigen::Array<T, Dimension, 1> xp, T cellSize, const Eigen::Array<T, Dimension, 1> &origin) {
                auto pLoc = particleLoc(xp, origin);
                auto baseNode = baseNodeLoc(xp, origin, cellSize);
                Eigen::Array<T, Dimension, 1> baseNodeOffset = (pLoc - baseNode) / cellSize;

                // for each dimension
                for (uint32_t i = 0; i < Dimension; ++i) {

                    // for each step of the kernel
                    for (uint32_t j = 0; j < 3; ++j) {
                        // distance (in cells) to the jth node in the ith dimension
                        T x = baseNodeOffset[i] - j;
                        T absX = std::abs(x);

                        if (absX < 0.5f) {
                            N(i, j) = 0.75f - pow(absX, 2.f);
                            N_deriv(i, j) = -2.0f * x;
                        } else if (absX < 1.5f) {
                            N(i, j) = 0.5f * pow((1.5f - absX), 2.f);
                            N_deriv(i, j) = x - 1.5f * (x < 0 ? -1 : 1);
                        } else {
                            N(i, j) = 0.f;
                            N_deriv(i, j) = 0.f;
                        }
                    }
                }

            }

            T getWeight(const Eigen::Array<unsigned int, Dimension, 1> &index) const {
                T val = static_cast<T>(1);
                for (unsigned int i = 0; i < Dimension; ++i) {
                    val *= N(i, index[i]);
                }
                return val;
            }

            Eigen::Matrix<T, Dimension, 1> getWeightGradient(const Eigen::Array<unsigned int, Dimension, 1> &index, T cellSize) const {
                Eigen::Array<T, Dimension, 1> grad = Eigen::Array<T, Dimension, 1>::Ones();
                for (unsigned int d = 0; d < Dimension; ++d) {
                    for (unsigned int i_d = 0; i_d < Dimension; ++i_d) {
                        grad[d] *= (i_d == d) ? N_deriv(d, index[d]) : N(d, index[d]);
                    }
                }
                grad /= cellSize;
                return grad.matrix();
            }

        };

        /* PARTICLE MOVEMENT */

        // P2G

        template <unsigned int... I>
        static decltype(auto) KernelGridImpl(std::index_sequence<I...>) {
            return core::StaticMultiGrid<unsigned int, (I, Dimension)...>();
        }

        static const void ApplyOverKernel(const std::function<void(const GridIndex&)> &func) {
            //static constexpr auto kernel = KernelGridImpl(std::make_index_sequence<Dimension>());
            static const auto kernel = core::StaticMultiGrid<unsigned int, Dimension, Dimension, Dimension>();
            kernel.ApplyOverIndices(func);
        }

        template <typename ParticleSet, typename AttributeGrid>
        static void IterateParticleKernel(
                const ParticleSet &particleSet,
                AttributeGrid &attributeGrid,
                const Eigen::Array<T, Dimension, 1> &origin,
                const std::function<void(unsigned int, const GridIndex&, const GridIndex&)> &func) {

            const auto& keyAttributes = particleSet.template GetAttributeList<Key>();
            core::VectorUtils::ApplyOverIndices(keyAttributes, [&](unsigned int p) {
                auto& key = keyAttributes[p];
                Eigen::Array<unsigned int, Dimension, 1> baseNode = WeightVals::baseNode(key, origin, attributeGrid.CellSize());
                ApplyOverKernel([&](const auto& offset) {
                    func(p, offset, baseNode + offset);
                });
            });
        }
    };


}
