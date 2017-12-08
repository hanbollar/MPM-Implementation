
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
                return ((xp - origin - cellSize * 0.5) / cellSize).floor().template cast<unsigned int>();
            }

            static Eigen::Array<T, Dimension, 1> baseNodeLoc(Eigen::Array < T, Dimension, 1> xp, const Eigen::Array<T, Dimension, 1> &origin, T cellSize) {
                return ((xp - origin - cellSize * 0.5) / cellSize).floor() * cellSize;
            }

            // calc N and N' values for curr particle positions
            void fillWeights(Eigen::Array<T, Dimension, 1> xp, T cellSize, const Eigen::Array<T, Dimension, 1> &origin) {
                Eigen::Array<T, Dimension, 1> gridOffset = (xp - origin) / cellSize;
                Eigen::Array<T, Dimension, 1> baseNodeOffset = gridOffset - (gridOffset - 0.5).floor();

                // for each dimension
                for (uint32_t i = 0; i < Dimension; ++i) {
                    // for each step of the kernel
                    for (uint32_t j = 0; j < 3; ++j) {
                        // distance (in cells) to the jth node in the ith dimension
                        T x = baseNodeOffset[i] - static_cast<T>(j);
                        T absX = std::abs(x);
                        assert(absX <= 2);

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
                for (unsigned int r = 0; r < Dimension; ++r) {
                    for (unsigned int c = 0; c < Dimension; ++c) {
                        grad[r] *= (r == c) ? N_deriv(c, index[c]) : N(c, index[c]);
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

        template <typename Function>
        static const void ApplyOverKernel(const Function &func) {
            static const auto kernel = KernelGridImpl(std::make_index_sequence<Dimension>());
            //static const auto kernel = core::StaticMultiGrid<unsigned int, Dimension, Dimension, Dimension>();
            kernel.ApplyOverIndices(func);
        }

        template <typename ParticleSet, typename AttributeGrid, typename Function>
        static void IterateParticleKernel(
                const ParticleSet &particleSet,
                AttributeGrid &attributeGrid,
                const Eigen::Array<T, Dimension, 1> &origin,
                const Function &func) {

            const auto& keyAttributes = particleSet.template GetAttributeList<Key>();
            core::VectorUtils::ApplyOverIndices(keyAttributes, [&](unsigned int p) {
                Eigen::Array<unsigned int, Dimension, 1> baseNode = WeightVals::baseNode(keyAttributes[p], origin, attributeGrid.CellSize());
                ApplyOverKernel([&](const auto& offset) {
                    func(p, offset, baseNode + offset);
                });
            });
        }
    };


}
