#pragma once
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include "flint/core/VectorUtils.h"
#include "flint/utility/Sequence.h"
#include "ParticleSet.h"
#include "AttributeGrid.h"

namespace simulation {
namespace MPM {
    // Declare attributes we will use in the simulation
    enum class SimulationAttribute {
        Volume,
        Position,
        Velocity,
        Mass,
        Weights,
        KernelMask,
        Force,
        DeformationUpdate,
        Deformation,
        Stress,
        Momentum,
        mu,
        lambda
    };

    template <typename T, unsigned int Dimension>
    class KernelWeights {

    public:
        Eigen::Array<T, Dimension, 3> N;
        Eigen::Array<T, Dimension, 3> N_deriv;

        KernelWeights() :
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
                    }
                    else if (absX < 1.5f) {
                        N(i, j) = 0.5f * pow((1.5f - absX), 2.f);
                        N_deriv(i, j) = x - 1.5f * (x < 0 ? -1 : 1);
                    }
                    else {
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

    template <typename T, unsigned int Dimension>
    struct AttributeDefinitions {
        template <SimulationAttribute A>
        struct AttributeInfo {};

        template<>
        struct AttributeInfo<SimulationAttribute::Volume> {
            using type = T;
            static type Default() { return T(0); }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Position> {
            using type = Eigen::Matrix<T, Dimension, 1>;
            static type Default() { return {}; }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity> {
            using type = Eigen::Matrix<T, Dimension, 1>;
            static type Default() { return type::Zero(); }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Mass> {
            using type = T;
            static type Default() { return T(0); }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Force> {
            using type = Eigen::Matrix<T, Dimension, 1>;
            static type Default() { return type::Zeros(); }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Stress> {
            using type = Eigen::Matrix<T, Dimension, Dimension>;
            static type Default() { return type::Identity(); }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Deformation> {
            using type = Eigen::Matrix<T, Dimension, Dimension>;
            static type Default() { return type::Identity(); }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::DeformationUpdate> {
            using type = Eigen::Matrix<T, Dimension, Dimension>;
            static type Default() { return type::Zero(); }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Weights> {
            using type = KernelWeights<T, Dimension>;
            static type Default() { return {}; }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::KernelMask> {
            using type = core::StaticMultiGridCube<T, Dimension, 3>;
            static type Default() { return {}; }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Momentum> {
            using type = Eigen::Matrix<T, Dimension, 1>;
            static type Default() { return type::Zeros(); }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::mu> {
            using type = T;
            static type Default() { return {}; }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::lambda> {
            using type = T;
            static type Default() { return {}; }
        };
    };

    template <typename T, unsigned int Dimension>
    class MPMSimulation {
        using AttributeDefinitions = simulation::MPM::AttributeDefinitions<T, Dimension>;

    public:
        // Declare the set of particle attributes
        using ParticleSet = simulation::ParticleSet<SimulationAttribute, AttributeDefinitions,
            SimulationAttribute::Volume,
            SimulationAttribute::DeformationUpdate,
            SimulationAttribute::Position,
            SimulationAttribute::Velocity,
            SimulationAttribute::Mass,
            SimulationAttribute::Weights,
            SimulationAttribute::KernelMask,
            SimulationAttribute::Deformation,
            SimulationAttribute::Stress,
            SimulationAttribute::mu,
            SimulationAttribute::lambda
        >;

        // Declare the grid attributes
        using AttributeGrid = simulation::AttributeGrid<Dimension, T, SimulationAttribute, AttributeDefinitions,
            SimulationAttribute::Mass,
            SimulationAttribute::Momentum,
            SimulationAttribute::Velocity,
            SimulationAttribute::Force
        >;

    private:
        ParticleSet particles;
        AttributeGrid grid;

        Eigen::Array<T, Dimension, 1> gridOrigin;
        Eigen::Array<T, Dimension, 1> gridSize;

        T cellSize;
        T CFL = T(0.5);
        T gravity = T(9.80665);
        T maxdt = T(1e-3);

        template <typename Function>
        static const void ApplyOverKernel(const Function &func) {
            core::StaticMultiGridCube<unsigned int, Dimension, 3> kernel;
            kernel.ApplyOverIndices(func);
        }

        template <typename Function>
        void IterateParticleKernel(const Function &func) {
            const auto& particlePositions = particles.GetAttributeList<SimulationAttribute::Position>();
            core::VectorUtils::ApplyOverIndices(particlePositions, [&](unsigned int p) {
                Eigen::Array<unsigned int, Dimension, 1> baseNode = KernelWeights<T, Dimension>::baseNode(particlePositions[p], gridOrigin, grid.CellSize());
                ApplyOverKernel([&](const auto& offset) {
                    func(p, offset, baseNode + offset);
                });
            });
        }

    public:

        template <SimulationAttribute A>
        decltype(auto) GetParticleAttribute() {
            return particles.Get<A>();
        }

        template <SimulationAttribute A>
        decltype(auto) GetParticleAttribute() const {
            return particles.Get<A>();
        }

        template <SimulationAttribute A>
        decltype(auto) GetGridAttribute() {
            return grid.Get<A>();
        }

        template <SimulationAttribute A>
        decltype(auto) GetGridAttribute() const {
            return grid.Get<A>();
        }

        void SetCellSize(T _cellSize) {
            cellSize = _cellSize;
        }

        void SetBounds(const Eigen::Array<T, Dimension, 1>& min, const Eigen::Array<T, Dimension, 1>& max) {
            gridOrigin = min;
            gridSize = max - min;
        }

        void SetMaxdt(T dt) {
            maxdt = dt;
        }

        void AddParticles(const ParticleSet& _particles) {
            particles.Append(_particles);
        }

        void Init() {
            grid.Resize(cellSize, gridSize);

            auto& gridMasses = grid.GetGrid<SimulationAttribute::Mass>();
            // Just get an initial count of number of particles in each cell
            IterateParticleKernel([&](unsigned int p, const auto& offset, const auto& i) {
                auto* gridMass = gridMasses.at(i);
                if (gridMass) {
                    *gridMass += 1.0;
                }
            });

            T count = T(0);
            T cells = T(0);
            // Reset masses back to zero
            gridMasses.ApplyOverCells([&](auto& gridMass) {
                count += gridMass;
                cells++;
                gridMass = T(0);
            });

            std::cout << count / cells << " particles per cell" << std::endl;
        }

        T Step() {
            auto& particleWeights = particles.Get<SimulationAttribute::Weights>();
            const auto& particlePositions = particles.Get<SimulationAttribute::Position>();
            const auto& particleVelocities = particles.Get<SimulationAttribute::Velocity>();
            const auto& gridMasses = grid.Get<SimulationAttribute::Mass>();
            auto& kernelMasks = particles.Get<SimulationAttribute::KernelMask>();

            IterateParticleKernel([&](unsigned int p, const auto& offset, const auto& i) {
                if (gridMasses.at(i)) {
                    kernelMasks[p][offset] = T(1);
                } else {
                    kernelMasks[p][offset] = T(0);
                }
            });

            core::VectorUtils::ApplyOverIndices(particleWeights, [&](unsigned int p) {
                particleWeights[p].fillWeights(particlePositions[p], grid.CellSize(), gridOrigin);
            });

            T maxParticleVelocity = 0.f;
            core::VectorUtils::ApplyOverElements(particleVelocities, [&](const auto& particleVelocity) {
                maxParticleVelocity = std::max(maxParticleVelocity, particleVelocity.norm());
            });
            T dt = std::min(maxdt, CFL * grid.CellSize() / maxParticleVelocity);

            while (true) {
                P2G();
                ComputeForces();
                ApplyForces(dt);
                HandleCollisions();
                G2P();

                maxParticleVelocity = 0.f;
                core::VectorUtils::ApplyOverElements(particleVelocities, [&](const auto& particleVelocity) {
                    maxParticleVelocity = std::max(maxParticleVelocity, particleVelocity.norm());
                });

                if (maxParticleVelocity * dt > grid.CellSize() * CFL) {
                    T nextStep = std::min(maxdt, CFL * grid.CellSize() / maxParticleVelocity);
                    if (nextStep / dt > 0.5) {
                        dt *= 0.5;
                    } else {
                        dt = nextStep;
                    }
                } else {
                    break;
                }
            }

            std::cout << "dt: " << dt << std::endl;

            UpdatePositions(dt);
            UpdateDeformation(dt);

            return dt;
        }

    private:
        void P2G() {
            auto& gridMasses = grid.Get<SimulationAttribute::Mass>();
            auto& gridVelocities = grid.Get<SimulationAttribute::Velocity>();
            auto& gridMomentums = grid.Get<SimulationAttribute::Momentum>();
            auto& gridForces = grid.Get<SimulationAttribute::Force>();
            const auto& particleMasses = particles.Get<SimulationAttribute::Mass>();
            const auto& particleVelocities = particles.Get<SimulationAttribute::Velocity>();
            const auto& particleWeights = particles.Get<SimulationAttribute::Weights>();

            gridMasses.Fill(0.f);
            gridVelocities.Fill(AttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity>::type::Zero());
            gridMomentums.Fill(AttributeDefinitions::AttributeInfo<SimulationAttribute::Momentum>::type::Zero());
            gridForces.Fill(AttributeDefinitions::AttributeInfo<SimulationAttribute::Force>::type::Zero());

            IterateParticleKernel([&](unsigned int p, const auto& offset, const auto& i) {
                auto weight = particleWeights[p].getWeight(offset);
                gridMasses[i] += weight * particleMasses[p];
                gridMomentums[i] += weight * particleMasses[p] * particleVelocities[p];
            });

            gridVelocities.ApplyOverIndices([&](const auto& i) {
                if (gridMasses[i] > 0) {
                    gridVelocities[i] = gridMomentums[i] / gridMasses[i];
                }
            });
        }

        template <int i, int j, typename Matrix>
        static auto minorDet(const Matrix &m) {
            return m(i == 0 ? 1 : 0, j == 0 ? 1 : 0) * m(i == Dimension - 1 ? i - 1 : Dimension - 1, j == Dimension - 1 ? j - 1 : Dimension - 1)
                 - m(i == 0 ? 1 : 0, j == Dimension - 1 ? j - 1 : Dimension - 1) * m(i == Dimension - 1 ? i - 1 : Dimension - 1, j == 0 ? 1 : 0);
        };

        static Eigen::Matrix<T, 2, 2> ComputeJFInvTranspose(const Eigen::Matrix<T, 2, 2> &F) {
            Eigen::Matrix<T, 2, 2> result = Eigen::Matrix<T, 2, 2>::Zero();
            result(0, 0) = F(1, 1);
            result(1, 0) = -F(0, 1);
            result(0, 1) = -F(1, 0);
            result(1, 1) = F(0, 0);
            return result;
        }

        static Eigen::Matrix<T, 3, 3> ComputeJFInvTranspose(const Eigen::Matrix<T, 3, 3> &F) {
            Eigen::Matrix<T, 3, 3> result;
            result << minorDet<0, 0>(F), -minorDet<0, 1>(F),  minorDet<0, 2>(F),
                     -minorDet<1, 0>(F),  minorDet<1, 1>(F), -minorDet<1, 2>(F),
                      minorDet<2, 0>(F), -minorDet<2, 1>(F),  minorDet<2, 2>(F);
            return result;
        }

        void ComputeForces() {
            auto& particleStresses = particles.Get<SimulationAttribute::Stress>();
            const auto& particleDeformations = particles.Get<SimulationAttribute::Deformation>();
            const auto& particleMus = particles.Get<SimulationAttribute::mu>();
            const auto& particleLambdas = particles.Get<SimulationAttribute::lambda>();

            core::VectorUtils::ApplyOverIndices(particleStresses, [&](unsigned int p) {
                auto& stress = particleStresses[p];
                const auto& F = particleDeformations[p];
                auto J = F.determinant();
                Eigen::JacobiSVD<Eigen::Matrix<T, Dimension, Dimension>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Matrix<T, Dimension, Dimension> U = svd.matrixU();
                Eigen::Matrix<T, Dimension, 1> Sigma = svd.singularValues();
                Eigen::Matrix<T, Dimension, Dimension> V = svd.matrixV();

                Eigen::Matrix<T, Dimension, 1> temp;
                for (unsigned int i = 1; i < Sigma.size(); ++i) {
                    for (unsigned int j = i; j > 0 && Sigma(j - 1, 0) < Sigma(j, 0); j--) {
                        std::swap(Sigma(j, 0), Sigma(j - 1, 0));

                        temp = U.row(j);
                        U.row(j) = U.row(j - 1);
                        U.row(j - 1) = temp;

                        temp = V.row(j);
                        V.row(j) = V.row(j - 1);
                        V.row(j - 1) = temp;
                    }
                }

                if (U.determinant() < 0) {
                    U.col(Dimension - 1) *= -1;
                    Sigma(Dimension - 1, 0) *= -1;
                }
                if (V.determinant() < 0) {
                    V.col(Dimension - 1) *= -1;
                    Sigma(Dimension - 1, 0) *= -1;
                }

                assert(U.determinant() > 0);
                assert(V.determinant() > 0);
                auto _F = (U * Eigen::DiagonalMatrix<T, Dimension>(Sigma) * V.transpose()).eval();
                assert(((_F.array() - F.array()) < 1e-4).all());

                Eigen::Matrix<T, Dimension, Dimension> R = U * V.transpose();
                Eigen::Matrix<T, Dimension, Dimension> jFInvTranspose = ComputeJFInvTranspose(F);
                stress = 2 * particleMus[p] * (F - R) + particleLambdas[p] * (J - 1) * jFInvTranspose; // corotated
            });

            const auto& particlePositions = particles.Get<SimulationAttribute::Position>();
            const auto& particleVolumes = particles.Get<SimulationAttribute::Volume>();
            const auto& particleWeights = particles.Get<SimulationAttribute::Weights>();
            auto& gridForces = grid.Get<SimulationAttribute::Force>();
            core::VectorUtils::ApplyOverIndices(particlePositions, [&](unsigned int p) {
                Eigen::Array<unsigned int, Dimension, 1> baseNode = KernelWeights<T, Dimension>::baseNode(particlePositions[p], gridOrigin, grid.CellSize());

                Eigen::Matrix<T, Dimension, Dimension> m = particleVolumes[p] * particleStresses[p] * particleDeformations[p].transpose();
                ApplyOverKernel([&](const auto& offset) {
                    gridForces[baseNode + offset] -= m * particleWeights[p].getWeightGradient(offset, grid.CellSize());
                });
            });
        }

        void ApplyForces(T dt) {
            auto& gridVelocities = grid.Get<SimulationAttribute::Velocity>();
            const auto& gridForces = grid.Get<SimulationAttribute::Force>();
            const auto& gridMasses = grid.Get<SimulationAttribute::Mass>();

            gridVelocities.ApplyOverIndices([&](const auto& i) {
                if (gridMasses[i] > 0) {
                    gridVelocities[i] += (dt * gridForces[i]) / gridMasses[i];
                }
            });

            gridVelocities.ApplyOverIndices([&](const auto& i) {
                gridVelocities[i][1] -= gravity * dt;
            });
        }

        void HandleCollisions() {
            auto& gridVelocities = grid.Get<SimulationAttribute::Velocity>();
            const auto& gridVelocityDims = gridVelocities.GetSizes();

            // Wastes lots of time in the middle of the grid. This should be optimized
            gridVelocities.ApplyOverIndices([&](const auto &i) {
                for (unsigned int d = 0; d < Dimension; ++d) {
                    if (i[d] == 0 && gridVelocities[i][d] < T(0)) {
                        gridVelocities[i][d] = T(0);
                    } else if (i[d] == gridVelocityDims[d] - 1 && gridVelocities[i][d] > T(0)) {
                        gridVelocities[i][d] = T(0);
                    }
                }
            });
        }

        void G2P() {
            auto& particleVelocities = particles.Get<SimulationAttribute::Velocity>();
            const auto& particleWeights = particles.Get<SimulationAttribute::Weights>();
            const auto& gridVelocities = grid.Get<SimulationAttribute::Velocity>();
            const auto& kernelMasks = particles.Get<SimulationAttribute::KernelMask>();

            core::VectorUtils::ApplyOverElements(particleVelocities, [](auto &particleVelocity) {
                particleVelocity = AttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity>::type::Zero();
            });

            IterateParticleKernel([&](unsigned int p, const auto& offset, const auto& i) {
                particleVelocities[p] += kernelMasks[p][offset] * gridVelocities[i] * particleWeights[p].getWeight(offset);
            });
        }

        void UpdatePositions(T dt) {
            auto& particlePositions = particles.Get<SimulationAttribute::Position>();
            const auto& particleVelocities = particles.Get<SimulationAttribute::Velocity>();

            core::VectorUtils::ApplyOverIndices(particlePositions, [&](unsigned int p) {
                particlePositions[p] += particleVelocities[p] * dt;
            });

            // Clamp positions
            core::VectorUtils::ApplyOverElements(particlePositions, [&](auto &particlePosition) {
                particlePosition = gridOrigin.max(particlePosition.array()).min(gridOrigin + gridSize).matrix();
            });
        }

        void UpdateDeformation(T dt) {
            auto& particleDeformations = particles.Get<SimulationAttribute::Deformation>();
            auto& deformationUpdates = particles.Get<SimulationAttribute::DeformationUpdate>();
            const auto& particleWeights = particles.Get<SimulationAttribute::Weights>();
            const auto& gridVelocities = grid.Get<SimulationAttribute::Velocity>();
            const auto& kernelMasks = particles.Get<SimulationAttribute::KernelMask>();

            core::VectorUtils::ApplyOverElements(deformationUpdates, [](auto& deformationUpdate) {
                deformationUpdate = AttributeDefinitions::AttributeInfo<SimulationAttribute::DeformationUpdate>::type::Zero();
            });
            IterateParticleKernel([&](unsigned int p, const auto& offset, const auto& i) {
                deformationUpdates[p] += kernelMasks[p][offset] * gridVelocities[i] * particleWeights[p].getWeightGradient(offset, grid.CellSize()).transpose();
            });
            core::VectorUtils::ApplyOverIndices(particleDeformations, [&](unsigned int p) {
                particleDeformations[p] += dt * deformationUpdates[p] * particleDeformations[p];
            });
        }
    };
}
}
