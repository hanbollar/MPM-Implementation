#pragma once
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include "ParticleSet.h"
#include "AttributeGrid.h"
#include "AttributeTransfer.h"


namespace simulation {
namespace MPM {
    // Declare attributes we will use in the simulation
    enum class SimulationAttribute {
        Volume,
        Position,
        Velocity,
        Mass,
        Weights,
        Force,
        DeformationUpdate,
        Deformation,
        Stress,
        Momentum,
        AffineState, // The B matrix for APIC Simulation
        mu,
        lambda,
	mu_0, // for snow
	lambda_0, // for snow
	Jp_snow // for snow
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
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::AffineState> {
            using type = Eigen::Matrix<T, Dimension, Dimension>;
            static type Default() { return type::Zero(); }
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

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::mu_0> {
            using type = T;
            static type Default() { return{}; }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::lambda_0> {
            using type = T;
            static type Default() { return{}; }
        };

        template <>
        struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Jp_snow> {
            using type = T;
            static type Default() { return{}; }
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
            SimulationAttribute::Deformation,
            SimulationAttribute::Stress,
            SimulationAttribute::AffineState,
            SimulationAttribute::mu,
            SimulationAttribute::lambda,
            SimulationAttribute::mu_0,
            SimulationAttribute::lambda_0,
            SimulationAttribute::Jp_snow
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

        template <unsigned int I>
        struct KernelGridImpl {
            template <unsigned int D, unsigned int... Ds>
            static constexpr decltype(auto) Get(std::integer_sequence<unsigned int, D, Ds...>) {
                return KernelGridImpl<I - 1>::Get(std::integer_sequence<unsigned int, D, D, Ds...>{});
            }
        };

        template<>
        struct KernelGridImpl<0> {
            template <unsigned int D, unsigned int... Ds>
            static constexpr decltype(auto) Get(std::integer_sequence<unsigned int, D, Ds...>) {
                return core::StaticMultiGrid<unsigned int, Ds...>();
            }
        };

        template <typename Function>
        static const void ApplyOverKernel(const Function &func) {
            static const auto kernel = KernelGridImpl<Dimension>::Get(std::integer_sequence<unsigned int, Dimension>{});
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
            auto& particleWeights = particles.GetAttributeList<SimulationAttribute::Weights>();
            const auto& particlePositions = particles.GetAttributeList<SimulationAttribute::Position>();
            const auto& particleVelocities = particles.GetAttributeList<SimulationAttribute::Velocity>();

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

            //std::cout << "dt: " << dt << std::endl;

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
            auto& particleVelocities = particles.Get<SimulationAttribute::Velocity>();
            const auto& particleWeights = particles.Get<SimulationAttribute::Weights>();
            const auto& particleAffineStates = particles.Get<SimulationAttribute::AffineState>();
            const auto& particlePositions = particles.Get<SimulationAttribute::Position>();

            gridMasses.Fill(0.f);
            gridVelocities.Fill(AttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity>::type::Zero());
            gridMomentums.Fill(AttributeDefinitions::AttributeInfo<SimulationAttribute::Momentum>::type::Zero());
            gridForces.Fill(AttributeDefinitions::AttributeInfo<SimulationAttribute::Force>::type::Zero());
       
            const T D = (grid.CellSize() * grid.CellSize() * 0.25);

            IterateParticleKernel([&](unsigned int p, const auto& offset, const auto& i) {
                auto* gridMass = gridMasses.at(i);
                auto* gridMomentum = gridMomentums.at(i);

                Eigen::Array<T, Dimension, 1> gridPosition = i.template cast<T>() * this->grid.CellSize() + gridOrigin;

                if (gridMass && gridMomentum) {
                    auto weight = particleWeights[p].getWeight(offset);

                   const auto& particleAffineState = particleAffineStates[p];

                   const auto xixp = gridPosition.matrix() - particlePositions[p];

                   // Update grid weights
                   *gridMass     += weight * particleMasses[p];
                   *gridMomentum += weight * particleMasses[p] * (particleVelocities[p] + particleAffineState * (1/D) * xixp);
                }
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
                    auto* gridForce = gridForces.at(baseNode + offset);
                    if (gridForce) {
                        *gridForce -= m * particleWeights[p].getWeightGradient(offset, grid.CellSize());
                    }
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

                // don't use with normal elastic material, only snow
                /*Eigen::Matrix<T, Dimension, 1> gridCellPosWorld = i.matrix().cast<T>() * grid.CellSize() + gridOrigin.matrix().cast<T>();
                if (std::abs(gridCellPosWorld[0]) < 0.1 && gridCellPosWorld[1] < -0.6) {
                    gridVelocities[i][1] = 0.0;
                }*/
            });
        }

        void G2P() {
            auto& particleVelocities = particles.Get<SimulationAttribute::Velocity>();
            const auto& particleWeights = particles.Get<SimulationAttribute::Weights>();
            const auto& gridVelocities = grid.Get<SimulationAttribute::Velocity>();
            // The B matrix
            auto& particleAffineStates = particles.Get<SimulationAttribute::AffineState>();
            auto& particlePositions = particles.Get<SimulationAttribute::Position>();

            core::VectorUtils::ApplyOverElements(particleVelocities, [](auto &particleVelocity) {
                particleVelocity = AttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity>::type::Zero();
            });

            core::VectorUtils::ApplyOverElements(particleAffineStates, [](auto &particleAffineState) {
              particleAffineState = AttributeDefinitions::AttributeInfo<SimulationAttribute::AffineState>::type::Zero();
            });

            IterateParticleKernel([&](unsigned int p, const auto& offset, const auto& i) {
                auto* gridVelocity = gridVelocities.at(i);

                const Eigen::Array<T, Dimension, 1> gridPosition = i.template cast<T>() * this->grid.CellSize() + gridOrigin;
                const auto particlePosition = particlePositions[p];

                if (gridVelocity) {
                  particleVelocities[p] += gridVelocities[i] * particleWeights[p].getWeight(offset);

                  // Update the B
                  const auto xixp = gridPosition.matrix() - particlePosition;
                  particleAffineStates[p] += particleWeights[p].getWeight(offset) * (*gridVelocity) *  xixp.transpose();
                }
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

            core::VectorUtils::ApplyOverElements(deformationUpdates, [](auto& deformationUpdate) {
                deformationUpdate = AttributeDefinitions::AttributeInfo<SimulationAttribute::DeformationUpdate>::type::Zero();
            });

            IterateParticleKernel([&](unsigned int p, const auto& offset, const auto& i) {
                auto* gridVelocity = gridVelocities.at(i);
                if (gridVelocity) {
                    deformationUpdates[p] += gridVelocities[i] * particleWeights[p].getWeightGradient(offset, grid.CellSize()).transpose();
                }
            });

	    // Update deformation gradient
        core::VectorUtils::ApplyOverIndices(particleDeformations, [&](unsigned int p) {
            particleDeformations[p] += dt * deformationUpdates[p] * particleDeformations[p];
        });

        //#define SNOW
        #ifdef SNOW
        
	    // Snow
        // Apply plasticity for snow
        const T theta_s = 2e-3;
        const T theta_c = 7.5e-3;
        const T min_Jp = 0.1;
        const T ksi = 10; // hardening parameter

        auto& particleMus = particles.Get<SimulationAttribute::mu>();
        auto& particleLambdas = particles.Get<SimulationAttribute::lambda>();
        auto& particleMu_0s = particles.Get<SimulationAttribute::mu_0>();
        auto& particleLambda_0s = particles.Get<SimulationAttribute::lambda_0>();
        auto& particleJpSnows = particles.Get<SimulationAttribute::Jp_snow>();

        core::VectorUtils::ApplyOverIndices(particleDeformations, [&](unsigned int p) {
            auto& F = particleDeformations[p];
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
            		
		    // clamp the sigma
            Eigen::Matrix<T, Dimension, Dimension> SigmaMatrix = Eigen::Matrix<T, Dimension, Dimension>::Zero();
		    T Je_original = 1; // original determinant of elastic deformation gradient
		    T Je_new = 1; // new determinant of elastic deformation gradient
		    for(int d = 0; d < Dimension; ++d) {
		        Je_original *= Sigma(d);
		        SigmaMatrix(d, d) = std::min(std::max(Sigma(d), 1 - theta_c), 1 + theta_s); 
		        Je_new *= SigmaMatrix(d, d);
		    }

		    F = U * SigmaMatrix * V.transpose();

		    // Update Jp
		    particleJpSnows[p] = std::max(particleJpSnows[p] * Je_original / Je_new, min_Jp);

		    // Do hardening
		    T power = ksi * (1 - particleJpSnows[p]);
		    particleMus[p]= particleMu_0s[p] * std::exp(power);
		    particleLambdas[p]= particleLambda_0s[p] * std::exp(power);
        });
        #endif

        }
    };
}
}