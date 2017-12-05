
#include <iostream>
#include <thread>
#include <chrono>
#include <glad/glad.h>
#include "flint/accel/bvh/TreeBuilder.h"
#include "flint/accel/bvh/Tree.h"
#include "flint/core/AxisAlignedBox.h"
#include "flint/core/Camera.h"
#include "flint/core/Math.h"
#include "flint/core/Optional.h"
#include "flint/core/VectorUtils.h"
#include "flint/geometry/Triangle.h"
#include "flint/import/ObjLoader.h"
#include "flint/intersection/Ray.h"
#include "flint/intersection/bvh/Tree.h"
#include "flint/sampling/Box.h"
#include "flint/sampling/Mesh.h"
#include "flint_viewport/viewport.h"
#include "flint_viewport/CameraControls.h"
#include "flint_viewport/gl/ShaderProgram.h"
#include "simulation/ParticleSet.h"
#include "simulation/AttributeGrid.h"
#include "simulation/AttributeTransfer.h"

using namespace core;

constexpr unsigned int kDimension = 3; // 3D simulation
static constexpr auto simulationTimestep = std::chrono::duration<double, std::milli>(16); // timestep for simulation

Camera<float> camera;

// Declare attributes we will use in the simulation
enum class SimulationAttribute {
    InitialVolume,
    Position,
    Velocity,
    Mass,
	Weights,
    Force,
    DeformationUpdate,
    Deformation,
    Stress,
};

// Configure attribute transfer to transfer by position
using AttributeTransfer = simulation::AttributeTransfer<kDimension, float, SimulationAttribute, SimulationAttribute::Position>;

// Define the types of each simulation attribute
struct AttributeDefinitions {
    template <SimulationAttribute A>
    struct AttributeInfo {};
};

template <>
struct AttributeDefinitions::AttributeInfo<SimulationAttribute::InitialVolume> {
    using type = float;
};

template <>
struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Position> {
    using type = Eigen::Matrix<float, kDimension, 1>;
};

template <>
struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity> {
    using type = Eigen::Matrix<float, kDimension, 1>;
};

template <>
struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Mass> {
    using type = float;
};

template <>
struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Force> {
    using type = Eigen::Matrix<float, kDimension, 1>;
};

template <>
struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Stress> {
    using type = Eigen::Matrix<float, kDimension, kDimension>;
};

template <>
struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Deformation> {
    using type = Eigen::Matrix<float, kDimension, kDimension>;
};

template <>
struct AttributeDefinitions::AttributeInfo<SimulationAttribute::DeformationUpdate> {
    using type = Eigen::Matrix<float, kDimension, kDimension>;
};

template <>
struct AttributeDefinitions::AttributeInfo<SimulationAttribute::Weights> {
	using type = AttributeTransfer::WeightVals;
};

// Declare the set of particle attributes
simulation::ParticleSet<SimulationAttribute, AttributeDefinitions,
    SimulationAttribute::InitialVolume,
    SimulationAttribute::DeformationUpdate,
    SimulationAttribute::Position,
    SimulationAttribute::Velocity,
    SimulationAttribute::Mass,
	SimulationAttribute::Weights,
    SimulationAttribute::Deformation,
    SimulationAttribute::Stress
> particles;

// Declare the grid attributes
simulation::AttributeGrid<kDimension, float, SimulationAttribute, AttributeDefinitions,
	SimulationAttribute::Position,
    SimulationAttribute::Velocity,
	SimulationAttribute::Mass,
    SimulationAttribute::Force
> grid;

float IS_IT_INSIDE_THIS_THING_OR_NOT (const Eigen::Matrix<float, kDimension, 1> pos_point, const Eigen::Matrix<float, kDimension, 1> pos_geometry, const Eigen::Matrix<float, kDimension, 1> norm_geometry) {
    // Assume planes
    Eigen::Matrix<float, kDimension, 1> vec = (pos_point - pos_geometry);
    vec.normalize();
    return (float) std::signbit(vec.dot(norm_geometry));
}

// HARD-CODED for 3D:
template <int i, int j, typename Matrix>
auto minorDet(const Matrix &m) {
    return m(i == 0 ? 1 : 0, j == 0 ? 1 : 0) * m(i == kDimension - 1 ? i - 1 : kDimension - 1, j == kDimension - 1 ? j - 1 : kDimension - 1)
         - m(i == 0 ? 1 : 0, j == kDimension - 1 ? j - 1 : kDimension - 1) * m(i == kDimension - 1 ? i - 1 : kDimension - 1, j == 0 ? 1 : 0);
};

void Loop(display::Viewport::Window* window) {
    window->Init(1280, 720); // originally 640, 480
    CameraControls<float> controls(camera, window->GetGLFWWindow());
    controls.SetCurrent();

    GLuint sampleBuffer;
    glGenBuffers(1, &sampleBuffer);

    Shader vertexShader(GL_VERTEX_SHADER, R"(
        #version 130

        uniform mat4 viewProjectionMatrix;
        in vec3 position;
        out vec3 color;

        void main() {
            color = abs(normalize(position.xyz));
            gl_Position = viewProjectionMatrix * vec4(position, 1.0);
        }
    )");

    Shader fragmentShader(GL_FRAGMENT_SHADER, R"(
        #version 130

        in vec3 color;

        void main() {
            gl_FragColor = vec4(color, 1.0);
        }
    )");

    ShaderProgram shaderProgram(vertexShader, fragmentShader);
    shaderProgram.GetAttributeLocations("position");
    shaderProgram.GetUniformLocations("viewProjectionMatrix");

    int width, height;
    glfwGetFramebufferSize(window->GetGLFWWindow(), &width, &height);

    glClearColor(0.f, 0.f, 0.f, 0.f);
    glViewport(0, 0, width, height);
    glEnable(GL_DEPTH_TEST);

    shaderProgram.Use();

    const auto& particlePositions = particles.GetAttributeList<SimulationAttribute::Position>();

    while(!window->ShouldClose()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUniformMatrix4fv(shaderProgram.Uniform("viewProjectionMatrix"), 1, GL_FALSE, reinterpret_cast<const float*>(camera.GetViewProjection().data()));

        glBindBuffer(GL_ARRAY_BUFFER, sampleBuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * kDimension * particlePositions.size(), particlePositions.data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(shaderProgram.Attribute("position"));
        glVertexAttribPointer(shaderProgram.Attribute("position"), kDimension, GL_FLOAT, false, 0, 0);

        glDrawArrays(GL_POINTS, 0, particlePositions.size());

        glDisableVertexAttribArray(shaderProgram.Attribute("position"));

        window->SwapBuffers();

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(16ms);
    }

    glDeleteBuffers(1, &sampleBuffer);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Expected at least one argument" << std::endl;
        return 1;
    }

    float density = 30.f; // usually the range is between 2e0 to 2e2

    if (argc >= 3) {
        density = stof(std::string(argv[2]));
    }

    // Create and initialize the viewport thread
    auto* viewport = new display::Viewport();
    std::thread viewportThread(Loop, viewport->GetWindow());

    // Load in the input mesh
    using Triangle = geometry::Triangle<kDimension, float>;
    auto* mesh = new geometry::Mesh<kDimension, Triangle>();
    import::TriangleObjLoader<float> loader;
    loader.Load(argv[1], mesh);

    // Compute the scene bounding box
    Optional<AxisAlignedBox<kDimension, float>> boundingBox;
    for (const auto* geometry : mesh->geometries()) {
        Merge(boundingBox, geometry->getAxisAlignedBound());
    }
    auto largestLength = boundingBox->Extent(boundingBox->GreatestExtent());

    // Point the camera at the bounding box
    camera.LookAt( (boundingBox->max() + boundingBox->min()) / 2.f );
    camera.SetAspectRatio(1280.f / 720.f);
    camera.SetFieldOfView(60.f * static_cast<float>(kPI) / 180.f);
    camera.SetNearFar(0.1f, 2000.f);
    camera.SetDistance(2.5f * largestLength);
    camera.Rotate(-70.f * static_cast<float>(kPI) / 180.f, 0);

    using GridIndex = Eigen::Array<unsigned int, kDimension, 1>;

    // Initialize a grid to twice the size of the bounding box
    Eigen::Array<float, kDimension, 1> gridSize = (boundingBox->max() - boundingBox->min()) * 1.5f;
    Eigen::Array<float, kDimension, 1> gridOrigin = (boundingBox->max() + boundingBox->min()) / 2.f - gridSize / 2.f;
    grid.Resize(largestLength / density / std::sqrt(kDimension), gridSize);

    // Generate samples inside the mesh
    float coverage;
    auto samples = sampling::SampleMesh<float>(mesh, largestLength / density, &coverage);
    delete mesh; // mesh is no longer needed

    particles.Resize(samples.size());

    auto extent = boundingBox->max() - boundingBox->min();
    float initialParticleVolume = extent[0] * extent[1] * extent[2] * coverage / static_cast<float>(samples.size());
    std::cout << "Particle bounding box: " << extent.transpose() << " m" << std::endl;

    std::cout << "Particle volume: " << initialParticleVolume << " m^3" << std::endl;
    auto& particleVolumes = particles.GetAttributeList<SimulationAttribute::InitialVolume>();
    core::VectorUtils::ApplyOverElements(particleVolumes, [&](auto& particleVolume) {
        particleVolume = initialParticleVolume;
    });

    float particleDensity = 1000.f;
    std::cout << "Particle density: " << particleDensity << " kg / m^3" << std::endl;
    float particleMass = initialParticleVolume * particleDensity;
    std::cout << "Particle mass: " << particleMass << " kg" << std::endl;
    auto& particleMasses = particles.GetAttributeList<SimulationAttribute::Mass>();
    core::VectorUtils::ApplyOverElements(particleMasses, [&](auto& mass) {
        mass = particleMass;
    });

    // Initialize particle velocities to 0
    auto& particleVelocities = particles.GetAttributeList<SimulationAttribute::Velocity>();
    core::VectorUtils::ApplyOverElements(particleVelocities, [](auto& velocity) {
        velocity = AttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity>::type::Zero();
    });

    // Initialize particle deformations to identity
    auto& particleDeformations = particles.GetAttributeList<SimulationAttribute::Deformation>();
    core::VectorUtils::ApplyOverElements(particleDeformations, [](auto& deformation) {
        deformation = AttributeDefinitions::AttributeInfo<SimulationAttribute::Deformation>::type::Identity();
    });

    // Set particle positions as sample positions
    auto& particlePositions = particles.GetAttributeList<SimulationAttribute::Position>();
    core::VectorUtils::ApplyOverIndices(particlePositions, [&](unsigned int p) {
        particlePositions[p] = samples[p].matrix();
    });

    // Initialize grid forces to 0
    auto& gridForces = grid.GetGrid<SimulationAttribute::Force>();
    gridForces.ApplyOverCells([&](auto& gridForce) {
        gridForce = AttributeDefinitions::AttributeInfo<SimulationAttribute::Force>::type::Zero();
    });

     // Initialize grid masses to 0
     auto& gridMasses = grid.GetGrid<SimulationAttribute::Mass>();
     gridMasses.ApplyOverCells([&](auto& gridMass) {
         gridMass = 0.f;
     });

     {
         // Just get an initial count of number of particles in each cell
         AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, GridIndex offset, GridIndex i) {
             auto* gridMass = gridMasses.at(i);
             if (gridMass) {
                 *gridMass += 1.0;
             }
         });

         float count = 0.f;
         float cells = 0.f;
         // Reset masses back to zero
         gridMasses.ApplyOverCells([&](auto& gridMass) {
             count += gridMass;
             cells++;
             gridMass = 0.f;
         });

         std::cout << count / cells << " particles per cell" << std::endl;
     }

    // Wait in case the viewport has not yet been initialized
    while (!viewport->GetWindow()->IsInitialized()) {
        std::this_thread::sleep_for(simulationTimestep);
    }

    // Step simulation until the viewport closes
    unsigned int step = 0;
    unsigned int substeps = 4;
    float stepSize = simulationTimestep.count() / 1000.f / static_cast<float>(substeps);
    while (!viewport->GetWindow()->ShouldClose()) {
        auto start = std::chrono::system_clock::now();

        for (unsigned int substep = 0; substep < substeps; ++substep) {
            auto& gridVelocities = grid.GetGrid<SimulationAttribute::Velocity>();
            auto& gridForces = grid.GetGrid<SimulationAttribute::Force>();
            auto& gridMasses = grid.GetGrid<SimulationAttribute::Mass>();


            // Update grid weights
            auto& particleWeights = particles.GetAttributeList<SimulationAttribute::Weights>();
            core::VectorUtils::ApplyOverIndices(particleWeights, [&](unsigned int p) {
                particleWeights[p].fillWeights(particlePositions[p], grid.CellSize(), gridOrigin);
            });

            /***************************
            ****************************
            ******* Patricle2Grid ******
            ****************************
            ****************************/

            gridMasses.ApplyOverCells([](auto& gridMass) {
                gridMass = 0.f;
            });

            gridVelocities.ApplyOverCells([](auto& gridVelocity) {
                gridVelocity = AttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity>::type::Zero();
            });

            gridForces.ApplyOverCells([](auto& gridForce) {
                gridForce = AttributeDefinitions::AttributeInfo<SimulationAttribute::Force>::type::Zero();
            });

            AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, GridIndex offset, GridIndex i) {
                auto* gridMass = gridMasses.at(i);
                if (gridMass) {
                    *gridMass += particleMasses[p] * particleWeights[p].getWeight(offset);
                }
            });

            AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, GridIndex offset, GridIndex i) {
                auto* gridVelocity = gridVelocities.at(i);
                auto* gridMass = gridMasses.at(i);

                // if grid location exits in grid then alter it
                if (gridVelocity && gridMass && fabs(*gridMass) >= 0.000001) {
                    *gridVelocity += ((particleWeights[p].getWeight(offset) * particleMasses[p] * particleVelocities[p]) / *gridMass);
                }
            });

            /***************************
            ****************************
            ***** UPDATE ATTRIBUTES ****
            ****************************
            ****************************/

            auto& particleStresses = particles.GetAttributeList<SimulationAttribute::Stress>();
            core::VectorUtils::ApplyOverIndices(particleStresses, [&](unsigned int p) {
                auto& stress = particleStresses[p];
                const auto& F = particleDeformations[p];
                auto J = F.determinant();
                Eigen::JacobiSVD<Eigen::Matrix<float, kDimension, kDimension>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Matrix<float, kDimension, kDimension> U = svd.matrixU();
                Eigen::Matrix<float, kDimension, 1> Sigma = svd.singularValues();
                Eigen::Matrix<float, kDimension, kDimension> V = svd.matrixV();

                Eigen::Matrix<float, kDimension, 1> temp;
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
                    U.col(kDimension - 1) *= -1;
                    Sigma(kDimension - 1, 0) *= -1;
                }
                if (V.determinant() < 0) {
                    V.col(kDimension - 1) *= -1;
                    Sigma(kDimension - 1, 0) *= -1;
                }

                Eigen::Matrix<float, kDimension, kDimension> R = U * V;

                constexpr float YoungsModulus = static_cast<float>(1e2); // around 3e1 to 3e3
                constexpr float PoissonRatio = 0.33f; // usually it's arround 0.3 - 0.4
                constexpr float mu = YoungsModulus / (2 * (1 + PoissonRatio));
                constexpr float lambda = YoungsModulus * PoissonRatio / ((1 + PoissonRatio) * (1 - 2 * PoissonRatio));

                // HARD-CODED for 3D:
                Eigen::Matrix<float, kDimension, kDimension> jFInvTranspose;
                jFInvTranspose <<
                     minorDet<0, 0>(F), -minorDet<0, 1>(F),  minorDet<0, 2>(F),
                    -minorDet<1, 0>(F),  minorDet<1, 1>(F), -minorDet<1, 2>(F),
                     minorDet<2, 0>(F), -minorDet<2, 1>(F),  minorDet<2, 2>(F);

                stress = 2 * mu * (F - R) + lambda * (J - 1) * jFInvTranspose; // corotated
            });

            AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, GridIndex offset, GridIndex i) {
                auto* gridForce = gridForces.at(i);
                if (gridForce) {
                    *gridForce -= particleVolumes[p] * particleStresses[p] * particleDeformations[p].transpose() * particleWeights[p].getWeightGradient(offset, grid.CellSize());
                }
            });

            gridVelocities.ApplyOverIndices([&](const auto& i) {
                if (fabs(gridMasses[i]) >= 0.000001) {
                    gridVelocities[i] += (stepSize * gridForces[i] / gridMasses[i]);
                }
            });

            gridVelocities.ApplyOverIndices([&](const auto& i) {
                gridVelocities[i][1] -= 9.80665 * stepSize; // gravity -2
            });

            // Collisions

            // Iterate over each plane making up the grid
            // yes, this will double-count some grid cells on the border of two planes (the edges of the grid)
            // currently implenents an inelastic bounce

            auto gridVelocityDims = gridVelocities.GetSizes();
            // X-planes
            for (unsigned int z = 0; z < gridVelocityDims[2]; ++z) {
                for (unsigned int y = 0; y < gridVelocityDims[1]; ++y) {
                    const Eigen::Array<unsigned int, kDimension, 1> index3D_min = { 0, y, z };
                    const Eigen::Array<unsigned int, kDimension, 1> index3D_max = { gridVelocityDims[0] - 1, y, z };

                    if (gridVelocities[index3D_min][0] < 0.f) {
                        gridVelocities[index3D_min][0] = 0.f;
                    }

                    if (gridVelocities[index3D_max][0] > 0.f) {
                        gridVelocities[index3D_max][0] = 0.f;
                    }
                }
            }

            // Y-planes
            for (unsigned int x = 0; x < gridVelocityDims[0]; ++x) {
                for (unsigned int z = 0; z < gridVelocityDims[2]; ++z) {
                    const Eigen::Array<unsigned int, kDimension, 1> index3D_min = { x, 0, z };
                    const Eigen::Array<unsigned int, kDimension, 1> index3D_max = { x, gridVelocityDims[1] - 1, z };

                    if (gridVelocities[index3D_min][1] < 0.f) {
                        gridVelocities[index3D_min][1] = 0.f;
                    }

                    if (gridVelocities[index3D_max][1] > 0.f) {
                        gridVelocities[index3D_max][1] = 0.f;
                    }
                }
            }

            // Z-planes
            for (unsigned int x = 0; x < gridVelocityDims[0]; ++x) {
                for (unsigned int y = 0; y < gridVelocityDims[1]; ++y) {
                    const Eigen::Array<unsigned int, kDimension, 1> index3D_min = { x, y, 0 };
                    const Eigen::Array<unsigned int, kDimension, 1> index3D_max = { x, y, gridVelocityDims[2] - 1 };

                    if (gridVelocities[index3D_min][2] < 0.f) {
                        gridVelocities[index3D_min][2] = 0.f;
                    }

                    if (gridVelocities[index3D_max][2] > 0.f) {
                        gridVelocities[index3D_max][2] = 0.f;
                    }
                }
            }


            /***************************
            ****************************
            ******* Grid2Particle ******
            ****************************
            ****************************/

            // Transfer back to particles
            // Velocity
            core::VectorUtils::ApplyOverElements(particleVelocities, [](auto &particleVelocity) {
                particleVelocity = AttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity>::type::Zero();
            });

            AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, GridIndex offset, GridIndex i) {
                auto* gridVelocity = gridVelocities.at(i);
                if (gridVelocity) {
                    particleVelocities[p] += gridVelocities[i] * particleWeights[p].getWeight(offset);
                }
            });

            // Update positions
            core::VectorUtils::ApplyOverIndices(particlePositions, [&](unsigned int p) {
                particlePositions[p] += particleVelocities[p] * stepSize;
            });

            // Clamp positions
            core::VectorUtils::ApplyOverElements(particlePositions, [&](auto &particlePosition) {
                particlePosition = gridOrigin.max(particlePosition.array()).min(gridOrigin + gridSize).matrix();
            });

            // Update particle force
            auto& deformationUpdates = particles.GetAttributeList<SimulationAttribute::DeformationUpdate>();
            core::VectorUtils::ApplyOverElements(deformationUpdates, [](auto& deformationUpdate) {
                deformationUpdate = AttributeDefinitions::AttributeInfo<SimulationAttribute::DeformationUpdate>::type::Zero();
            });
            AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, GridIndex offset, GridIndex i) {
                auto* gridVelocity = gridVelocities.at(i);
                if (gridVelocity) {
                    deformationUpdates[p] += stepSize * gridVelocities[i] * particleWeights[p].getWeightGradient(offset, grid.CellSize()).transpose();
                }
            });
            core::VectorUtils::ApplyOverIndices(particleDeformations, [&](unsigned int p) {
                particleDeformations[p] += deformationUpdates[p] * particleDeformations[p];
            });
        }

        auto end = std::chrono::system_clock::now();
        auto elapsed = end - start;
        if (elapsed < simulationTimestep) {
            std::this_thread::sleep_for(simulationTimestep - elapsed);
        }
        step++;
    }

    // Wait for the viewport thread to finish
    viewportThread.join();
    delete viewport;

    return 0;
}
