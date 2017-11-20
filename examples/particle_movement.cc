
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
    SimulationAttribute::Force
> particles;

// Declare the grid attributes
simulation::AttributeGrid<kDimension, float, SimulationAttribute, AttributeDefinitions,
	SimulationAttribute::Position,
    SimulationAttribute::Velocity,
	SimulationAttribute::Mass,
    SimulationAttribute::Force
> grid;

void Loop(display::Viewport::Window* window) {
    window->Init(640, 480);
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

    float density = 30.f;

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
    camera.SetAspectRatio(640.f / 480.f);
    camera.SetFieldOfView(60.f * static_cast<float>(kPI) / 180.f);
    camera.SetNearFar(0.1f, 2000.f);
    camera.SetDistance(1.5f * largestLength);
    camera.Rotate(-70.f * static_cast<float>(kPI) / 180.f, 0);

    using GridIndex = Eigen::Array<unsigned int, kDimension, 1>;
        
    // Initialize a grid to twice the size of the bounding box
    Eigen::Array<float, kDimension, 1> gridSize = (boundingBox->max() - boundingBox->min()) * 2.f;
    Eigen::Array<float, kDimension, 1> gridOrigin = (boundingBox->max() + boundingBox->min()) / 2.f - gridSize / 2.f;
    grid.Resize(2.f / density, gridSize);
    
    // Generate samples inside the mesh
    float coverage;
    auto samples = sampling::SampleMesh<float>(mesh, largestLength / density, &coverage);
    delete mesh; // mesh is no longer needed

    particles.Resize(samples.size());

    auto extent = boundingBox->max() - boundingBox->min();
    float initialParticleVolume = extent[0] * extent[1] * extent[2] * coverage / static_cast<float>(samples.size());

    auto& particleVolumes = particles.GetAttributeList<SimulationAttribute::InitialVolume>();
    core::VectorUtils::ApplyOverElements(particleVolumes, [&](auto& particleVolume) {
        particleVolume = 1.f;
    });

    // Initialize particle masses to 1
    auto& particleMasses = particles.GetAttributeList<SimulationAttribute::Mass>();
    core::VectorUtils::ApplyOverElements(particleMasses, [](auto& mass) {
        mass = 1.f;
    });

    // Initialize particle velocities to 0
    auto& particleVelocities = particles.GetAttributeList<SimulationAttribute::Velocity>();
    core::VectorUtils::ApplyOverElements(particleVelocities, [](auto& velocity) {
        velocity = { 0, 0, 0 };
    });

    // Initialize particle forces to 0
    auto& particleForces = particles.GetAttributeList<SimulationAttribute::Force>();
    core::VectorUtils::ApplyOverElements(particleForces, [](auto& force) {
        force = { 0, 0, 0 };
    });

    // Set particle positions as sample positions
    auto& particlePositions = particles.GetAttributeList<SimulationAttribute::Position>();
    core::VectorUtils::ApplyOverIndices(particlePositions, [&](unsigned int p) {
        particlePositions[p] = samples[p].matrix();
    });

	// Initialize grid weights for each particle
    auto& particleWeights = particles.GetAttributeList<SimulationAttribute::Weights>();
    core::VectorUtils::ApplyOverIndices(particleWeights, [&](unsigned int p) {
        particleWeights[p].fillWeights(particlePositions[p], grid.CellSize(), gridOrigin);
    });

    // Initialize grid forces to 0
    auto& gridForces = grid.GetGrid<SimulationAttribute::Force>();
    gridForces.ApplyOverCells([&](auto& gridForce) {
        gridForce = { 0, 0, 0 };
    });
   
     // Initialize grid masses to 0
     auto& gridMasses = grid.GetGrid<SimulationAttribute::Mass>();
     gridMasses.ApplyOverCells([&](auto& gridMass) {
         gridMass = 0.f;
     });

    // Wait in case the viewport has not yet been initialized
    while (!viewport->GetWindow()->IsInitialized()) {
        std::this_thread::sleep_for(simulationTimestep);
    }

    // Step simulation until the viewport closes
    unsigned int step = 0;
    float stepSize = simulationTimestep.count() / 1000.0f;
    while (!viewport->GetWindow()->ShouldClose()) {
        auto start = std::chrono::system_clock::now();

        auto& gridVelocities = grid.GetGrid<SimulationAttribute::Velocity>();
        auto& gridForces = grid.GetGrid<SimulationAttribute::Force>();
        auto& gridMasses = grid.GetGrid<SimulationAttribute::Mass>();

        /***************************
        ****************************
        ******* Patricle2Grid ******
        ****************************
        ****************************/

        gridMasses.ApplyOverCells([](auto& gridMass) {
            gridMass = 0.f;
        });

        gridVelocities.ApplyOverCells([](auto& gridVelocity) {
            gridVelocity = { 0, 0, 0 };
        });

        gridForces.ApplyOverCells([](auto& gridForce) {
            gridForce = { 0, 0, 0 };
        });

        AttributeTransfer::ParticleToGrid<SimulationAttribute::Mass>(particles, grid, gridOrigin);

        // Replaced by loop below
        // AttributeTransfer::ParticleToGrid<SimulationAttribute::Velocity>(particles, grid, gridOrigin);
        AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, float weight, Eigen::Array<float, kDimension, 1> weightGrad, GridIndex offset, GridIndex i) {
            auto* gridVelocity = gridVelocities.at(i);
            auto* gridMass = gridMasses.at(i);

            // if grid location exits in grid then alter it
            if (gridVelocity && gridMass && *gridMass != 0) {
                *gridVelocity += ((weight * particleMasses[p] * particleVelocities[p]) / *gridMass);
            }
        });

        /***************************
        ****************************
        ***** UPDATE ATTRIBUTES ****
        ****************************
        ****************************/

        /*AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, float weight, Eigen::Array<float, kDimension, 1> weightGrad, GridIndex offset, GridIndex i) {
            auto* gridVelocity = gridVelocities.at(i);
            auto* gridForce    = gridForces.at(i);
            auto* gridMass     = gridMasses.at(i);

            // if grid location exits in grid then alter it
            if (gridVelocity && gridMass) {
               *gridVelocity += (*gridForce / *gridMass) * stepSize;
            }
        });*/
        

        // still need to update force - see hannah's notes
        // do that above the force
        // velocity is computed above already

        // then do boundary condition - inelastic bounce? choose type?
        // do condition: if boundary grid node is inside an object, the object is outside of the grid, aka set that grid node's velocity to 0...how do you do that?????
        /******* TODO: Uncomment this once you figure out how to check for grid boundary  ****/
        // AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, float weight, GridIndex offset, GridIndex i) {
        //     auto* gridPosition   = gridPositions.at(i);
        //     auto* gridVelocities = gridVelocities.at(i); 

        //     // if grid location exits in grid then alter it
        //     if (gridPosition && outOfBounds(gridPosition)) {
        //         *gridVelocity = 0;
        //     }
        // });

        AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, float weight, Eigen::Matrix<float, kDimension, 1> weightGrad, GridIndex offset, GridIndex i) {
            auto* gridForce = gridForces.at(i);
            if (gridForce) {
                auto P = Eigen::Matrix<float, kDimension, 1>();
                *gridForce -= particleVolumes[p] * P * particleForces[p].transpose() * weightGrad;
            }
        });

        gridVelocities.ApplyOverIndices([&](const auto& i) {
            if (gridMasses[i] != 0) { // OPTIMIZE later
                gridVelocities[i] += stepSize * gridForces[i] / gridMasses[i];
            }
        });

        gridVelocities.ApplyOverIndices([&](const auto& i) {
            gridVelocities[i][1] -= 9.80665 * stepSize;
        });

        /***************************
        ****************************
        ******* Grid2Particle ******
        ****************************
        ****************************/

        // Transfer back to particles
        // Velocity
        AttributeTransfer::GridToParticle<SimulationAttribute::Velocity>(grid, gridOrigin, particles);
        //AttributeTransfer::GridToParticle<SimulationAttribute::Position>(grid, gridOrigin, particles);

        // Update positions
        core::VectorUtils::ApplyOverIndices(particlePositions, [&](unsigned int p) {
            particlePositions[p] += particleVelocities[p] * stepSize;
        });

        // Update particle force
        auto& deformationUpdates = particles.GetAttributeList<SimulationAttribute::DeformationUpdate>();
        core::VectorUtils::ApplyOverElements(deformationUpdates, [](auto& deformationUpdate) {
            deformationUpdate = Eigen::Matrix<float, kDimension, kDimension>::Zero();
        });
        AttributeTransfer::IterateParticleKernel(particles, grid, gridOrigin, [&](unsigned int p, float weight, Eigen::Matrix<float, kDimension, 1> weightGrad, GridIndex offset, GridIndex i) {
            auto* gridVelocity = gridVelocities.at(i);
            if (gridVelocity) {
                deformationUpdates[p] += stepSize * gridVelocities[i] * weightGrad.transpose();
            }
        });
        core::VectorUtils::ApplyOverIndices(particleForces, [&](unsigned int p) {
            particleForces[p] += deformationUpdates[p] * particleForces[p];
        });
        
		// Update grid weights
        core::VectorUtils::ApplyOverIndices(particleWeights, [&](unsigned int p) {
            particleWeights[p].fillWeights(particlePositions[p], grid.CellSize(), gridOrigin);
        });

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
