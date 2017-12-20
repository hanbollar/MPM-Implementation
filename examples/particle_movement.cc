
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
#include "simulation/MPMSimulation.h"

using namespace core;

Camera<float> camera;
constexpr unsigned int kDimension = 3; // 3D simulation
static constexpr auto simulationTimestep = std::chrono::duration<double, std::milli>(16); // timestep for simulation
simulation::MPM::MPMSimulation<float, kDimension> sim;

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

    const auto& particlePositions = sim.GetParticleAttribute<simulation::MPM::SimulationAttribute::Position>();

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
    camera.LookAt((boundingBox->max() + boundingBox->min()) / 2.f);
    camera.SetAspectRatio(1280.f / 720.f);
    camera.SetFieldOfView(60.f * static_cast<float>(kPI) / 180.f);
    camera.SetNearFar(0.1f, 2000.f);
    camera.SetDistance(2.5f * largestLength);
    camera.Rotate(-70.f * static_cast<float>(kPI) / 180.f, 0);

    Eigen::Array<float, kDimension, 1> gridSize = (boundingBox->max() - boundingBox->min()) * 1.5f;
    Eigen::Array<float, kDimension, 1> gridOrigin = (boundingBox->max() + boundingBox->min()) / 2.f - gridSize / 2.f;

    // Generate samples inside the mesh
    float coverage;
    auto samples = sampling::SampleMesh<float>(mesh, largestLength / density, &coverage);
    delete mesh; // mesh is no longer needed
    float particleVolume = boundingBox->Volume() * coverage / static_cast<float>(samples.size());
    float cellSize = largestLength / density / std::sqrt(kDimension);
    
    float YoungsModulus = 1500.f; // around 3e1 to 3e3
    float PoissonRatio = 0.3f; // usually it's arround 0.3 - 0.4
    float materialDensity = 30.f; // 2 for snow
    // gravity = -1.5 in the y direction

    sim.SetBounds(gridOrigin, gridOrigin + gridSize);
    sim.SetCellSize(cellSize);
    sim.SetMaxdt(1e-4);

    simulation::MPM::MPMSimulation<float, kDimension>::ParticleSet particles;
    particles.Resize(samples.size());
    for (unsigned int p = 0; p < samples.size(); ++p) {
        particles.Get<simulation::MPM::SimulationAttribute::Position>()[p] = samples[p].matrix();
        particles.Get<simulation::MPM::SimulationAttribute::Position>()[p][1] += (gridSize[1] - boundingBox->Extent(1)) / 2.f - 2.f * cellSize;
        particles.Get<simulation::MPM::SimulationAttribute::Volume>()[p] = particleVolume;
        particles.Get<simulation::MPM::SimulationAttribute::Mass>()[p] = particleVolume * materialDensity;
        particles.Get<simulation::MPM::SimulationAttribute::mu>()[p] = YoungsModulus / (2 * (1 + PoissonRatio));
        particles.Get<simulation::MPM::SimulationAttribute::lambda>()[p] = YoungsModulus * PoissonRatio / ((1 + PoissonRatio) * (1 - 2 * PoissonRatio));
        particles.Get<simulation::MPM::SimulationAttribute::mu_0>()[p] = YoungsModulus / (2 * (1 + PoissonRatio));
        particles.Get<simulation::MPM::SimulationAttribute::lambda_0>()[p] = YoungsModulus * PoissonRatio / ((1 + PoissonRatio) * (1 - 2 * PoissonRatio));
        particles.Get<simulation::MPM::SimulationAttribute::Jp_snow>()[p] = 1;
    }

    sim.AddParticles(particles);
    sim.Init();

    // Wait in case the viewport has not yet been initialized
    while (!viewport->GetWindow()->IsInitialized()) {
        std::this_thread::sleep_for(simulationTimestep);
    }

    unsigned int frame = 0;
    while (!viewport->GetWindow()->ShouldClose()) {
        auto start = std::chrono::system_clock::now();

        float t = 0;
        while ((t += sim.Step()) < simulationTimestep.count() / 1000.f);

        auto end = std::chrono::system_clock::now();
        auto elapsed = end - start;
        //std::cout << "frame " << frame << ": " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << std::endl;
        if (elapsed < simulationTimestep) {
            std::this_thread::sleep_for(simulationTimestep - elapsed);
        }
        frame++;
    }

    // Wait for the viewport thread to finish
    viewportThread.join();
    delete viewport;
    return 0;
}
