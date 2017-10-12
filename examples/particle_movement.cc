
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
#include "flint/geometry/Triangle.h"
#include "flint/import/ObjLoader.h"
#include "flint/intersection/Ray.h"
#include "flint/intersection/bvh/Tree.h"
#include "flint/sampling/Box.h"
#include "flint/sampling/Mesh.h"
#include "flint_viewport/viewport.h"
#include "flint_viewport/CameraControls.h"
#include "simulation/MACGrid.h"
#include "simulation/ParticleSet.h"

using namespace core;

enum class SimulationAttribute {
    Mass,
    Position,
    Velocity,
    Pressure,
};

struct GridAttributeDefinitions {
    template <SimulationAttribute A>
    struct AttributeInfo { };
};

template <>
struct GridAttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity> {
    using Info = simulation::GridAttributeInfo<simulation::MAC::Attribute::Interpolated, float>;
};

template <>
struct GridAttributeDefinitions::AttributeInfo<SimulationAttribute::Pressure> {
    using Info = simulation::GridAttributeInfo<simulation::MAC::Attribute::Grid, float>;
};

struct ParticleAttributeDefinitions {
    template <SimulationAttribute A>
    struct AttributeInfo { };
};

template <>
struct ParticleAttributeDefinitions::AttributeInfo<SimulationAttribute::Position> {
    using Info = simulation::ParticleAttributeInfo<Eigen::Array<float, 3, 1>>;
};

template <>
struct ParticleAttributeDefinitions::AttributeInfo<SimulationAttribute::Velocity> {
    using Info = simulation::ParticleAttributeInfo<Eigen::Array<float, 3, 1>>;
};

template <>
struct ParticleAttributeDefinitions::AttributeInfo<SimulationAttribute::Mass> {
    using Info = simulation::ParticleAttributeInfo<float>;
};

const float* sampleData = nullptr;
unsigned int sampleCount = 0;
Camera<float> camera;

void Loop(display::Viewport::Window* window) {
    window->Init(640, 480);
    CameraControls<float> controls(camera, window->GetGLFWWindow());
    controls.SetCurrent();

    GLuint sampleBuffer;
    glGenBuffers(1, &sampleBuffer);

    static std::string vsSource = R"(
        #version 130

        uniform mat4 viewProjectionMatrix;
        in vec3 position;
        out vec3 color;

        void main() {
            color = abs(normalize(position.xyz));
            gl_Position = viewProjectionMatrix * vec4(position, 1.0);
        }
    )";

    static std::string fsSource = R"(
        #version 130

        in vec3 color;

        void main() {
            gl_FragColor = vec4(color, 1.0);
        }
    )";

    auto compileShader = [](GLenum type, const char* source, int length) {
        auto checkCompileStatus = [](GLuint shader) {
            GLint compileFlag;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &compileFlag);
            if (compileFlag == GL_FALSE) {
                GLint maxLength = 0;
                glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);
                std::vector<GLchar> errorLog(maxLength);
                glGetShaderInfoLog(shader, maxLength, &maxLength, &errorLog[0]);

                std::cerr << errorLog.data() << std::endl;
            }
        };

        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, &length);
        glCompileShader(shader);
        checkCompileStatus(shader);

        return shader;
    };

    GLuint vs = compileShader(GL_VERTEX_SHADER, vsSource.c_str(), static_cast<int>(vsSource.size()));
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSource.c_str(), static_cast<int>(fsSource.size()));

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vs);
    glAttachShader(shaderProgram, fs);
    glLinkProgram(shaderProgram);

    GLint linkFlag = 0;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &linkFlag);
    if (linkFlag == GL_FALSE) {
        GLint maxLength = 0;
        glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &maxLength);
        std::vector<GLchar> errorLog(maxLength);
        glGetProgramInfoLog(shaderProgram, maxLength, &maxLength, &errorLog[0]);

        std::cerr << errorLog.data() << std::endl;
    }

    GLint positionLocation = glGetAttribLocation(shaderProgram, "position");
    if (positionLocation < 0) {
        std::cerr << "Could not get shader attribute position" << std::endl;
    }

    GLint viewProjectionMatrixLocation = glGetUniformLocation(shaderProgram, "viewProjectionMatrix");
    if (viewProjectionMatrixLocation < 0) {
        std::cerr << "Could not get shader uniform viewProjectionMatrix" << std::endl;
    }

    int width, height;
    glfwGetFramebufferSize(window->GetGLFWWindow(), &width, &height);

    glClearColor(0.f, 0.f, 0.f, 0.f);
    glViewport(0, 0, width, height);
    glEnable(GL_DEPTH_TEST);

    while(!window->ShouldClose()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shaderProgram);

        glUniformMatrix4fv(viewProjectionMatrixLocation, 1, GL_FALSE, reinterpret_cast<const float*>(camera.GetViewProjection().data()));

        glBindBuffer(GL_ARRAY_BUFFER, sampleBuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * sampleCount, sampleData, GL_STATIC_DRAW);
        glEnableVertexAttribArray(positionLocation);
        glVertexAttribPointer(positionLocation, 3, GL_FLOAT, false, 0, 0);

        glDrawArrays(GL_POINTS, 0, sampleCount);

        glDisableVertexAttribArray(positionLocation);

        window->SwapBuffers();

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(16ms);
    }

    glDeleteBuffers(1, &sampleBuffer);
    glDeleteProgram(shaderProgram);
    glDeleteShader(vs);
    glDeleteShader(fs);
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

    using Triangle = geometry::Triangle<3, float>;

    auto* mesh = new geometry::Mesh<3, Triangle>();
    import::TriangleObjLoader<float> loader;
    loader.Load(argv[1], mesh);

    Optional<AxisAlignedBox<3, float>> boundingBox;
    for (const auto* geometry : mesh->geometries()) {
        Merge(boundingBox, geometry->getAxisAlignedBound());
    }

    auto largestLength = boundingBox->Extent(boundingBox->GreatestExtent());

    camera.LookAt( (boundingBox->max() + boundingBox->min()) / 2.f );
    camera.SetAspectRatio(640.f / 480.f);
    camera.SetFieldOfView(60.f * static_cast<float>(kPI) / 180.f);
    camera.SetNearFar(0.1f, 2000.f);
    camera.SetDistance(1.5f * largestLength);
    camera.Rotate(-70.f * static_cast<float>(kPI) / 180.f, 0);

    auto samples = sampling::SampleMesh<float>(mesh, largestLength / density);
    delete mesh;

    simulation::MACGrid<3, SimulationAttribute, GridAttributeDefinitions,
        SimulationAttribute::Velocity,
        SimulationAttribute::Pressure
    > grid(largestLength / density, boundingBox->Extent(0), boundingBox->Extent(1), boundingBox->Extent(2));

    auto& gridVelocities = grid.GetGrid<SimulationAttribute::Velocity>();
    auto& gridPressures = grid.GetGrid<SimulationAttribute::Pressure>();

    simulation::ParticleSet<SimulationAttribute, ParticleAttributeDefinitions,
        SimulationAttribute::Velocity,
        SimulationAttribute::Position,
        SimulationAttribute::Mass
    > particleSet(samples.size());

    auto& particlePositions = particleSet.GetAttributeList<SimulationAttribute::Position>();
    particlePositions = samples;

    auto& particleMasses = particleSet.GetAttributeList<SimulationAttribute::Mass>();
    auto& particleVelocities = particleSet.GetAttributeList<SimulationAttribute::Velocity>();

    for (unsigned int i = 0; i < particleSet.Size(); ++i) {
        particleMasses[i] = 10.f;
    }

    sampleData = reinterpret_cast<const float*>(particlePositions.data());
    sampleCount = static_cast<unsigned int>(particleSet.Size());

    auto* viewport = new display::Viewport();
    std::thread viewportThread(Loop, viewport->GetWindow());

    static constexpr auto timestep = std::chrono::duration<double, std::milli>(4);
    while (!viewport->GetWindow()->IsInitialized()) {
        std::this_thread::sleep_for(timestep);
    }

    unsigned int step = 0;
    while (!viewport->GetWindow()->ShouldClose()) {
        auto start = std::chrono::system_clock::now();
        for (auto& position : particlePositions) {
            position[1] += static_cast<float>(0.02 * std::sin(0.05 * step));
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed = end - start;
        if (elapsed < timestep) {
            std::this_thread::sleep_for(timestep - elapsed);
        }
        step++;
    }

    viewportThread.join();
    delete viewport;

    return 0;
}