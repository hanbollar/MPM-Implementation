
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
#include "flint_viewport/gl/ShaderProgram.h"

using namespace core;

const float* sampleData = nullptr;
unsigned int sampleCount = 0;
Camera<float> camera;

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

    while(!window->ShouldClose()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUniformMatrix4fv(shaderProgram.Uniform("viewProjectionMatrix"), 1, GL_FALSE, reinterpret_cast<const float*>(camera.GetViewProjection().data()));

        glBindBuffer(GL_ARRAY_BUFFER, sampleBuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * sampleCount, sampleData, GL_STATIC_DRAW);

        glEnableVertexAttribArray(shaderProgram.Attribute("position"));
        glVertexAttribPointer(shaderProgram.Attribute("position"), 3, GL_FLOAT, false, 0, 0);

        glDrawArrays(GL_POINTS, 0, sampleCount);

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
    using Triangle = geometry::Triangle<3, float>;
    auto* mesh = new geometry::Mesh<3, Triangle>();
    import::TriangleObjLoader<float> loader;
    loader.Load(argv[1], mesh);

    // Compute the scene bounding box
    Optional<AxisAlignedBox<3, float>> boundingBox;
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

    // Generate samples inside the mesh
    auto samples = sampling::SampleMesh<float>(mesh, largestLength / density);
    sampleData = reinterpret_cast<const float*>(samples.data());
    sampleCount = static_cast<unsigned int>(samples.size());
    delete mesh; // mesh is no longer needed

    static constexpr auto timestep = std::chrono::duration<double, std::milli>(4);

    // Wait in case the viewport has not yet been initialized
    while (!viewport->GetWindow()->IsInitialized()) {
        std::this_thread::sleep_for(timestep);
    }

    // Step simulation until the viewport closes
    unsigned int step = 0;
    while (!viewport->GetWindow()->ShouldClose()) {
        auto start = std::chrono::system_clock::now();
        for (auto& sample : samples) {
            sample[1] += 0.02 * std::sin(0.05 * step);
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed = end - start;
        if (elapsed < timestep) {
            std::this_thread::sleep_for(timestep - elapsed);
        }
        step++;
    }

    // Wait for the viewport thread to finish
    viewportThread.join();
    delete viewport;

    return 0;
}
