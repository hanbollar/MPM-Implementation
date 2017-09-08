
#define GL_GLEXT_PROTOTYPES

#include <iostream>
#include <thread>
#include <chrono>
#include <GL/gl.h>
#include "flint/accel/bvh/TreeBuilder.h"
#include "flint/accel/bvh/Tree.h"
#include "flint/core/AxisAlignedBox.h"
#include "flint/core/Optional.h"
#include "flint/geometry/Triangle.h"
#include "flint/import/ObjLoader.h"
#include "flint/intersection/Ray.h"
#include "flint/intersection/bvh/Tree.h"
#include "flint/sampling/Box.h"
#include "flint/sampling/Mesh.h"
#include "flint_viewport/viewport.h"

using namespace core;

const float* sampleData = nullptr;
unsigned int sampleCount = 0;

void Loop(display::Viewport::Window* window) {
    window->Init();

    GLuint sampleBuffer;
    glGenBuffers(1, &sampleBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, sampleBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * sampleCount, sampleData, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    static std::string vsSource = R"(
        #version 330
        uniform mat4 viewProjectionMatrix;
        layout(location = 0) in vec3 position;
        void main() {
            gl_Position = viewProjectionMatrix * vec4(position, 1.0);
        }
    )";

    static std::string fsSource = R"(
        #version 330

        layout(location = 0) out vec4 fragColor;
        void main() {
            fragColor = vec4(1.0);
        }
    )";

    const char* source;
    int length;

    source = vsSource.c_str();
    length = static_cast<int>(vsSource.size());
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &source, &length);
    glCompileShader(vs);

    source = fsSource.c_str();
    length = static_cast<int>(fsSource.size());
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &source, &length);
    glCompileShader(fs);

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vs);
    glAttachShader(shaderProgram, fs);
    glLinkProgram(shaderProgram);

    GLuint viewProjectionMatrixLocation = glGetUniformLocation(shaderProgram, "viewProjectionMatrix");

    glUseProgram(shaderProgram);

    auto mvp = Eigen::Matrix4f::Identity();
    glUniformMatrix4fv(viewProjectionMatrixLocation, 1, GL_FALSE, reinterpret_cast<const float*>(&mvp));

    while(!window->ShouldClose()) {
        glBindBuffer(GL_ARRAY_BUFFER, sampleBuffer);
        glDrawArrays(GL_POINTS, 0, sampleCount);

        std::cout << window->GetFrameNumber() << std::endl;

        window->SwapBuffers();

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(16ms);
    }

    glDeleteBuffers(1, &sampleBuffer);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected one argument" << std::endl;
        return 1;
    }

    using Triangle = geometry::Triangle<3, float>;

    auto* mesh = new geometry::Mesh<3, Triangle>();
    import::TriangleObjLoader<float> loader;
    loader.Load(argv[1], mesh);

    Optional<AxisAlignedBox<3, float>> boundingBox;
    for (const auto* geometry : mesh->geometries()) {
        Merge(boundingBox, geometry->getAxisAlignedBound());
    }

    auto samples = sampling::SampleMesh<float>(mesh, boundingBox->Extent(boundingBox->GreatestExtent()) / 10.f);
    sampleData = reinterpret_cast<const float*>(samples.data());
    sampleCount = samples.size();

    delete mesh;

    auto* viewport = new display::Viewport();
    std::thread viewportThread(Loop, viewport->GetWindow());

    viewport->Close();
    viewportThread.join();

    delete viewport;

    return 0;
}
